#include "IntersectionManager.h"
#include "Sketch.h"
#include "SnapManager.h"
#include "SketchEntity.h"
#include "SketchLine.h"
#include "SketchArc.h"
#include "SketchCircle.h"
#include "SketchPoint.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>

namespace onecad::core::sketch {

IntersectionManager::IntersectionManager() = default;
IntersectionManager::~IntersectionManager() = default;

IntersectionResult IntersectionManager::processIntersections(
    EntityID newEntityId,
    Sketch& sketch,
    const SnapManager& snapManager)
{
    IntersectionResult result;

    if (!enabled_) {
        return result;
    }

    auto* newEntity = sketch.getEntity(newEntityId);
    if (!newEntity) {
        return result;
    }

    // Only process lines, arcs, circles for now
    if (newEntity->type() != EntityType::Line &&
        newEntity->type() != EntityType::Arc &&
        newEntity->type() != EntityType::Circle) {
        return result;
    }

    // Collect all existing entities (exclude the new one and points)
    std::vector<SketchEntity*> existingEntities;
    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->id() == newEntityId) {
            continue;
        }
        if (entity->type() == EntityType::Point) {
            continue;
        }
        if (entity->type() == EntityType::Line ||
            entity->type() == EntityType::Arc ||
            entity->type() == EntityType::Circle) {
            existingEntities.push_back(entity.get());
        }
    }

    // Find all intersection points
    std::vector<Vec2d> allIntersections;
    std::vector<EntityID> intersectedEntityIds;

    for (auto* existingEntity : existingEntities) {
        auto intersections = findIntersections(
            newEntity, existingEntity, sketch, snapManager);

        if (!intersections.empty()) {
            allIntersections.insert(
                allIntersections.end(),
                intersections.begin(),
                intersections.end());

            // Track which entity was intersected (for splitting)
            for (size_t i = 0; i < intersections.size(); ++i) {
                intersectedEntityIds.push_back(existingEntity->id());
            }
        }
    }

    if (allIntersections.empty()) {
        return result;
    }

    // Merge nearby points
    auto mergedPoints = mergeNearbyPoints(allIntersections, minPointSpacing_);
    result.intersectionPoints = mergedPoints;

    // Create points at each unique intersection (or use existing)
    std::vector<EntityID> intersectionPointIds;
    for (const auto& pt : mergedPoints) {
        EntityID existingPtId = findExistingPointAt(pt, sketch, minPointSpacing_);

        if (!existingPtId.empty()) {
            intersectionPointIds.push_back(existingPtId);
        } else {
            EntityID ptId = sketch.addPoint(pt.x, pt.y, false);
            if (!ptId.empty()) {
                intersectionPointIds.push_back(ptId);
                result.pointsCreated++;
            }
        }
    }

    // Build map of entity -> intersection points (using MERGED points for accurate splitting)
    // For each merged point, determine which entities it lies on
    std::unordered_map<EntityID, std::vector<Vec2d>> entityIntersections;
    double mergeTolSq = minPointSpacing_ * minPointSpacing_;

    for (const Vec2d& mergedPt : mergedPoints) {
        // Track which entities this merged point should be applied to
        std::unordered_set<EntityID> entitiesForThisPoint;

        // All merged points lie on the new entity (by construction)
        if (newEntity->type() == EntityType::Line || newEntity->type() == EntityType::Arc) {
            entitiesForThisPoint.insert(newEntityId);
        }

        // Find which existing entities contributed to this merged point
        for (size_t i = 0; i < allIntersections.size(); ++i) {
            double dx = allIntersections[i].x - mergedPt.x;
            double dy = allIntersections[i].y - mergedPt.y;
            if (dx*dx + dy*dy <= mergeTolSq) {
                // This raw intersection was merged into mergedPt
                entitiesForThisPoint.insert(intersectedEntityIds[i]);
            }
        }

        // Add merged point to all relevant entities
        for (const EntityID& entityId : entitiesForThisPoint) {
            entityIntersections[entityId].push_back(mergedPt);
        }
    }

    // Split each intersected entity at its intersection points
    // NOTE: We need to be careful about order - split from end to start
    // to avoid invalidating entity IDs during splitting

    for (auto& [entityId, points] : entityIntersections) {
        auto* entity = sketch.getEntity(entityId);
        if (!entity) {
            continue;
        }

        if (entity->type() == EntityType::Line) {
            auto* line = sketch.getEntityAs<SketchLine>(entityId);
            if (!line) {
                continue;
            }

            auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
            auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
            if (!startPt || !endPt) {
                continue;
            }

            // Sort intersection points along line (by parameter t)
            gp_Pnt2d p1 = startPt->position();
            gp_Pnt2d p2 = endPt->position();
            double dx = p2.X() - p1.X();
            double dy = p2.Y() - p1.Y();
            double lenSq = dx * dx + dy * dy;

            if (lenSq < 1e-10) {
                continue;
            }

            struct SplitPoint {
                Vec2d pos;
                double t;
            };
            std::vector<SplitPoint> splits;

            for (const auto& pt : points) {
                double t = ((pt.x - p1.X()) * dx + (pt.y - p1.Y()) * dy) / lenSq;
                // Only split if point is actually on segment interior
                if (t > 0.001 && t < 0.999) {
                    splits.push_back({pt, t});
                }
            }

            if (splits.empty()) {
                continue;
            }

            // Sort by parameter t (ascending)
            std::sort(splits.begin(), splits.end(),
                [](const SplitPoint& a, const SplitPoint& b) {
                    return a.t < b.t;
                });

            // Split multiple times (from end to start to maintain IDs)
            EntityID currentLineId = entityId;
            for (auto it = splits.rbegin(); it != splits.rend(); ++it) {
                auto [seg1, seg2] = sketch.splitLineAt(currentLineId, it->pos);
                if (!seg1.empty() && !seg2.empty()) {
                    result.newSegments.push_back(seg1);
                    result.newSegments.push_back(seg2);
                    result.entitiesSplit++;
                    currentLineId = seg1;  // Continue splitting the first segment
                }
            }
        }
        else if (entity->type() == EntityType::Arc) {
            auto* arc = sketch.getEntityAs<SketchArc>(entityId);
            if (!arc) {
                continue;
            }

            auto* centerPt = sketch.getEntityAs<SketchPoint>(arc->centerPointId());
            if (!centerPt) {
                continue;
            }

            gp_Pnt2d center = centerPt->position();

            // Calculate angles for each intersection point
            struct SplitAngle {
                double angle;
            };
            std::vector<SplitAngle> splits;

            for (const auto& pt : points) {
                double angle = std::atan2(pt.y - center.Y(), pt.x - center.X());
                if (arc->containsAngle(angle)) {
                    // Check not too close to endpoints
                    double startAngle = arc->startAngle();
                    double endAngle = arc->endAngle();

                    auto angleDiff = [](double a1, double a2) {
                        double diff = std::fmod(std::abs(a1 - a2), 2.0 * std::numbers::pi);
                        return std::min(diff, 2.0 * std::numbers::pi - diff);
                    };

                    if (angleDiff(angle, startAngle) > 0.01 &&
                        angleDiff(angle, endAngle) > 0.01) {
                        splits.push_back({angle});
                    }
                }
            }

            if (splits.empty()) {
                continue;
            }

            // Sort by angle
            std::sort(splits.begin(), splits.end(),
                [](const SplitAngle& a, const SplitAngle& b) {
                    return a.angle < b.angle;
                });

            // Split arc (from end to start)
            EntityID currentArcId = entityId;
            for (auto it = splits.rbegin(); it != splits.rend(); ++it) {
                auto [seg1, seg2] = sketch.splitArcAt(currentArcId, it->angle);
                if (!seg1.empty() && !seg2.empty()) {
                    result.newSegments.push_back(seg1);
                    result.newSegments.push_back(seg2);
                    result.entitiesSplit++;
                    currentArcId = seg1;
                }
            }
        }
        // Circles are not split (they're closed curves)
    }

    return result;
}

std::vector<Vec2d> IntersectionManager::findIntersections(
    const SketchEntity* e1,
    const SketchEntity* e2,
    const Sketch& sketch,
    const SnapManager& snapManager) const
{
    // Delegate to SnapManager's intersection finding logic
    return snapManager.findEntityIntersections(e1, e2, sketch);
}

EntityID IntersectionManager::findExistingPointAt(
    const Vec2d& pos,
    const Sketch& sketch,
    double tolerance) const
{
    double tolSq = tolerance * tolerance;

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->type() != EntityType::Point) {
            continue;
        }

        auto* pt = static_cast<const SketchPoint*>(entity.get());
        gp_Pnt2d ptPos = pt->position();

        double dx = pos.x - ptPos.X();
        double dy = pos.y - ptPos.Y();
        double distSq = dx * dx + dy * dy;

        if (distSq <= tolSq) {
            return pt->id();
        }
    }

    return {};
}

std::vector<Vec2d> IntersectionManager::mergeNearbyPoints(
    const std::vector<Vec2d>& points,
    double tolerance) const
{
    if (points.empty()) {
        return {};
    }

    std::vector<Vec2d> merged;
    std::vector<bool> used(points.size(), false);
    double tolSq = tolerance * tolerance;

    for (size_t i = 0; i < points.size(); ++i) {
        if (used[i]) {
            continue;
        }

        // Start a cluster with this point
        Vec2d cluster = points[i];
        size_t clusterSize = 1;
        used[i] = true;

        // Find all nearby points and average them
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (used[j]) {
                continue;
            }

            double dx = points[j].x - points[i].x;
            double dy = points[j].y - points[i].y;
            double distSq = dx * dx + dy * dy;

            if (distSq <= tolSq) {
                cluster.x += points[j].x;
                cluster.y += points[j].y;
                clusterSize++;
                used[j] = true;
            }
        }

        // Average the cluster
        cluster.x /= clusterSize;
        cluster.y /= clusterSize;
        merged.push_back(cluster);
    }

    return merged;
}

} // namespace onecad::core::sketch
