/**
 * @file AutoConstrainer.cpp
 * @brief Implementation of automatic constraint inference
 */

#include "AutoConstrainer.h"
#include "Sketch.h"
#include "SketchPoint.h"
#include "SketchLine.h"
#include "SketchArc.h"
#include "SketchCircle.h"
#include <QLoggingCategory>
#include <QString>
#include <cmath>
#include <algorithm>
#include <limits>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logAutoConstrainer, "onecad.core.autoconstraint")

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double HALF_PI = PI / 2.0;

// Helper to convert gp_Pnt2d to Vec2d
inline Vec2d toVec2d(const gp_Pnt2d& p) {
    return {p.X(), p.Y()};
}

// Normalize angle to [-π, π]
inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}
} // anonymous namespace

AutoConstrainer::AutoConstrainer() {
    // Enable all constraint types by default
    for (int i = static_cast<int>(ConstraintType::Coincident);
         i <= static_cast<int>(ConstraintType::Symmetric); ++i) {
        typeEnabled_[static_cast<ConstraintType>(i)] = true;
    }
}

AutoConstrainer::AutoConstrainer(const AutoConstrainerConfig& config)
    : config_(config) {
    // Enable all constraint types by default
    for (int i = static_cast<int>(ConstraintType::Coincident);
         i <= static_cast<int>(ConstraintType::Symmetric); ++i) {
        typeEnabled_[static_cast<ConstraintType>(i)] = true;
    }
}

void AutoConstrainer::setTypeEnabled(ConstraintType type, bool enabled) {
    typeEnabled_[type] = enabled;
}

bool AutoConstrainer::isTypeEnabled(ConstraintType type) const {
    auto it = typeEnabled_.find(type);
    return it != typeEnabled_.end() ? it->second : true;
}

void AutoConstrainer::setAllTypesEnabled(bool enabled) {
    for (int i = static_cast<int>(ConstraintType::Coincident);
         i <= static_cast<int>(ConstraintType::Symmetric); ++i) {
        typeEnabled_[static_cast<ConstraintType>(i)] = enabled;
    }
}

std::vector<InferredConstraint> AutoConstrainer::inferConstraints(
    const Vec2d& point,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    if (!config_.enabled) {
        qCDebug(logAutoConstrainer) << "inferConstraints:disabled";
        return {};
    }

    qCDebug(logAutoConstrainer) << "inferConstraints:start"
                                << "point=" << point.x << point.y
                                << "activeEntity=" << QString::fromStdString(context.activeEntity);

    std::vector<InferredConstraint> results;

    // Infer coincident with existing points
    if (isTypeEnabled(ConstraintType::Coincident)) {
        if (auto c = inferCoincident(point, sketch, context.activeEntity)) {
            results.push_back(*c);
        }
    }

    qCDebug(logAutoConstrainer) << "inferConstraints:done" << "count=" << results.size();
    return results;
}

std::vector<InferredConstraint> AutoConstrainer::inferLineConstraints(
    const Vec2d& startPoint,
    const Vec2d& endPoint,
    EntityID lineId,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    if (!config_.enabled) {
        qCDebug(logAutoConstrainer) << "inferLineConstraints:disabled";
        return {};
    }

    std::vector<InferredConstraint> results;
    double length = distance(startPoint, endPoint);
    qCDebug(logAutoConstrainer) << "inferLineConstraints:start"
                                << "lineId=" << QString::fromStdString(lineId)
                                << "start=" << startPoint.x << startPoint.y
                                << "end=" << endPoint.x << endPoint.y
                                << "length=" << length
                                << "polylineMode=" << context.isPolylineMode
                                << "previousEntity="
                                << (context.previousEntity
                                        ? QString::fromStdString(*context.previousEntity)
                                        : QStringLiteral("<none>"));
    if (length < constants::MIN_GEOMETRY_SIZE) {
        qCDebug(logAutoConstrainer) << "inferLineConstraints:skip-short" << length;
        return results;
    }

    // Infer horizontal
    bool hasOrientationConstraint = false;
    if (isTypeEnabled(ConstraintType::Horizontal)) {
        if (auto c = inferHorizontal(startPoint, endPoint, lineId)) {
            results.push_back(*c);
            hasOrientationConstraint = true;
        }
    }

    // Infer vertical
    if (isTypeEnabled(ConstraintType::Vertical)) {
        if (auto c = inferVertical(startPoint, endPoint, lineId)) {
            results.push_back(*c);
            hasOrientationConstraint = true;
        }
    }

    // Infer perpendicular to existing lines
    if (!hasOrientationConstraint && isTypeEnabled(ConstraintType::Perpendicular)) {
        if (auto c = inferPerpendicular(startPoint, endPoint, lineId, sketch, context)) {
            results.push_back(*c);
        }
    }

    // Infer parallel to existing lines
    if (!hasOrientationConstraint && isTypeEnabled(ConstraintType::Parallel)) {
        if (auto c = inferParallel(startPoint, endPoint, lineId, sketch, context)) {
            results.push_back(*c);
        }
    }

    // Infer coincident for endpoint
    if (isTypeEnabled(ConstraintType::Coincident)) {
        if (auto c = inferCoincident(endPoint, sketch, lineId)) {
            results.push_back(*c);
        }
    }

    qCDebug(logAutoConstrainer) << "inferLineConstraints:done" << "count=" << results.size();
    return results;
}

std::vector<InferredConstraint> AutoConstrainer::inferCircleConstraints(
    const Vec2d& center,
    double radius,
    EntityID circleId,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    (void)context;
    if (!config_.enabled) {
        qCDebug(logAutoConstrainer) << "inferCircleConstraints:disabled";
        return {};
    }

    qCDebug(logAutoConstrainer) << "inferCircleConstraints:start"
                                << "circleId=" << QString::fromStdString(circleId)
                                << "center=" << center.x << center.y
                                << "radius=" << radius;

    std::vector<InferredConstraint> results;

    // Infer coincident for center
    if (isTypeEnabled(ConstraintType::Coincident)) {
        if (auto c = inferCoincident(center, sketch, circleId)) {
            results.push_back(*c);
        }
    }

    // Infer concentric with existing circles/arcs
    if (isTypeEnabled(ConstraintType::Concentric)) {
        if (auto c = inferConcentric(center, circleId, sketch)) {
            results.push_back(*c);
        }
    }

    // Infer equal radius with existing circles/arcs
    if (isTypeEnabled(ConstraintType::Equal)) {
        if (auto c = inferEqualRadius(radius, circleId, sketch)) {
            results.push_back(*c);
        }
    }

    qCDebug(logAutoConstrainer) << "inferCircleConstraints:done" << "count=" << results.size();
    return results;
}

std::vector<InferredConstraint> AutoConstrainer::inferArcConstraints(
    const Vec2d& center,
    double radius,
    double startAngle,
    double endAngle,
    EntityID arcId,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    if (!config_.enabled) {
        qCDebug(logAutoConstrainer) << "inferArcConstraints:disabled";
        return {};
    }

    qCDebug(logAutoConstrainer) << "inferArcConstraints:start"
                                << "arcId=" << QString::fromStdString(arcId)
                                << "center=" << center.x << center.y
                                << "radius=" << radius
                                << "startAngle=" << startAngle
                                << "endAngle=" << endAngle;

    std::vector<InferredConstraint> results;

    // Calculate arc start point
    Vec2d arcStartPoint = {
        center.x + radius * std::cos(startAngle),
        center.y + radius * std::sin(startAngle)
    };

    // Infer tangent to existing lines at arc start
    if (isTypeEnabled(ConstraintType::Tangent)) {
        if (auto c = inferTangent(center, arcStartPoint, arcId, sketch, context)) {
            results.push_back(*c);
        }
    }

    // Infer concentric with existing circles/arcs
    if (isTypeEnabled(ConstraintType::Concentric)) {
        if (auto c = inferConcentric(center, arcId, sketch)) {
            results.push_back(*c);
        }
    }

    // Infer equal radius with existing circles/arcs
    if (isTypeEnabled(ConstraintType::Equal)) {
        if (auto c = inferEqualRadius(radius, arcId, sketch)) {
            results.push_back(*c);
        }
    }

    qCDebug(logAutoConstrainer) << "inferArcConstraints:done" << "count=" << results.size();
    return results;
}

std::vector<InferredConstraint> AutoConstrainer::filterForAutoApply(
    const std::vector<InferredConstraint>& constraints) const
{
    std::vector<InferredConstraint> result;
    for (const auto& c : constraints) {
        if (c.confidence >= config_.autoApplyThreshold) {
            result.push_back(c);
        }
    }
    return result;
}

// ========== Individual Inference Methods ==========

std::optional<InferredConstraint> AutoConstrainer::inferHorizontal(
    const Vec2d& startPoint,
    const Vec2d& endPoint,
    EntityID lineId) const
{
    double angle = lineAngle(startPoint, endPoint);

    // Check if angle is near 0 or π (horizontal)
    double deviation = std::min(std::abs(angle), std::abs(std::abs(angle) - PI));

    if (deviation <= config_.horizontalTolerance) {
        // Calculate confidence: higher when closer to exact horizontal
        double confidence = 1.0 - (deviation / config_.horizontalTolerance);

        return InferredConstraint{
            .type = ConstraintType::Horizontal,
            .entity1 = lineId,
            .confidence = confidence
        };
    }

    return std::nullopt;
}

std::optional<InferredConstraint> AutoConstrainer::inferVertical(
    const Vec2d& startPoint,
    const Vec2d& endPoint,
    EntityID lineId) const
{
    double angle = lineAngle(startPoint, endPoint);

    // Check if angle is near ±π/2 (vertical)
    double deviation = std::abs(std::abs(angle) - HALF_PI);

    if (deviation <= config_.verticalTolerance) {
        // Calculate confidence: higher when closer to exact vertical
        double confidence = 1.0 - (deviation / config_.verticalTolerance);

        return InferredConstraint{
            .type = ConstraintType::Vertical,
            .entity1 = lineId,
            .confidence = confidence
        };
    }

    return std::nullopt;
}

std::optional<InferredConstraint> AutoConstrainer::inferCoincident(
    const Vec2d& point,
    const Sketch& sketch,
    EntityID excludeEntity) const
{
    const SketchPoint* nearestPoint = nullptr;
    double nearestDistSq = config_.coincidenceTolerance * config_.coincidenceTolerance;
    EntityID nearestId;

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->id() == excludeEntity) continue;
        if (entity->type() != EntityType::Point) continue;

        const auto* pt = static_cast<const SketchPoint*>(entity.get());
        if (!excludeEntity.empty()) {
            const auto& connected = pt->connectedEntities();
            if (std::find(connected.begin(), connected.end(), excludeEntity) != connected.end()) {
                continue;
            }
        }
        Vec2d ptPos = toVec2d(pt->position());
        double distSq = (point.x - ptPos.x) * (point.x - ptPos.x) +
                        (point.y - ptPos.y) * (point.y - ptPos.y);

        if (distSq < nearestDistSq) {
            nearestDistSq = distSq;
            nearestPoint = pt;
            nearestId = entity->id();
        }
    }

    if (nearestPoint) {
        double dist = std::sqrt(nearestDistSq);
        double confidence = 1.0 - (dist / config_.coincidenceTolerance);

        return InferredConstraint{
            .type = ConstraintType::Coincident,
            .entity1 = nearestId,
            .confidence = confidence,
            .position = toVec2d(nearestPoint->position())
        };
    }

    return std::nullopt;
}

std::optional<InferredConstraint> AutoConstrainer::inferPerpendicular(
    const Vec2d& line1Start,
    const Vec2d& line1End,
    EntityID line1Id,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    auto considerLine = [&](const SketchLine* line) -> std::optional<InferredConstraint> {
        if (!line) {
            return std::nullopt;
        }
        if (!line1Id.empty() && line->id() == line1Id) {
            return std::nullopt;
        }

        const auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
        const auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
        if (!startPt || !endPt) {
            return std::nullopt;
        }

        Vec2d line2Start = toVec2d(startPt->position());
        Vec2d line2End = toVec2d(endPt->position());
        if (distance(line2Start, line2End) < constants::MIN_GEOMETRY_SIZE) {
            return std::nullopt;
        }

        double angle = angleBetweenLines(line1Start, line1End, line2Start, line2End);
        double deviation = std::abs(angle - HALF_PI);
        if (deviation > config_.perpendicularTolerance) {
            return std::nullopt;
        }

        double confidence = 1.0 - (deviation / config_.perpendicularTolerance);
        return InferredConstraint{
            .type = ConstraintType::Perpendicular,
            .entity1 = line1Id,
            .entity2 = line->id(),
            .confidence = confidence
        };
    };

    if (context.previousEntity && context.isPolylineMode) {
        if (const auto* prev = sketch.getEntityAs<SketchLine>(*context.previousEntity)) {
            if (auto inferred = considerLine(prev)) {
                return inferred;
            }
        }
    }

    std::optional<InferredConstraint> best;
    double bestDeviation = std::numeric_limits<double>::infinity();

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->type() != EntityType::Line) continue;
        if (!line1Id.empty() && entity->id() == line1Id) continue;

        const auto* line = static_cast<const SketchLine*>(entity.get());
        const auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
        const auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
        if (!startPt || !endPt) continue;

        Vec2d line2Start = toVec2d(startPt->position());
        Vec2d line2End = toVec2d(endPt->position());
        if (distance(line2Start, line2End) < constants::MIN_GEOMETRY_SIZE) {
            continue;
        }

        double angle = angleBetweenLines(line1Start, line1End, line2Start, line2End);
        double deviation = std::abs(angle - HALF_PI);
        if (deviation > config_.perpendicularTolerance) {
            continue;
        }

        if (deviation < bestDeviation) {
            bestDeviation = deviation;
            double confidence = 1.0 - (deviation / config_.perpendicularTolerance);
            best = InferredConstraint{
                .type = ConstraintType::Perpendicular,
                .entity1 = line1Id,
                .entity2 = entity->id(),
                .confidence = confidence
            };
        }
    }

    return best;
}

std::optional<InferredConstraint> AutoConstrainer::inferParallel(
    const Vec2d& lineStart,
    const Vec2d& lineEnd,
    EntityID lineId,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    auto considerLine = [&](const SketchLine* line) -> std::optional<InferredConstraint> {
        if (!line) {
            return std::nullopt;
        }
        if (!lineId.empty() && line->id() == lineId) {
            return std::nullopt;
        }

        const auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
        const auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
        if (!startPt || !endPt) {
            return std::nullopt;
        }

        Vec2d line2Start = toVec2d(startPt->position());
        Vec2d line2End = toVec2d(endPt->position());
        if (distance(line2Start, line2End) < constants::MIN_GEOMETRY_SIZE) {
            return std::nullopt;
        }

        double angle = angleBetweenLines(lineStart, lineEnd, line2Start, line2End);
        double deviation = std::min(angle, std::abs(angle - PI));
        if (deviation > config_.parallelTolerance) {
            return std::nullopt;
        }

        double confidence = 1.0 - (deviation / config_.parallelTolerance);
        return InferredConstraint{
            .type = ConstraintType::Parallel,
            .entity1 = lineId,
            .entity2 = line->id(),
            .confidence = confidence
        };
    };

    if (context.previousEntity && context.isPolylineMode) {
        if (const auto* prev = sketch.getEntityAs<SketchLine>(*context.previousEntity)) {
            if (auto inferred = considerLine(prev)) {
                return inferred;
            }
        }
    }

    std::optional<InferredConstraint> best;
    double bestDeviation = std::numeric_limits<double>::infinity();

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->type() != EntityType::Line) continue;
        if (!lineId.empty() && entity->id() == lineId) continue;

        const auto* line = static_cast<const SketchLine*>(entity.get());
        const auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
        const auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
        if (!startPt || !endPt) continue;

        Vec2d line2Start = toVec2d(startPt->position());
        Vec2d line2End = toVec2d(endPt->position());
        if (distance(line2Start, line2End) < constants::MIN_GEOMETRY_SIZE) {
            continue;
        }

        double angle = angleBetweenLines(lineStart, lineEnd, line2Start, line2End);
        double deviation = std::min(angle, std::abs(angle - PI));
        if (deviation > config_.parallelTolerance) {
            continue;
        }

        if (deviation < bestDeviation) {
            bestDeviation = deviation;
            double confidence = 1.0 - (deviation / config_.parallelTolerance);
            best = InferredConstraint{
                .type = ConstraintType::Parallel,
                .entity1 = lineId,
                .entity2 = entity->id(),
                .confidence = confidence
            };
        }
    }

    return best;
}

std::optional<InferredConstraint> AutoConstrainer::inferTangent(
    const Vec2d& arcCenter,
    const Vec2d& arcStartPoint,
    EntityID arcId,
    const Sketch& sketch,
    const DrawingContext& context) const
{
    (void)context;
    // Check if arc starts from a line endpoint and is tangent to it
    double bestDeviation = std::numeric_limits<double>::infinity();
    std::optional<InferredConstraint> best;

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->id() == arcId) continue;
        if (entity->type() != EntityType::Line) continue;

        const auto* line = static_cast<const SketchLine*>(entity.get());
        const auto* startPt = sketch.getEntityAs<SketchPoint>(line->startPointId());
        const auto* endPt = sketch.getEntityAs<SketchPoint>(line->endPointId());
        if (!startPt || !endPt) continue;

        Vec2d lineStart = toVec2d(startPt->position());
        Vec2d lineEnd = toVec2d(endPt->position());

        // Check if arc starts from line endpoint
        double distToStart = distance(arcStartPoint, lineStart);
        double distToEnd = distance(arcStartPoint, lineEnd);

        Vec2d lineEndpoint;
        bool startsFromEndpoint = false;

        if (distToStart < config_.coincidenceTolerance) {
            lineEndpoint = lineStart;
            startsFromEndpoint = true;
        } else if (distToEnd < config_.coincidenceTolerance) {
            lineEndpoint = lineEnd;
            startsFromEndpoint = true;
        }

        if (!startsFromEndpoint) continue;

        // Calculate line direction at the endpoint
        Vec2d lineDir;
        if (distToStart < distToEnd) {
            lineDir = {lineStart.x - lineEnd.x, lineStart.y - lineEnd.y};
        } else {
            lineDir = {lineEnd.x - lineStart.x, lineEnd.y - lineStart.y};
        }

        // Normalize line direction
        double lineLen = std::sqrt(lineDir.x * lineDir.x + lineDir.y * lineDir.y);
        if (lineLen < 1e-12) continue;
        lineDir.x /= lineLen;
        lineDir.y /= lineLen;

        // Calculate arc tangent direction at start point
        // For CCW arc, tangent is perpendicular to radius pointing left
        Vec2d radialDir = {
            arcStartPoint.x - arcCenter.x,
            arcStartPoint.y - arcCenter.y
        };
        double radialLen = std::sqrt(radialDir.x * radialDir.x + radialDir.y * radialDir.y);
        if (radialLen < 1e-12) continue;
        radialDir.x /= radialLen;
        radialDir.y /= radialLen;

        // Tangent is perpendicular to radial (CCW rotation)
        Vec2d tangentDir = {-radialDir.y, radialDir.x};

        // Check if tangent direction aligns with line direction
        double dot = std::abs(lineDir.x * tangentDir.x + lineDir.y * tangentDir.y);

        if (dot > std::cos(config_.tangentTolerance)) {
            double deviation = std::acos(std::min(1.0, dot));
            if (deviation < bestDeviation) {
                bestDeviation = deviation;
                double confidence = 1.0 - (deviation / config_.tangentTolerance);
                best = InferredConstraint{
                    .type = ConstraintType::Tangent,
                    .entity1 = arcId,
                    .entity2 = entity->id(),
                    .confidence = confidence
                };
            }
        }
    }

    return best;
}

std::optional<InferredConstraint> AutoConstrainer::inferConcentric(
    const Vec2d& center,
    EntityID entityId,
    const Sketch& sketch) const
{
    std::optional<InferredConstraint> best;
    double bestDist = config_.coincidenceTolerance;

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->id() == entityId) continue;

        Vec2d existingCenter;
        bool hasCenter = false;

        if (entity->type() == EntityType::Circle) {
            const auto* circle = static_cast<const SketchCircle*>(entity.get());
            const auto* centerPt = sketch.getEntityAs<SketchPoint>(circle->centerPointId());
            if (centerPt) {
                existingCenter = toVec2d(centerPt->position());
                hasCenter = true;
            }
        } else if (entity->type() == EntityType::Arc) {
            const auto* arc = static_cast<const SketchArc*>(entity.get());
            const auto* centerPt = sketch.getEntityAs<SketchPoint>(arc->centerPointId());
            if (centerPt) {
                existingCenter = toVec2d(centerPt->position());
                hasCenter = true;
            }
        }

        if (!hasCenter) continue;

        double dist = distance(center, existingCenter);
        if (dist < bestDist) {
            bestDist = dist;
            double confidence = 1.0 - (dist / config_.coincidenceTolerance);

            best = InferredConstraint{
                .type = ConstraintType::Concentric,
                .entity1 = entityId,
                .entity2 = entity->id(),
                .confidence = confidence
            };
        }
    }

    return best;
}

std::optional<InferredConstraint> AutoConstrainer::inferEqualRadius(
    double radius,
    EntityID entityId,
    const Sketch& sketch) const
{
    constexpr double RADIUS_TOLERANCE = 0.5;  // mm

    std::optional<InferredConstraint> best;
    double bestDiff = RADIUS_TOLERANCE;

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity->id() == entityId) continue;

        double existingRadius = 0;
        bool hasRadius = false;

        if (entity->type() == EntityType::Circle) {
            existingRadius = static_cast<const SketchCircle*>(entity.get())->radius();
            hasRadius = true;
        } else if (entity->type() == EntityType::Arc) {
            existingRadius = static_cast<const SketchArc*>(entity.get())->radius();
            hasRadius = true;
        }

        if (!hasRadius) continue;

        double diff = std::abs(radius - existingRadius);
        if (diff < bestDiff) {
            bestDiff = diff;
            double confidence = 1.0 - (diff / RADIUS_TOLERANCE);

            best = InferredConstraint{
                .type = ConstraintType::Equal,
                .entity1 = entityId,
                .entity2 = entity->id(),
                .confidence = confidence
            };
        }
    }

    return best;
}

// ========== Geometry Helpers ==========

double AutoConstrainer::lineAngle(const Vec2d& start, const Vec2d& end) {
    return std::atan2(end.y - start.y, end.x - start.x);
}

double AutoConstrainer::angleBetweenLines(
    const Vec2d& line1Start, const Vec2d& line1End,
    const Vec2d& line2Start, const Vec2d& line2End)
{
    double angle1 = lineAngle(line1Start, line1End);
    double angle2 = lineAngle(line2Start, line2End);

    double diff = std::abs(normalizeAngle(angle1 - angle2));
    // Return angle in [0, π]
    return std::min(diff, PI - diff);
}

bool AutoConstrainer::areLinesPerpendicular(
    const Vec2d& line1Start, const Vec2d& line1End,
    const Vec2d& line2Start, const Vec2d& line2End) const
{
    double angle = angleBetweenLines(line1Start, line1End, line2Start, line2End);
    return std::abs(angle - HALF_PI) <= config_.perpendicularTolerance;
}

bool AutoConstrainer::areLinesParallel(
    const Vec2d& line1Start, const Vec2d& line1End,
    const Vec2d& line2Start, const Vec2d& line2End) const
{
    double angle = angleBetweenLines(line1Start, line1End, line2Start, line2End);
    // Parallel means angle is 0 or π
    return angle <= config_.parallelTolerance ||
           std::abs(angle - PI) <= config_.parallelTolerance;
}

double AutoConstrainer::distance(const Vec2d& a, const Vec2d& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace onecad::core::sketch
