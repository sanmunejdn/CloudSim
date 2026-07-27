#include "Sketch.h"
#include "constraints/Constraints.h"
#include "solver/ConstraintSolver.h"
#include "solver/SolverAdapter.h"
#include "../loop/RegionUtils.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <limits>
#include <numbers>
#include <utility>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSketchEngine, "onecad.core.sketch")

namespace {

ConstraintSupportResult unsupportedConstraint(std::string reason) {
    return ConstraintSupportResult{.supported = false, .reason = std::move(reason)};
}

bool hasType(const Sketch& sketch, const EntityID& id, EntityType type) {
    const SketchEntity* entity = sketch.getEntity(id);
    return entity && entity->type() == type;
}

bool hasCurveType(const Sketch& sketch, const EntityID& id) {
    const SketchEntity* entity = sketch.getEntity(id);
    if (!entity) {
        return false;
    }
    const EntityType type = entity->type();
    return type == EntityType::Line || type == EntityType::Arc || type == EntityType::Circle;
}

bool isDistancePairSupported(EntityType first, EntityType second) {
    const bool firstPointOrLine = first == EntityType::Point || first == EntityType::Line;
    const bool secondPointOrLine = second == EntityType::Point || second == EntityType::Line;
    return firstPointOrLine && secondPointOrLine;
}

bool isEqualPairSupported(EntityType first, EntityType second) {
    const bool firstLine = first == EntityType::Line;
    const bool secondLine = second == EntityType::Line;
    const bool firstCircular = first == EntityType::Arc || first == EntityType::Circle;
    const bool secondCircular = second == EntityType::Arc || second == EntityType::Circle;
    return (firstLine && secondLine) || (firstCircular && secondCircular);
}

bool isTangentPairSupported(EntityType first, EntityType second) {
    const bool firstLine = first == EntityType::Line;
    const bool secondLine = second == EntityType::Line;
    const bool firstCircular = first == EntityType::Arc || first == EntityType::Circle;
    const bool secondCircular = second == EntityType::Arc || second == EntityType::Circle;
    return (firstLine && secondCircular) || (secondLine && firstCircular) ||
           (firstCircular && secondCircular);
}

bool isCircularCurve(EntityType type) {
    return type == EntityType::Arc || type == EntityType::Circle;
}

std::optional<std::string> firstUnsupportedConstraint(const Sketch& sketch) {
    for (const auto& constraint : sketch.getAllConstraints()) {
        if (!constraint) {
            continue;
        }
        const ConstraintSupportResult support = sketch.validateConstraintSupport(*constraint);
        if (!support.supported) {
            return "Unsupported constraint " + constraint->id() + ": " + support.reason;
        }
    }
    return std::nullopt;
}

} // namespace

Sketch::Sketch(const SketchPlane& plane)
    : plane_(plane) {
}

Sketch::~Sketch() = default;

Sketch::Sketch(Sketch&& other) noexcept = default;

Sketch& Sketch::operator=(Sketch&& other) noexcept = default;

EntityID Sketch::addPoint(double x, double y, bool construction) {
    qCDebug(logSketchEngine) << "addPoint" << x << y << "construction=" << construction;
    auto point = std::make_unique<SketchPoint>(x, y);
    point->setConstruction(construction);

    EntityID id = point->id();
    entityIndex_[id] = entities_.size();
    entities_.push_back(std::move(point));

    invalidateSolver();
    qCDebug(logSketchEngine) << "addPoint:done"
                             << "id=" << QString::fromStdString(id)
                             << "totalEntities=" << entities_.size();
    return id;
}

EntityID Sketch::addLine(EntityID startId, EntityID endId, bool construction) {
    qCDebug(logSketchEngine) << "addLine"
                             << "startId=" << QString::fromStdString(startId)
                             << "endId=" << QString::fromStdString(endId)
                             << "construction=" << construction;
    auto* startPoint = getEntityAs<SketchPoint>(startId);
    auto* endPoint = getEntityAs<SketchPoint>(endId);
    if (!startPoint || !endPoint) {
        qCWarning(logSketchEngine) << "addLine:missing-endpoints";
        return {};
    }

    auto line = std::make_unique<SketchLine>(startId, endId);
    line->setConstruction(construction);

    EntityID id = line->id();
    entityIndex_[id] = entities_.size();
    entities_.push_back(std::move(line));

    startPoint->addConnectedEntity(id);
    endPoint->addConnectedEntity(id);

    invalidateSolver();
    return id;
}

EntityID Sketch::addLine(double x1, double y1, double x2, double y2, bool construction) {
    EntityID startId = addPoint(x1, y1, construction);
    EntityID endId = addPoint(x2, y2, construction);
    if (startId.empty() || endId.empty()) {
        return {};
    }
    return addLine(startId, endId, construction);
}

EntityID Sketch::addArc(EntityID centerId, double radius, double startAngle,
                        double endAngle, bool construction) {
    auto* centerPoint = getEntityAs<SketchPoint>(centerId);
    if (!centerPoint) {
        return {};
    }

    auto arc = std::make_unique<SketchArc>(centerId, radius, startAngle, endAngle);
    arc->setConstruction(construction);

    EntityID id = arc->id();
    entityIndex_[id] = entities_.size();
    entities_.push_back(std::move(arc));

    centerPoint->addConnectedEntity(id);

    invalidateSolver();
    return id;
}

EntityID Sketch::addCircle(EntityID centerId, double radius, bool construction) {
    auto* centerPoint = getEntityAs<SketchPoint>(centerId);
    if (!centerPoint) {
        return {};
    }

    auto circle = std::make_unique<SketchCircle>(centerId, radius);
    circle->setConstruction(construction);

    EntityID id = circle->id();
    entityIndex_[id] = entities_.size();
    entities_.push_back(std::move(circle));

    centerPoint->addConnectedEntity(id);

    invalidateSolver();
    return id;
}

EntityID Sketch::addCircle(double centerX, double centerY, double radius, bool construction) {
    EntityID centerId = addPoint(centerX, centerY);
    if (centerId.empty()) {
        return {};
    }
    return addCircle(centerId, radius, construction);
}

EntityID Sketch::addEllipse(EntityID centerId, double majorRadius, double minorRadius,
                            double rotation, bool construction) {
    auto* centerPoint = getEntityAs<SketchPoint>(centerId);
    if (!centerPoint) {
        return {};
    }

    // Normalize ellipse parameters: ensure major >= minor
    if (minorRadius > majorRadius) {
        std::swap(majorRadius, minorRadius);
        rotation += std::numbers::pi / 2.0;
        // Normalize rotation to [-π, π]
        if (rotation > std::numbers::pi) {
            rotation -= 2.0 * std::numbers::pi;
        }
    }

    auto ellipse = std::make_unique<SketchEllipse>(centerId, majorRadius, minorRadius, rotation);
    ellipse->setConstruction(construction);

    EntityID id = ellipse->id();
    entityIndex_[id] = entities_.size();
    entities_.push_back(std::move(ellipse));

    centerPoint->addConnectedEntity(id);

    invalidateSolver();
    return id;
}

bool Sketch::removeEntity(EntityID id) {
    auto it = entityIndex_.find(id);
    if (it == entityIndex_.end()) {
        return false;
    }

    if (it->second >= entities_.size()) {
        rebuildEntityIndex();
        it = entityIndex_.find(id);
        if (it == entityIndex_.end() || it->second >= entities_.size()) {
            return false;
        }
    }

    SketchEntity* entity = entities_[it->second].get();
    if (!entity) {
        return false;
    }
    if (entity->isReferenceLocked()) {
        return false;
    }

    if (auto* point = dynamic_cast<SketchPoint*>(entity)) {
        std::unordered_set<EntityID> dependents;
        for (const auto& candidate : entities_) {
            if (!candidate) {
                continue;
            }
            if (auto* line = dynamic_cast<SketchLine*>(candidate.get())) {
                if (line->startPointId() == id || line->endPointId() == id) {
                    dependents.insert(line->id());
                }
            } else if (auto* arc = dynamic_cast<SketchArc*>(candidate.get())) {
                if (arc->centerPointId() == id) {
                    dependents.insert(arc->id());
                }
            } else if (auto* circle = dynamic_cast<SketchCircle*>(candidate.get())) {
                if (circle->centerPointId() == id) {
                    dependents.insert(circle->id());
                }
            } else if (auto* ellipse = dynamic_cast<SketchEllipse*>(candidate.get())) {
                if (ellipse->centerPointId() == id) {
                    dependents.insert(ellipse->id());
                }
            }
        }

        for (const auto& entityId : point->connectedEntities()) {
            if (!entityId.empty() && entityId != id) {
                dependents.insert(entityId);
            }
        }

        for (const auto& depId : dependents) {
            if (depId == id) {
                continue;
            }
            const SketchEntity* dependent = getEntity(depId);
            if (dependent && dependent->isReferenceLocked()) {
                return false;
            }
        }

        for (const auto& depId : dependents) {
            if (depId == id) {
                continue;
            }
            if (getEntity(depId) && !removeEntity(depId)) {
                return false;
            }
        }
    }

    // Track points that may become orphaned after this entity is removed
    std::vector<EntityID> potentiallyOrphanedPoints;

    if (auto* line = dynamic_cast<SketchLine*>(entity)) {
        if (auto* start = getEntityAs<SketchPoint>(line->startPointId())) {
            start->removeConnectedEntity(line->id());
            potentiallyOrphanedPoints.push_back(line->startPointId());
        }
        if (auto* end = getEntityAs<SketchPoint>(line->endPointId())) {
            end->removeConnectedEntity(line->id());
            potentiallyOrphanedPoints.push_back(line->endPointId());
        }
    } else if (auto* arc = dynamic_cast<SketchArc*>(entity)) {
        if (auto* center = getEntityAs<SketchPoint>(arc->centerPointId())) {
            center->removeConnectedEntity(arc->id());
            potentiallyOrphanedPoints.push_back(arc->centerPointId());
        }
    } else if (auto* circle = dynamic_cast<SketchCircle*>(entity)) {
        if (auto* center = getEntityAs<SketchPoint>(circle->centerPointId())) {
            center->removeConnectedEntity(circle->id());
            potentiallyOrphanedPoints.push_back(circle->centerPointId());
        }
    } else if (auto* ellipse = dynamic_cast<SketchEllipse*>(entity)) {
        if (auto* center = getEntityAs<SketchPoint>(ellipse->centerPointId())) {
            center->removeConnectedEntity(ellipse->id());
            potentiallyOrphanedPoints.push_back(ellipse->centerPointId());
        }
    }

    bool removedConstraints = false;
    for (size_t i = 0; i < constraints_.size();) {
        if (constraints_[i] && constraints_[i]->references(id)) {
            constraints_.erase(constraints_.begin() + static_cast<long>(i));
            removedConstraints = true;
        } else {
            ++i;
        }
    }

    if (removedConstraints) {
        rebuildConstraintIndex();
    }

    entities_.erase(entities_.begin() + static_cast<long>(it->second));
    rebuildEntityIndex();
    invalidateSolver();

    // Clean up orphaned points (points with no connected entities)
    for (const auto& pointId : potentiallyOrphanedPoints) {
        auto* point = getEntityAs<SketchPoint>(pointId);
        if (point && point->connectedEntities().empty()) {
            // Recursively remove orphaned point
            removeEntity(pointId);
        }
    }

    return true;
}

std::pair<EntityID, EntityID> Sketch::splitLineAt(EntityID lineId, const Vec2d& splitPoint) {
    auto* line = getEntityAs<SketchLine>(lineId);
    if (!line) {
        return {{}, {}};
    }
    if (line->isReferenceLocked()) {
        return {{}, {}};
    }

    auto* startPt = getEntityAs<SketchPoint>(line->startPointId());
    auto* endPt = getEntityAs<SketchPoint>(line->endPointId());
    if (!startPt || !endPt) {
        return {{}, {}};
    }

    // Verify split point is on line segment
    gp_Pnt2d p1 = startPt->position();
    gp_Pnt2d p2 = endPt->position();
    double dx = p2.X() - p1.X();
    double dy = p2.Y() - p1.Y();
    double lenSq = dx * dx + dy * dy;

    if (lenSq < 1e-10) {
        return {{}, {}};  // Degenerate line
    }

    // Calculate parameter t for split point
    double t = ((splitPoint.x - p1.X()) * dx + (splitPoint.y - p1.Y()) * dy) / lenSq;

    // Tolerance check: point must be on segment (not at endpoints)
    constexpr double MIN_SEGMENT_PARAM = 0.001;  // Avoid creating tiny segments
    if (t < MIN_SEGMENT_PARAM || t > (1.0 - MIN_SEGMENT_PARAM)) {
        return {{}, {}};  // Too close to endpoint
    }

    // Store original properties
    EntityID origStartId = line->startPointId();
    EntityID origEndId = line->endPointId();
    bool construction = line->isConstruction();

    // Collect constraints that reference this line
    std::vector<std::unique_ptr<SketchConstraint>> constraintsToMigrate;
    for (auto& constraint : constraints_) {
        if (constraint && constraint->references(lineId)) {
            // Clone constraint for migration (we'll recreate for new segments)
            // For now, we'll just remove them - proper migration TBD
        }
    }

    // Remove original line (this also removes constraints)
    if (!removeEntity(lineId)) {
        return {{}, {}};
    }

    // Create intermediate point at split location
    EntityID midPointId = addPoint(splitPoint.x, splitPoint.y, construction);
    if (midPointId.empty()) {
        return {{}, {}};
    }

    // Create two new line segments
    EntityID line1Id = addLine(origStartId, midPointId, construction);
    EntityID line2Id = addLine(midPointId, origEndId, construction);

    if (line1Id.empty() || line2Id.empty()) {
        return {{}, {}};
    }

    return {line1Id, line2Id};
}

std::pair<EntityID, EntityID> Sketch::splitArcAt(EntityID arcId, double splitAngle) {
    auto* arc = getEntityAs<SketchArc>(arcId);
    if (!arc) {
        return {{}, {}};
    }
    if (arc->isReferenceLocked()) {
        return {{}, {}};
    }

    auto* centerPt = getEntityAs<SketchPoint>(arc->centerPointId());
    if (!centerPt) {
        return {{}, {}};
    }

    // Verify split angle is within arc extent
    if (!arc->containsAngle(splitAngle)) {
        return {{}, {}};
    }

    // Check not too close to endpoints
    double startAngle = arc->startAngle();
    double endAngle = arc->endAngle();
    double sweep = arc->sweepAngle();

    // Normalize angle difference to check proximity
    auto angleDiff = [](double a1, double a2) {
        double diff = std::fmod(std::abs(a1 - a2), 2.0 * std::numbers::pi);
        return std::min(diff, 2.0 * std::numbers::pi - diff);
    };

    constexpr double MIN_ANGLE_SEPARATION = 0.01;  // ~0.57 degrees
    if (angleDiff(splitAngle, startAngle) < MIN_ANGLE_SEPARATION ||
        angleDiff(splitAngle, endAngle) < MIN_ANGLE_SEPARATION) {
        return {{}, {}};  // Too close to endpoint
    }

    // Store original properties
    EntityID centerId = arc->centerPointId();
    double radius = arc->radius();
    bool construction = arc->isConstruction();

    // Remove original arc
    if (!removeEntity(arcId)) {
        return {{}, {}};
    }

    // Calculate split point position
    gp_Pnt2d center = centerPt->position();
    double splitX = center.X() + radius * std::cos(splitAngle);
    double splitY = center.Y() + radius * std::sin(splitAngle);

    // Create point at split location (shared between two arcs)
    EntityID splitPointId = addPoint(splitX, splitY, construction);
    if (splitPointId.empty()) {
        return {{}, {}};
    }

    // Create two arc segments
    EntityID arc1Id = addArc(centerId, radius, startAngle, splitAngle, construction);
    EntityID arc2Id = addArc(centerId, radius, splitAngle, endAngle, construction);

    if (arc1Id.empty() || arc2Id.empty()) {
        return {{}, {}};
    }

    return {arc1Id, arc2Id};
}

SketchEntity* Sketch::getEntity(EntityID id) {
    auto it = entityIndex_.find(id);
    if (it == entityIndex_.end()) {
        return nullptr;
    }
    if (it->second >= entities_.size()) {
        rebuildEntityIndex();
        it = entityIndex_.find(id);
        if (it == entityIndex_.end() || it->second >= entities_.size()) {
            return nullptr;
        }
    }
    return entities_[it->second].get();
}

const SketchEntity* Sketch::getEntity(EntityID id) const {
    auto it = entityIndex_.find(id);
    if (it == entityIndex_.end()) {
        return nullptr;
    }
    if (it->second >= entities_.size()) {
        return nullptr;
    }
    return entities_[it->second].get();
}

bool Sketch::isEntityReferenceLocked(EntityID id) const {
    const SketchEntity* entity = getEntity(id);
    return entity && entity->isReferenceLocked();
}

bool Sketch::setEntityReferenceLocked(EntityID id, bool locked) {
    SketchEntity* entity = getEntity(id);
    if (!entity) {
        return false;
    }
    entity->setReferenceLocked(locked);
    return true;
}

std::vector<SketchEntity*> Sketch::getEntitiesByType(EntityType type) {
    std::vector<SketchEntity*> results;
    for (auto& entity : entities_) {
        if (entity->type() == type) {
            results.push_back(entity.get());
        }
    }
    return results;
}

ConstraintSupportResult Sketch::validateConstraintSupport(const SketchConstraint& constraint) const {
    using namespace onecad::core::sketch::constraints;

    auto entityType = [this](const EntityID& id) -> std::optional<EntityType> {
        const SketchEntity* entity = getEntity(id);
        if (!entity) {
            return std::nullopt;
        }
        return entity->type();
    };

    switch (constraint.type()) {
        case ConstraintType::Coincident: {
            const auto* c = dynamic_cast<const CoincidentConstraint*>(&constraint);
            if (!c || !hasType(*this, c->point1(), EntityType::Point) ||
                !hasType(*this, c->point2(), EntityType::Point)) {
                return unsupportedConstraint("Coincident requires two points");
            }
            return {};
        }

        case ConstraintType::Horizontal: {
            const auto* c = dynamic_cast<const HorizontalConstraint*>(&constraint);
            if (!c || !hasType(*this, c->lineId(), EntityType::Line)) {
                return unsupportedConstraint("Horizontal requires a line");
            }
            return {};
        }

        case ConstraintType::Vertical: {
            const auto* c = dynamic_cast<const VerticalConstraint*>(&constraint);
            if (!c || !hasType(*this, c->lineId(), EntityType::Line)) {
                return unsupportedConstraint("Vertical requires a line");
            }
            return {};
        }

        case ConstraintType::Fixed: {
            const auto* c = dynamic_cast<const FixedConstraint*>(&constraint);
            if (!c || !hasType(*this, c->pointId(), EntityType::Point)) {
                return unsupportedConstraint("Fixed requires a point");
            }
            return {};
        }

        case ConstraintType::Midpoint: {
            const auto* c = dynamic_cast<const MidpointConstraint*>(&constraint);
            if (!c || !hasType(*this, c->pointId(), EntityType::Point) ||
                !hasType(*this, c->lineId(), EntityType::Line)) {
                return unsupportedConstraint("Midpoint requires one point and one line");
            }
            return {};
        }

        case ConstraintType::OnCurve: {
            const auto* c = dynamic_cast<const PointOnCurveConstraint*>(&constraint);
            if (!c || !hasType(*this, c->pointId(), EntityType::Point)) {
                return unsupportedConstraint("Point On Curve requires one point");
            }
            const auto curveType = entityType(c->curveId());
            if (!curveType.has_value() || !hasCurveType(*this, c->curveId())) {
                return unsupportedConstraint("Point On Curve supports lines, arcs, and circles only");
            }
            if (c->position() != CurvePosition::Arbitrary && *curveType != EntityType::Line) {
                return unsupportedConstraint("Point On Curve endpoint positions support lines only");
            }
            return {};
        }

        case ConstraintType::Parallel: {
            const auto* c = dynamic_cast<const ParallelConstraint*>(&constraint);
            if (!c || !hasType(*this, c->line1(), EntityType::Line) ||
                !hasType(*this, c->line2(), EntityType::Line)) {
                return unsupportedConstraint("Parallel requires two lines");
            }
            return {};
        }

        case ConstraintType::Perpendicular: {
            const auto* c = dynamic_cast<const PerpendicularConstraint*>(&constraint);
            if (!c || !hasType(*this, c->line1(), EntityType::Line) ||
                !hasType(*this, c->line2(), EntityType::Line)) {
                return unsupportedConstraint("Perpendicular requires two lines");
            }
            return {};
        }

        case ConstraintType::Tangent: {
            const auto* c = dynamic_cast<const TangentConstraint*>(&constraint);
            if (!c) {
                return unsupportedConstraint("Tangent constraint implementation missing");
            }
            const auto first = entityType(c->entity1());
            const auto second = entityType(c->entity2());
            if (!first.has_value() || !second.has_value() ||
                !isTangentPairSupported(*first, *second)) {
                return unsupportedConstraint("Tangent supports lines, arcs, and circles only");
            }
            return {};
        }

        case ConstraintType::Equal: {
            const auto* c = dynamic_cast<const EqualConstraint*>(&constraint);
            if (!c) {
                return unsupportedConstraint("Equal constraint implementation missing");
            }
            const auto first = entityType(c->entity1());
            const auto second = entityType(c->entity2());
            if (!first.has_value() || !second.has_value() ||
                !isEqualPairSupported(*first, *second)) {
                return unsupportedConstraint("Equal supports line-line or circular curve pairs only");
            }
            return {};
        }

        case ConstraintType::Distance: {
            const auto* c = dynamic_cast<const DistanceConstraint*>(&constraint);
            if (!c) {
                return unsupportedConstraint("Distance constraint implementation missing");
            }
            const auto first = entityType(c->entity1());
            const auto second = entityType(c->entity2());
            if (!first.has_value() || !second.has_value() ||
                !isDistancePairSupported(*first, *second)) {
                return unsupportedConstraint("Distance supports point/line combinations only");
            }
            return {};
        }

        case ConstraintType::Angle: {
            const auto* c = dynamic_cast<const AngleConstraint*>(&constraint);
            if (!c || !hasType(*this, c->line1(), EntityType::Line) ||
                !hasType(*this, c->line2(), EntityType::Line)) {
                return unsupportedConstraint("Angle requires two lines");
            }
            return {};
        }

        case ConstraintType::Radius: {
            const auto* c = dynamic_cast<const RadiusConstraint*>(&constraint);
            std::optional<EntityType> type;
            if (c) {
                type = entityType(c->entityId());
            }
            if (!type.has_value() || (*type != EntityType::Arc && *type != EntityType::Circle)) {
                return unsupportedConstraint("Radius requires an arc or circle");
            }
            return {};
        }

        case ConstraintType::Concentric: {
            const auto* c = dynamic_cast<const ConcentricConstraint*>(&constraint);
            if (!c) {
                return unsupportedConstraint("Concentric constraint implementation missing");
            }
            const auto first = entityType(c->entity1());
            const auto second = entityType(c->entity2());
            if (!first.has_value() || !second.has_value() ||
                !isCircularCurve(*first) || !isCircularCurve(*second)) {
                return unsupportedConstraint("Concentric requires two arcs/circles");
            }
            return {};
        }
        case ConstraintType::Diameter: {
            const auto* c = dynamic_cast<const DiameterConstraint*>(&constraint);
            std::optional<EntityType> type;
            if (c) {
                type = entityType(c->entityId());
            }
            if (!type.has_value() || !isCircularCurve(*type)) {
                return unsupportedConstraint("Diameter requires an arc or circle");
            }
            return {};
        }
        case ConstraintType::HorizontalDistance: {
            const auto* c = dynamic_cast<const HorizontalDistanceConstraint*>(&constraint);
            if (!c || !hasType(*this, c->point1(), EntityType::Point) ||
                !hasType(*this, c->point2(), EntityType::Point)) {
                return unsupportedConstraint("Horizontal Distance requires two points");
            }
            return {};
        }
        case ConstraintType::VerticalDistance: {
            const auto* c = dynamic_cast<const VerticalDistanceConstraint*>(&constraint);
            if (!c || !hasType(*this, c->point1(), EntityType::Point) ||
                !hasType(*this, c->point2(), EntityType::Point)) {
                return unsupportedConstraint("Vertical Distance requires two points");
            }
            return {};
        }
        case ConstraintType::Symmetric: {
            const auto* c = dynamic_cast<const SymmetricConstraint*>(&constraint);
            if (!c || !hasType(*this, c->point1(), EntityType::Point) ||
                !hasType(*this, c->point2(), EntityType::Point) ||
                !hasType(*this, c->axisLine(), EntityType::Line)) {
                return unsupportedConstraint("Symmetric requires two points and an axis line");
            }
            return {};
        }
    }

    return unsupportedConstraint("Unknown constraint type");
}

ConstraintID Sketch::addConstraint(std::unique_ptr<SketchConstraint> constraint) {
    if (!constraint) {
        qCWarning(logSketchEngine) << "addConstraint:null";
        return {};
    }

    qCDebug(logSketchEngine) << "addConstraint:start"
                             << "type=" << static_cast<int>(constraint->type())
                             << "id=" << QString::fromStdString(constraint->id());

    for (const auto& entityId : constraint->referencedEntities()) {
        const SketchEntity* referenced = getEntity(entityId);
        if (entityId.empty() || !referenced) {
            qCWarning(logSketchEngine) << "addConstraint:invalid-reference"
                                      << QString::fromStdString(entityId);
            return {};
        }
        if (referenced->isReferenceLocked() && constraint->type() != ConstraintType::Fixed) {
            qCWarning(logSketchEngine) << "addConstraint:reference-locked"
                                      << QString::fromStdString(entityId)
                                      << "constraintType=" << static_cast<int>(constraint->type());
            return {};
        }
    }

    const ConstraintSupportResult support = validateConstraintSupport(*constraint);
    if (!support.supported) {
        qCWarning(logSketchEngine) << "addConstraint:unsupported"
                                  << "type=" << static_cast<int>(constraint->type())
                                  << "reason=" << QString::fromStdString(support.reason);
        return {};
    }

    ConstraintID id = constraint->id();
    constraintIndex_[id] = constraints_.size();
    constraints_.push_back(std::move(constraint));

    invalidateSolver();
    qCDebug(logSketchEngine) << "addConstraint:done"
                             << "id=" << QString::fromStdString(id)
                             << "totalConstraints=" << constraints_.size();
    return id;
}

ConstraintID Sketch::addCoincident(EntityID point1, EntityID point2) {
    return addConstraint(std::make_unique<constraints::CoincidentConstraint>(point1, point2));
}

ConstraintID Sketch::addHorizontal(EntityID lineOrPoint1, EntityID point2) {
    EntityID lineId = lineOrPoint1;
    if (!point2.empty()) {
        for (const auto& entity : entities_) {
            auto* line = dynamic_cast<SketchLine*>(entity.get());
            if (!line) {
                continue;
            }
            bool matches = (line->startPointId() == lineOrPoint1 && line->endPointId() == point2) ||
                           (line->startPointId() == point2 && line->endPointId() == lineOrPoint1);
            if (matches) {
                lineId = line->id();
                break;
            }
        }
    }

    if (!getEntityAs<SketchLine>(lineId)) {
        return {};
    }

    return addConstraint(std::make_unique<constraints::HorizontalConstraint>(lineId));
}

ConstraintID Sketch::addVertical(EntityID lineOrPoint1, EntityID point2) {
    EntityID lineId = lineOrPoint1;
    if (!point2.empty()) {
        for (const auto& entity : entities_) {
            auto* line = dynamic_cast<SketchLine*>(entity.get());
            if (!line) {
                continue;
            }
            bool matches = (line->startPointId() == lineOrPoint1 && line->endPointId() == point2) ||
                           (line->startPointId() == point2 && line->endPointId() == lineOrPoint1);
            if (matches) {
                lineId = line->id();
                break;
            }
        }
    }

    if (!getEntityAs<SketchLine>(lineId)) {
        return {};
    }

    return addConstraint(std::make_unique<constraints::VerticalConstraint>(lineId));
}

ConstraintID Sketch::addParallel(EntityID line1, EntityID line2) {
    if (!getEntityAs<SketchLine>(line1) || !getEntityAs<SketchLine>(line2)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::ParallelConstraint>(line1, line2));
}

ConstraintID Sketch::addPerpendicular(EntityID line1, EntityID line2) {
    if (!getEntityAs<SketchLine>(line1) || !getEntityAs<SketchLine>(line2)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::PerpendicularConstraint>(line1, line2));
}

ConstraintID Sketch::addDistance(EntityID entity1, EntityID entity2, double distance) {
    if (!getEntity(entity1) || !getEntity(entity2)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::DistanceConstraint>(entity1, entity2, distance));
}

ConstraintID Sketch::addHorizontalDistance(EntityID point1, EntityID point2, double distance) {
    if (!getEntityAs<SketchPoint>(point1) || !getEntityAs<SketchPoint>(point2)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::HorizontalDistanceConstraint>(point1, point2, distance));
}

ConstraintID Sketch::addVerticalDistance(EntityID point1, EntityID point2, double distance) {
    if (!getEntityAs<SketchPoint>(point1) || !getEntityAs<SketchPoint>(point2)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::VerticalDistanceConstraint>(point1, point2, distance));
}

ConstraintID Sketch::addRadius(EntityID arcOrCircle, double radius) {
    if (!getEntityAs<SketchArc>(arcOrCircle) && !getEntityAs<SketchCircle>(arcOrCircle)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::RadiusConstraint>(arcOrCircle, radius));
}

ConstraintID Sketch::addDiameter(EntityID arcOrCircle, double diameter) {
    if (!getEntityAs<SketchArc>(arcOrCircle) && !getEntityAs<SketchCircle>(arcOrCircle)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::DiameterConstraint>(arcOrCircle, diameter));
}

ConstraintID Sketch::addConcentric(EntityID entity1, EntityID entity2) {
    const auto* first = getEntity(entity1);
    const auto* second = getEntity(entity2);
    if (!first || !second || !isCircularCurve(first->type()) || !isCircularCurve(second->type())) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::ConcentricConstraint>(entity1, entity2));
}

ConstraintID Sketch::addSymmetric(EntityID point1, EntityID point2, EntityID axisLine) {
    if (!getEntityAs<SketchPoint>(point1) || !getEntityAs<SketchPoint>(point2) ||
        !getEntityAs<SketchLine>(axisLine)) {
        return {};
    }
    return addConstraint(std::make_unique<constraints::SymmetricConstraint>(point1, point2, axisLine));
}

ConstraintID Sketch::addAngle(EntityID line1, EntityID line2, double angleDegrees) {
    if (!getEntityAs<SketchLine>(line1) || !getEntityAs<SketchLine>(line2)) {
        return {};
    }
    double radians = angleDegrees * std::numbers::pi_v<double> / 180.0;
    return addConstraint(std::make_unique<constraints::AngleConstraint>(line1, line2, radians));
}

ConstraintID Sketch::addFixed(EntityID entityId) {
    auto* point = getEntityAs<SketchPoint>(entityId);
    if (!point) {
        return {};
    }
    double x = point->position().X();
    double y = point->position().Y();
    return addConstraint(
        std::make_unique<constraints::FixedConstraint>(entityId, x, y));
}

bool Sketch::hasFixedConstraint(EntityID pointId) const {
    for (const auto& constraint : constraints_) {
        if (constraint && constraint->type() == ConstraintType::Fixed &&
            constraint->references(pointId)) {
            return true;
        }
    }
    return false;
}

void Sketch::translatePlaneInSketch(const Vec2d& deltaSketch) {
    plane_.origin.x += deltaSketch.x * plane_.xAxis.x + deltaSketch.y * plane_.yAxis.x;
    plane_.origin.y += deltaSketch.x * plane_.xAxis.y + deltaSketch.y * plane_.yAxis.y;
    plane_.origin.z += deltaSketch.x * plane_.xAxis.z + deltaSketch.y * plane_.yAxis.z;
}

void Sketch::translateSketch(double dx, double dy) {
    for (auto& entity : entities_) {
        if (!entity) {
            continue;
        }
        auto* point = dynamic_cast<SketchPoint*>(entity.get());
        if (point && !point->isReferenceLocked()) {
            gp_Pnt2d p = point->position();
            point->setPosition(p.X() + dx, p.Y() + dy);
        }
    }
    for (auto& constraint : constraints_) {
        if (constraint && constraint->type() == ConstraintType::Fixed) {
            auto* fc = dynamic_cast<constraints::FixedConstraint*>(constraint.get());
            if (fc && !isEntityReferenceLocked(fc->pointId())) {
                fc->translate(dx, dy);
            }
        }
    }
    invalidateSolver();
    dofDirty_ = true;
}

void Sketch::translateSketchRegion(const std::string& regionId, double dx, double dy) {
    if (regionId.empty()) {
        return;
    }
    std::vector<EntityID> entityIds = onecad::core::loop::getEntityIdsInRegion(*this, regionId);
    std::unordered_set<EntityID> pointIds;
    for (const auto& id : entityIds) {
        if (getEntityAs<SketchPoint>(id)) {
            pointIds.insert(id);
        }
    }
    for (auto& entity : entities_) {
        if (!entity || pointIds.find(entity->id()) == pointIds.end()) {
            continue;
        }
        auto* point = dynamic_cast<SketchPoint*>(entity.get());
        if (point && !point->isReferenceLocked()) {
            gp_Pnt2d p = point->position();
            point->setPosition(p.X() + dx, p.Y() + dy);
        }
    }
    for (auto& constraint : constraints_) {
        if (!constraint || constraint->type() != ConstraintType::Fixed) {
            continue;
        }
        auto* fc = dynamic_cast<constraints::FixedConstraint*>(constraint.get());
        if (fc &&
            pointIds.find(fc->pointId()) != pointIds.end() &&
            !isEntityReferenceLocked(fc->pointId())) {
            fc->translate(dx, dy);
        }
    }
    invalidateSolver();
    dofDirty_ = true;
}

void Sketch::setHostFaceAttachment(const std::string& bodyId, const std::string& faceId) {
    if (bodyId.empty() || faceId.empty()) {
        hostFaceAttachment_.reset();
        return;
    }
    hostFaceAttachment_ = HostFaceAttachment{bodyId, faceId, 0};
}

void Sketch::setProjectedHostBoundariesVersion(int version) {
    if (!hostFaceAttachment_) {
        return;
    }
    hostFaceAttachment_->projectedBoundaryVersion = std::max(0, version);
}

ConstraintID Sketch::addPointOnCurve(EntityID pointId, EntityID curveId,
                                      CurvePosition position) {
    // Validate point exists
    if (!getEntityAs<SketchPoint>(pointId)) {
        return {};
    }

    // Validate curve exists and get its type
    auto* curve = getEntity(curveId);
    if (!curve) {
        return {};
    }

    auto curveType = curve->type();
    if (curveType != EntityType::Arc && curveType != EntityType::Circle &&
        curveType != EntityType::Line) {
        qCWarning(logSketchEngine) << "addPointOnCurve:unsupported-curve"
                                  << QString::fromStdString(curveId)
                                  << "type=" << static_cast<int>(curveType);
        return {};
    }

    // Auto-detect position if Arbitrary and curve is Arc
    CurvePosition finalPosition = position;
    if (position == CurvePosition::Arbitrary && curveType == EntityType::Arc) {
        finalPosition = detectArcPosition(pointId, curveId);
        if (finalPosition != CurvePosition::Arbitrary) {
            qCWarning(logSketchEngine) << "addPointOnCurve:unsupported-arc-endpoint"
                                      << "point=" << QString::fromStdString(pointId)
                                      << "arc=" << QString::fromStdString(curveId);
            return {};
        }
    }

    return addConstraint(std::make_unique<constraints::PointOnCurveConstraint>(
        pointId, curveId, finalPosition));
}

CurvePosition Sketch::detectArcPosition(EntityID pointId, EntityID arcId) const {
    constexpr double kPositionTolerance = 1e-6;  // mm

    auto* point = getEntityAs<SketchPoint>(pointId);
    auto* arc = getEntityAs<SketchArc>(arcId);
    if (!point || !arc) {
        return CurvePosition::Arbitrary;
    }

    auto* centerPt = getEntityAs<SketchPoint>(arc->centerPointId());
    if (!centerPt) {
        return CurvePosition::Arbitrary;
    }

    gp_Pnt2d centerPos = centerPt->position();
    gp_Pnt2d startPos = arc->startPoint(centerPos);
    gp_Pnt2d endPos = arc->endPoint(centerPos);
    gp_Pnt2d testPos = point->position();

    if (testPos.Distance(startPos) < kPositionTolerance) {
        return CurvePosition::Start;
    }
    if (testPos.Distance(endPos) < kPositionTolerance) {
        return CurvePosition::End;
    }
    return CurvePosition::Arbitrary;
}

bool Sketch::removeConstraint(ConstraintID id) {
    qCDebug(logSketchEngine) << "removeConstraint" << QString::fromStdString(id);
    auto it = constraintIndex_.find(id);
    if (it == constraintIndex_.end()) {
        qCWarning(logSketchEngine) << "removeConstraint:not-found" << QString::fromStdString(id);
        return false;
    }

    if (it->second >= constraints_.size()) {
        rebuildConstraintIndex();
        it = constraintIndex_.find(id);
        if (it == constraintIndex_.end() || it->second >= constraints_.size()) {
            return false;
        }
    }

    const auto* constraint = constraints_[it->second].get();
    if (constraint) {
        for (const auto& entityId : constraint->referencedEntities()) {
            const SketchEntity* entity = getEntity(entityId);
            if (entity && entity->isReferenceLocked()) {
                return false;
            }
        }
    }

    constraints_.erase(constraints_.begin() + static_cast<long>(it->second));
    rebuildConstraintIndex();
    invalidateSolver();
    return true;
}

SketchConstraint* Sketch::getConstraint(ConstraintID id) {
    auto it = constraintIndex_.find(id);
    if (it == constraintIndex_.end()) {
        return nullptr;
    }
    if (it->second >= constraints_.size()) {
        rebuildConstraintIndex();
        it = constraintIndex_.find(id);
        if (it == constraintIndex_.end() || it->second >= constraints_.size()) {
            return nullptr;
        }
    }
    return constraints_[it->second].get();
}

const SketchConstraint* Sketch::getConstraint(ConstraintID id) const {
    auto it = constraintIndex_.find(id);
    if (it == constraintIndex_.end() || it->second >= constraints_.size()) {
        return nullptr;
    }
    return constraints_[it->second].get();
}

std::vector<SketchConstraint*> Sketch::getConstraintsForEntity(EntityID entityId) {
    std::vector<SketchConstraint*> results;
    for (const auto& constraint : constraints_) {
        if (constraint && constraint->references(entityId)) {
            results.push_back(constraint.get());
        }
    }
    return results;
}

std::vector<Vec2d> Sketch::getPointFreeDirections(EntityID pointId) const {
    if (!getEntityAs<SketchPoint>(pointId)) {
        return {};
    }

    bool removedX = false;
    bool removedY = false;

    // Constraints that reference the point directly
    for (const auto& constraint : constraints_) {
        if (!constraint || !constraint->references(pointId)) {
            continue;
        }
        if (constraint->type() == ConstraintType::Fixed ||
            constraint->type() == ConstraintType::Coincident) {
            return {};  // 0 free DOF
        }
    }

    // Line constraints (Horizontal/Vertical) that affect this point via a line endpoint
    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }
        auto* line = dynamic_cast<SketchLine*>(entity.get());
        if (!line) {
            continue;
        }
        if (line->startPointId() != pointId && line->endPointId() != pointId) {
            continue;
        }
        for (const auto& constraint : constraints_) {
            if (!constraint || !constraint->references(line->id())) {
                continue;
            }
            if (constraint->type() == ConstraintType::Horizontal) {
                removedY = true;  // line horizontal => point can only move in X
            } else if (constraint->type() == ConstraintType::Vertical) {
                removedX = true;  // line vertical => point can only move in Y
            }
        }
    }

    if (removedX && removedY) {
        return {};
    }
    if (removedX && !removedY) {
        return {{0.0, 1.0}};  // free along Y
    }
    if (!removedX && removedY) {
        return {{1.0, 0.0}};  // free along X
    }
    return {{1.0, 0.0}, {0.0, 1.0}};  // full plane
}

SolveResult Sketch::solve() {
    SolveResult result;

    if (constraints_.empty()) {
        lastConflictingConstraints_.clear();
        result.success = true;
        return result;
    }

    if (const auto unsupported = firstUnsupportedConstraint(*this)) {
        lastConflictingConstraints_.clear();
        result.success = false;
        result.errorMessage = *unsupported;
        qCWarning(logSketchEngine) << "solve:unsupported-constraint"
                                  << QString::fromStdString(*unsupported);
        return result;
    }

    if (!solver_ || solverDirty_) {
        rebuildSolver();
    }

    if (!solver_) {
        result.success = false;
        result.errorMessage = "Solver not available";
        return result;
    }

    SolverResult solverResult = solver_->solve();
    result.success = solverResult.success;
    result.iterations = solverResult.iterations;
    result.residual = solverResult.residual;
    result.conflictingConstraints = solverResult.conflictingConstraints;
    lastConflictingConstraints_ = solverResult.conflictingConstraints;
    result.errorMessage = solverResult.errorMessage;
    return result;
}

void Sketch::beginPointDrag(EntityID draggedPoint) {
    activeDragFixedPoints_.clear();
    isDraggingPoint_ = false;
    dragStartPositions_.clear();
    dragSessionHadFailure_ = false;

    if (draggedPoint.empty() || !getEntityAs<SketchPoint>(draggedPoint)) {
        return;
    }

    std::unordered_set<EntityID> allPointIds;
    allPointIds.reserve(entities_.size());
    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }
        auto* point = dynamic_cast<SketchPoint*>(entity.get());
        if (point) {
            allPointIds.insert(point->id());
        }
    }

    bool usedRectangleStrategy = false;
    auto regionId = onecad::core::loop::getRegionIdContainingEntity(*this, draggedPoint);
    if (regionId.has_value()) {
        auto face = onecad::core::loop::resolveRegionFace(*this, *regionId);
        if (face.has_value()) {
            std::vector<EntityID> boundaryPointIds =
                onecad::core::loop::getOrderedBoundaryPointIds(*this, face->outerLoop);
            if (boundaryPointIds.size() == 4) {
                auto pointIt = std::find(boundaryPointIds.begin(), boundaryPointIds.end(), draggedPoint);
                if (pointIt != boundaryPointIds.end()) {
                    size_t idx = static_cast<size_t>(std::distance(boundaryPointIds.begin(), pointIt));
                    activeDragFixedPoints_.insert(boundaryPointIds[(idx + 2) % boundaryPointIds.size()]);
                    usedRectangleStrategy = true;
                }
            }
        }
    }

    if (!usedRectangleStrategy) {
        for (const auto& pointId : allPointIds) {
            if (pointId != draggedPoint) {
                activeDragFixedPoints_.insert(pointId);
            }
        }
    }

    dragStartPositions_.reserve(allPointIds.size());
    for (const auto& pointId : allPointIds) {
        const auto* point = getEntityAs<SketchPoint>(pointId);
        if (!point) {
            continue;
        }
        dragStartPositions_[pointId] = Vec2d{point->position().X(), point->position().Y()};
    }

    isDraggingPoint_ = true;
}

void Sketch::endPointDrag() {
    if (dragSessionHadFailure_) {
        for (const auto& [pointId, startPos] : dragStartPositions_) {
            auto* point = getEntityAs<SketchPoint>(pointId);
            if (!point) {
                continue;
            }
            point->setPosition(startPos.x, startPos.y);
        }
        invalidateSolver();
    }

    dragStartPositions_.clear();
    dragSessionHadFailure_ = false;
    activeDragFixedPoints_.clear();
    isDraggingPoint_ = false;
}

void Sketch::beginGroupDrag(const std::unordered_set<EntityID>& selectedPointIds) {
    activeGroupDragPoints_.clear();
    groupDragStartPose_.clear();
    isDraggingGroup_ = false;

    if (selectedPointIds.empty()) {
        return;
    }

    activeGroupDragPoints_.reserve(selectedPointIds.size());
    groupDragStartPose_.reserve(selectedPointIds.size());
    for (const auto& pointId : selectedPointIds) {
        const auto* point = getEntityAs<SketchPoint>(pointId);
        if (!point) {
            continue;
        }
        activeGroupDragPoints_.insert(pointId);
        groupDragStartPose_[pointId] = Vec2d{point->position().X(), point->position().Y()};
    }

    isDraggingGroup_ = !activeGroupDragPoints_.empty();
}

void Sketch::restoreGroupDragStartPose() {
    for (const auto& [pointId, startPos] : groupDragStartPose_) {
        auto* point = getEntityAs<SketchPoint>(pointId);
        if (!point) {
            continue;
        }
        point->setPosition(startPos.x, startPos.y);
    }
    invalidateSolver();
}

SolveResult Sketch::solveWithGroupDrag(
    const std::unordered_map<EntityID, Vec2d>& targetPositions) {
    SolveResult result;
    constexpr double rigidTargetTolerance = 1e-6;
    auto enforceAtomicReject = [&](const std::string& fallbackMessage) {
        restoreGroupDragStartPose();
        result.success = false;
        if (result.errorMessage.empty()) {
            result.errorMessage = fallbackMessage;
        }
    };

    if (!isDraggingGroup_) {
        result.success = false;
        result.errorMessage = "Group drag session not active";
        return result;
    }

    if (activeGroupDragPoints_.empty()) {
        result.success = false;
        result.errorMessage = "No selected points in group drag";
        return result;
    }

    std::optional<EntityID> anchorPointId;
    for (const auto& [pointId, _] : targetPositions) {
        if (activeGroupDragPoints_.find(pointId) != activeGroupDragPoints_.end() &&
            groupDragStartPose_.find(pointId) != groupDragStartPose_.end()) {
            anchorPointId = pointId;
            break;
        }
    }

    if (!anchorPointId.has_value()) {
        result.success = false;
        result.errorMessage = "No valid group drag anchor target";
        return result;
    }

    const auto anchorTargetIt = targetPositions.find(*anchorPointId);
    const auto anchorStartIt = groupDragStartPose_.find(*anchorPointId);
    if (anchorTargetIt == targetPositions.end() || anchorStartIt == groupDragStartPose_.end()) {
        result.success = false;
        result.errorMessage = "Invalid group drag anchor";
        return result;
    }

    const Vec2d delta{
        .x = anchorTargetIt->second.x - anchorStartIt->second.x,
        .y = anchorTargetIt->second.y - anchorStartIt->second.y,
    };

    std::unordered_map<EntityID, Vec2d> rigidTargets;
    rigidTargets.reserve(activeGroupDragPoints_.size());
    for (const auto& pointId : activeGroupDragPoints_) {
        const auto startIt = groupDragStartPose_.find(pointId);
        if (startIt == groupDragStartPose_.end()) {
            result.success = false;
            result.errorMessage = "Group drag start pose incomplete";
            return result;
        }
        rigidTargets[pointId] = Vec2d{.x = startIt->second.x + delta.x, .y = startIt->second.y + delta.y};
    }

    if (constraints_.empty()) {
        lastConflictingConstraints_.clear();
        for (const auto& [pointId, rigidTarget] : rigidTargets) {
            auto* point = getEntityAs<SketchPoint>(pointId);
            if (!point) {
                result.errorMessage = "Group drag point not found";
                enforceAtomicReject("Group drag rejected");
                return result;
            }
            point->setPosition(rigidTarget.x, rigidTarget.y);
        }
        result.success = true;
        return result;
    }

    if (const auto unsupported = firstUnsupportedConstraint(*this)) {
        result.errorMessage = *unsupported;
        enforceAtomicReject("Group drag rejected by unsupported constraint");
        qCWarning(logSketchEngine) << "solveWithGroupDrag:unsupported-constraint"
                                  << QString::fromStdString(*unsupported);
        return result;
    }

    if (!solver_ || solverDirty_) {
        rebuildSolver();
    }

    if (!solver_) {
        result.success = false;
        result.errorMessage = "Solver not available";
        return result;
    }

    SolverResult solverResult = solver_->solveWithGroupDrag(rigidTargets);
    result.success = solverResult.success;
    result.iterations = solverResult.iterations;
    result.residual = solverResult.residual;
    result.conflictingConstraints = solverResult.conflictingConstraints;
    lastConflictingConstraints_ = solverResult.conflictingConstraints;
    result.errorMessage = solverResult.errorMessage;

    if (result.success) {
        for (const auto& [pointId, rigidTarget] : rigidTargets) {
            const auto* point = getEntityAs<SketchPoint>(pointId);
            if (!point) {
                result.errorMessage = "Group drag point not found after solve";
                enforceAtomicReject("Group drag rejected");
                return result;
            }

            const Vec2d solvedPos{point->position().X(), point->position().Y()};
            const double dx = std::abs(solvedPos.x - rigidTarget.x);
            const double dy = std::abs(solvedPos.y - rigidTarget.y);
            if (dx > rigidTargetTolerance || dy > rigidTargetTolerance) {
                result.errorMessage = "Group drag rejected: rigid translation not achievable";
                enforceAtomicReject("Group drag rejected");
                return result;
            }
        }
    } else {
        enforceAtomicReject("Group drag rejected by constraints");
    }

    return result;
}

void Sketch::endGroupDrag() {
    activeGroupDragPoints_.clear();
    groupDragStartPose_.clear();
    isDraggingGroup_ = false;
}

SolveResult Sketch::solveWithDrag(EntityID draggedPoint, const Vec2d& targetPos) {
    SolveResult result;

    auto* point = getEntityAs<SketchPoint>(draggedPoint);
    if (!point) {
        result.success = false;
        result.errorMessage = "Dragged point not found";
        return result;
    }
    if (point->isReferenceLocked()) {
        result.success = false;
        result.errorMessage = "Point is locked";
        return result;
    }

    if (constraints_.empty()) {
        lastConflictingConstraints_.clear();
        point->setPosition(targetPos.x, targetPos.y);
        result.success = true;
        return result;
    }

    if (const auto unsupported = firstUnsupportedConstraint(*this)) {
        result.success = false;
        result.errorMessage = *unsupported;
        qCWarning(logSketchEngine) << "solveWithDrag:unsupported-constraint"
                                  << QString::fromStdString(*unsupported);
        if (isDraggingPoint_) {
            dragSessionHadFailure_ = true;
        }
        return result;
    }

    if (!solver_ || solverDirty_) {
        rebuildSolver();
    }

    if (!solver_) {
        result.success = false;
        result.errorMessage = "Solver not available";
        return result;
    }

    static const std::unordered_set<EntityID> kNoFixedPoints;
    const std::unordered_set<EntityID>& pointIdsToFix =
        isDraggingPoint_ ? activeDragFixedPoints_ : kNoFixedPoints;

    SolverResult solverResult = solver_->solveWithDrag(draggedPoint, targetPos, pointIdsToFix);
    result.success = solverResult.success;
    result.iterations = solverResult.iterations;
    result.residual = solverResult.residual;
    result.conflictingConstraints = solverResult.conflictingConstraints;
    lastConflictingConstraints_ = solverResult.conflictingConstraints;
    result.errorMessage = solverResult.errorMessage;

    if (isDraggingPoint_ && !result.success) {
        dragSessionHadFailure_ = true;
    }

    return result;
}

int Sketch::naiveDegreesOfFreedom() const {
    // Static count: entity DOF minus constraint arity. Wrong in the presence
    // of redundant constraints (each still subtracts); kept only as the
    // fallback for sketches the solver cannot represent (ellipses).
    int total = 0;
    for (const auto& entity : entities_) {
        if (entity) {
            total += entity->degreesOfFreedom();
        }
    }
    for (const auto& constraint : constraints_) {
        if (constraint) {
            total -= constraint->degreesRemoved();
        }
    }
    return total;
}

bool Sketch::hasSolverUnsupportedEntities() const {
    for (const auto& entity : entities_) {
        if (entity && entity->type() == EntityType::Ellipse) {
            return true;  // ellipses are not registered with PlaneGCS
        }
    }
    return false;
}

int Sketch::getDegreesOfFreedom() const {
    if (!dofDirty_ && cachedDOF_ >= 0) {
        return cachedDOF_;
    }

    int dof = std::max(naiveDegreesOfFreedom(), 0);
    if (!hasSolverUnsupportedEntities()) {
        // PlaneGCS diagnosis gives the TRUE remaining DOF: redundant
        // constraints no longer make a fully-defined sketch read as
        // over-constrained. Solver rebuild mutates cache state only.
        auto* self = const_cast<Sketch*>(this);
        if (self->solverDirty_ || !self->solver_) {
            self->rebuildSolver();
        }
        if (self->solver_) {
            const int diagnosed = self->solver_->diagnose();
            if (diagnosed >= 0) {
                dof = diagnosed;
            }
        }
    }

    cachedDOF_ = dof;
    dofDirty_ = false;
    return cachedDOF_;
}

bool Sketch::isOverConstrained() const {
    if (!hasSolverUnsupportedEntities()) {
        auto* self = const_cast<Sketch*>(this);
        if (self->solverDirty_ || !self->solver_) {
            self->rebuildSolver();
        }
        if (self->solver_) {
            self->solver_->diagnose();
            return self->solver_->hasConflicting();
        }
    }
    return naiveDegreesOfFreedom() < 0;
}

std::vector<ConstraintID> Sketch::getConflictingConstraints() const {
    return lastConflictingConstraints_;
}

ValidationResult Sketch::validate() const {
    ValidationResult result;

    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }

        if (auto* point = dynamic_cast<SketchPoint*>(entity.get())) {
            if (point->connectedEntities().empty()) {
                result.warnings.push_back("Orphaned point: " + point->id());
                result.invalidEntities.push_back(point->id());
            }
            continue;
        }

        if (auto* line = dynamic_cast<SketchLine*>(entity.get())) {
            auto* start = getEntityAs<SketchPoint>(line->startPointId());
            auto* end = getEntityAs<SketchPoint>(line->endPointId());
            if (!start || !end) {
                result.valid = false;
                result.errors.push_back("Line has missing endpoint: " + line->id());
                result.invalidEntities.push_back(line->id());
                continue;
            }
            double length = SketchLine::length(start->position(), end->position());
            if (length < constants::MIN_GEOMETRY_SIZE) {
                result.valid = false;
                result.errors.push_back("Line length too small: " + line->id());
                result.invalidEntities.push_back(line->id());
            }
            continue;
        }

        if (auto* arc = dynamic_cast<SketchArc*>(entity.get())) {
            if (arc->radius() < constants::MIN_GEOMETRY_SIZE) {
                result.valid = false;
                result.errors.push_back("Arc radius too small: " + arc->id());
                result.invalidEntities.push_back(arc->id());
            }
            continue;
        }

        if (auto* circle = dynamic_cast<SketchCircle*>(entity.get())) {
            if (circle->radius() < constants::MIN_GEOMETRY_SIZE) {
                result.valid = false;
                result.errors.push_back("Circle radius too small: " + circle->id());
                result.invalidEntities.push_back(circle->id());
            }
            continue;
        }

        if (auto* ellipse = dynamic_cast<SketchEllipse*>(entity.get())) {
            if (ellipse->majorRadius() < constants::MIN_GEOMETRY_SIZE ||
                ellipse->minorRadius() < constants::MIN_GEOMETRY_SIZE) {
                result.valid = false;
                result.errors.push_back("Ellipse radii too small: " + ellipse->id());
                result.invalidEntities.push_back(ellipse->id());
            }
        }
    }

    for (const auto& constraint : constraints_) {
        if (!constraint) {
            continue;
        }
        const ConstraintSupportResult support = validateConstraintSupport(*constraint);
        if (!support.supported) {
            result.valid = false;
            result.errors.push_back("Unsupported constraint " + constraint->id() + ": " + support.reason);
            for (const auto& entityId : constraint->referencedEntities()) {
                result.invalidEntities.push_back(entityId);
            }
        }
    }

    return result;
}

std::string Sketch::toJson() const {
    QJsonObject root;
    root["version"] = 1;

    QJsonObject plane;
    plane["origin"] = QJsonArray{plane_.origin.x, plane_.origin.y, plane_.origin.z};
    plane["xAxis"] = QJsonArray{plane_.xAxis.x, plane_.xAxis.y, plane_.xAxis.z};
    plane["yAxis"] = QJsonArray{plane_.yAxis.x, plane_.yAxis.y, plane_.yAxis.z};
    plane["normal"] = QJsonArray{plane_.normal.x, plane_.normal.y, plane_.normal.z};
    root["plane"] = plane;

    if (hostFaceAttachment_ && hostFaceAttachment_->isValid()) {
        QJsonObject hostFace;
        hostFace["bodyId"] = QString::fromStdString(hostFaceAttachment_->bodyId);
        hostFace["faceId"] = QString::fromStdString(hostFaceAttachment_->faceId);
        hostFace["projectedBoundaryVersion"] = hostFaceAttachment_->projectedBoundaryVersion;
        root["hostFace"] = hostFace;
    }

    QJsonArray entityArray;
    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }
        QJsonObject obj;
        entity->serialize(obj);
        entityArray.append(obj);
    }
    root["entities"] = entityArray;

    QJsonArray constraintArray;
    for (const auto& constraint : constraints_) {
        if (!constraint) {
            continue;
        }
        QJsonObject obj;
        constraint->serialize(obj);
        constraintArray.append(obj);
    }
    root["constraints"] = constraintArray;

    QJsonDocument doc(root);
    return doc.toJson(QJsonDocument::Compact).toStdString();
}

std::unique_ptr<Sketch> Sketch::fromJson(const std::string& json) {
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(json).toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return nullptr;
    }

    QJsonObject root = doc.object();
    SketchPlane plane = SketchPlane::XY();
    if (root.contains("plane") && root["plane"].isObject()) {
        QJsonObject planeObj = root["plane"].toObject();
        auto readVec3 = [](const QJsonObject& obj, const char* key, Vec3d& out) {
            if (!obj.contains(key) || !obj[key].isArray()) {
                return false;
            }
            QJsonArray arr = obj[key].toArray();
            if (arr.size() != 3) {
                return false;
            }
            if (!arr[0].isDouble() || !arr[1].isDouble() || !arr[2].isDouble()) {
                return false;
            }
            out.x = arr[0].toDouble();
            out.y = arr[1].toDouble();
            out.z = arr[2].toDouble();
            return true;
        };

        readVec3(planeObj, "origin", plane.origin);
        readVec3(planeObj, "xAxis", plane.xAxis);
        readVec3(planeObj, "yAxis", plane.yAxis);
        readVec3(planeObj, "normal", plane.normal);
    }

    auto sketch = std::make_unique<Sketch>(plane);

    if (root.contains("hostFace") && root["hostFace"].isObject()) {
        const QJsonObject hostFace = root["hostFace"].toObject();
        const QString bodyId = hostFace["bodyId"].toString();
        const QString faceId = hostFace["faceId"].toString();
        if (!bodyId.isEmpty() && !faceId.isEmpty()) {
            if (!hostFace.contains("projectedBoundaryVersion") ||
                !hostFace["projectedBoundaryVersion"].isDouble()) {
                return nullptr;
            }
            sketch->setHostFaceAttachment(bodyId.toStdString(), faceId.toStdString());
            sketch->setProjectedHostBoundariesVersion(hostFace["projectedBoundaryVersion"].toInt(0));
        }
    }

    if (root.contains("entities") && root["entities"].isArray()) {
        QJsonArray entities = root["entities"].toArray();
        for (const auto& value : entities) {
            if (!value.isObject()) {
                return nullptr;
            }
            QJsonObject obj = value.toObject();
            if (!obj.contains("type") || !obj["type"].isString()) {
                return nullptr;
            }

            std::string type = obj["type"].toString().toStdString();
            std::unique_ptr<SketchEntity> entity;
            if (type == "Point") {
                entity = std::make_unique<SketchPoint>();
            } else if (type == "Line") {
                entity = std::make_unique<SketchLine>();
            } else if (type == "Arc") {
                entity = std::make_unique<SketchArc>();
            } else if (type == "Circle") {
                entity = std::make_unique<SketchCircle>();
            } else if (type == "Ellipse") {
                entity = std::make_unique<SketchEllipse>();
            } else {
                return nullptr;
            }

            if (!entity->deserialize(obj)) {
                return nullptr;
            }

            EntityID id = entity->id();
            sketch->entityIndex_[id] = sketch->entities_.size();
            sketch->entities_.push_back(std::move(entity));
        }
    }

    for (const auto& entity : sketch->entities_) {
        if (!entity) {
            continue;
        }
        if (auto* line = dynamic_cast<SketchLine*>(entity.get())) {
            if (auto* start = sketch->getEntityAs<SketchPoint>(line->startPointId())) {
                start->addConnectedEntity(line->id());
            }
            if (auto* end = sketch->getEntityAs<SketchPoint>(line->endPointId())) {
                end->addConnectedEntity(line->id());
            }
        } else if (auto* arc = dynamic_cast<SketchArc*>(entity.get())) {
            if (auto* center = sketch->getEntityAs<SketchPoint>(arc->centerPointId())) {
                center->addConnectedEntity(arc->id());
            }
        } else if (auto* circle = dynamic_cast<SketchCircle*>(entity.get())) {
            if (auto* center = sketch->getEntityAs<SketchPoint>(circle->centerPointId())) {
                center->addConnectedEntity(circle->id());
            }
        } else if (auto* ellipse = dynamic_cast<SketchEllipse*>(entity.get())) {
            if (auto* center = sketch->getEntityAs<SketchPoint>(ellipse->centerPointId())) {
                center->addConnectedEntity(ellipse->id());
            }
        }
    }

    if (root.contains("constraints") && root["constraints"].isArray()) {
        QJsonArray constraints = root["constraints"].toArray();
        for (const auto& value : constraints) {
            if (!value.isObject()) {
                return nullptr;
            }
            auto constraint = ConstraintFactory::fromJson(value.toObject());
            if (!constraint) {
                return nullptr;
            }
            if (sketch->addConstraint(std::move(constraint)).empty()) {
                return nullptr;
            }
        }
    }

    sketch->invalidateSolver();
    return sketch;
}

EntityID Sketch::findNearest(const Vec2d& pos, double tolerance,
                             std::optional<EntityType> filter) const {
    EntityID bestId;
    double bestDistance = tolerance;
    gp_Pnt2d query(pos.x, pos.y);

    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }
        if (filter && entity->type() != *filter) {
            continue;
        }

        double distance = std::numeric_limits<double>::infinity();
        switch (entity->type()) {
            case EntityType::Point: {
                auto* point = dynamic_cast<SketchPoint*>(entity.get());
                distance = point ? point->distanceTo(query) : distance;
                break;
            }
            case EntityType::Line: {
                auto* line = dynamic_cast<SketchLine*>(entity.get());
                if (!line) {
                    break;
                }
                auto* start = getEntityAs<SketchPoint>(line->startPointId());
                auto* end = getEntityAs<SketchPoint>(line->endPointId());
                if (!start || !end) {
                    break;
                }
                distance = SketchLine::distanceToPoint(query, start->position(), end->position());
                break;
            }
            case EntityType::Arc: {
                auto* arc = dynamic_cast<SketchArc*>(entity.get());
                if (!arc) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(arc->centerPointId());
                if (!center) {
                    break;
                }
                double radial = std::abs(center->position().Distance(query) - arc->radius());
                if (arc->isNearWithCenter(query, center->position(), tolerance)) {
                    distance = radial;
                }
                break;
            }
            case EntityType::Circle: {
                auto* circle = dynamic_cast<SketchCircle*>(entity.get());
                if (!circle) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(circle->centerPointId());
                if (!center) {
                    break;
                }
                distance = std::abs(center->position().Distance(query) - circle->radius());
                break;
            }
            case EntityType::Ellipse: {
                auto* ellipse = dynamic_cast<SketchEllipse*>(entity.get());
                if (!ellipse) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(ellipse->centerPointId());
                if (!center) {
                    break;
                }
                constexpr int kSamples = 72;
                double minDist = std::numeric_limits<double>::infinity();
                gp_Pnt2d centerPos = center->position();
                double step = 2.0 * std::numbers::pi / static_cast<double>(kSamples);
                for (int i = 0; i < kSamples; ++i) {
                    double t = step * static_cast<double>(i);
                    gp_Pnt2d point = ellipse->pointAtParameter(centerPos, t);
                    minDist = std::min(minDist, point.Distance(query));
                }
                if (minDist <= tolerance) {
                    distance = minDist;
                }
                break;
            }
            default:
                break;
        }

        if (distance <= bestDistance) {
            bestDistance = distance;
            bestId = entity->id();
        }
    }

    return bestId;
}

std::vector<EntityID> Sketch::findInRect(const Vec2d& min, const Vec2d& max) const {
    std::vector<EntityID> results;
    BoundingBox2d rect;
    rect.minX = std::min(min.x, max.x);
    rect.minY = std::min(min.y, max.y);
    rect.maxX = std::max(min.x, max.x);
    rect.maxY = std::max(min.y, max.y);

    for (const auto& entity : entities_) {
        if (!entity) {
            continue;
        }

        BoundingBox2d bounds;
        switch (entity->type()) {
            case EntityType::Point: {
                auto* point = dynamic_cast<SketchPoint*>(entity.get());
                if (point) {
                    bounds = point->bounds();
                }
                break;
            }
            case EntityType::Line: {
                auto* line = dynamic_cast<SketchLine*>(entity.get());
                if (!line) {
                    break;
                }
                auto* start = getEntityAs<SketchPoint>(line->startPointId());
                auto* end = getEntityAs<SketchPoint>(line->endPointId());
                if (!start || !end) {
                    break;
                }
                bounds = SketchLine::boundsWithPoints(start->position(), end->position());
                break;
            }
            case EntityType::Arc: {
                auto* arc = dynamic_cast<SketchArc*>(entity.get());
                if (!arc) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(arc->centerPointId());
                if (!center) {
                    break;
                }
                bounds = arc->boundsWithCenter(center->position());
                break;
            }
            case EntityType::Circle: {
                auto* circle = dynamic_cast<SketchCircle*>(entity.get());
                if (!circle) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(circle->centerPointId());
                if (!center) {
                    break;
                }
                bounds = circle->boundsWithCenter(center->position());
                break;
            }
            case EntityType::Ellipse: {
                auto* ellipse = dynamic_cast<SketchEllipse*>(entity.get());
                if (!ellipse) {
                    break;
                }
                auto* center = getEntityAs<SketchPoint>(ellipse->centerPointId());
                if (!center) {
                    break;
                }
                bounds = ellipse->boundsWithCenter(center->position());
                break;
            }
            default:
                break;
        }

        if (!bounds.isEmpty() && bounds.intersects(rect)) {
            results.push_back(entity->id());
        }
    }

    return results;
}

void Sketch::invalidateSolver() {
    solverDirty_ = true;
    dofDirty_ = true;
    lastConflictingConstraints_.clear();
    qCDebug(logSketchEngine) << "invalidateSolver"
                             << "entityCount=" << entities_.size()
                             << "constraintCount=" << constraints_.size();
}

void Sketch::rebuildSolver() {
    qCDebug(logSketchEngine) << "rebuildSolver:start"
                             << "entities=" << entities_.size()
                             << "constraints=" << constraints_.size();
    solver_ = std::make_unique<ConstraintSolver>();
    const bool populated = SolverAdapter::populateSolver(*this, *solver_);
    if (!populated) {
        qCWarning(logSketchEngine) << "rebuildSolver:constraint-translation-failed";
        solver_.reset();
        solverDirty_ = true;
        return;
    }

    solverDirty_ = false;
    qCDebug(logSketchEngine) << "rebuildSolver:done";
}

void Sketch::rebuildEntityIndex() {
    entityIndex_.clear();
    for (size_t i = 0; i < entities_.size(); ++i) {
        entityIndex_[entities_[i]->id()] = i;
    }
}

void Sketch::rebuildConstraintIndex() {
    constraintIndex_.clear();
    for (size_t i = 0; i < constraints_.size(); ++i) {
        constraintIndex_[constraints_[i]->id()] = i;
    }
}

} // namespace onecad::core::sketch
