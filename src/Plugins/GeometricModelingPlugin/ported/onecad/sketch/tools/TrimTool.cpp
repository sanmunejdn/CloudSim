#include "TrimTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../SketchEntity.h"
#include "../SketchLine.h"
#include "../SketchArc.h"
#include "../SketchCircle.h"
#include "../SketchEllipse.h"
#include "../SketchPoint.h"
#include "../SnapManager.h"

#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace onecad::core::sketch::tools {

namespace {

constexpr double kPickTolerance = 3.0;
constexpr double kParamEpsilon = 1e-5;
constexpr double kMinSpan = 1e-4;

struct ParamPoint {
    double t = 0.0;
    Vec2d point{};
};

double normalizeAnglePositive(double angle) {
    const double twoPi = 2.0 * std::numbers::pi_v<double>;
    angle = std::fmod(angle, twoPi);
    if (angle < 0.0) {
        angle += twoPi;
    }
    return angle;
}

double ccwDelta(double from, double to) {
    double delta = normalizeAnglePositive(to - from);
    if (delta < kParamEpsilon) {
        delta += 2.0 * std::numbers::pi_v<double>;
    }
    return delta;
}

void addUniqueParam(std::vector<ParamPoint>& params, double t, const Vec2d& point) {
    t = std::clamp(t, 0.0, 1.0);
    for (const auto& existing : params) {
        if (std::abs(existing.t - t) <= kParamEpsilon) {
            return;
        }
    }
    params.push_back({t, point});
}

std::vector<Vec2d> collectIntersections(const Sketch& sketch, const SketchEntity* target) {
    std::vector<Vec2d> out;
    if (!target) {
        return out;
    }

    SnapManager snapManager;
    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity || entity->id() == target->id() || entity->type() == EntityType::Point) {
            continue;
        }
        auto points = snapManager.findEntityIntersections(target, entity.get(), sketch);
        out.insert(out.end(), points.begin(), points.end());
    }
    return out;
}

std::optional<int> findClickedSpan(const std::vector<ParamPoint>& params, double clickT) {
    if (params.size() < 2) {
        return std::nullopt;
    }
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (params[i + 1].t - params[i].t <= kMinSpan) {
            continue;
        }
        if (clickT >= params[i].t - kParamEpsilon && clickT <= params[i + 1].t + kParamEpsilon) {
            return static_cast<int>(i);
        }
    }
    return std::nullopt;
}

bool trimLineAt(Sketch& sketch, const EntityID& lineId, const Vec2d& click) {
    auto* line = sketch.getEntityAs<SketchLine>(lineId);
    if (!line || line->isReferenceLocked()) {
        return false;
    }
    auto* start = sketch.getEntityAs<SketchPoint>(line->startPointId());
    auto* end = sketch.getEntityAs<SketchPoint>(line->endPointId());
    if (!start || !end) {
        return false;
    }

    gp_Pnt2d p1 = start->position();
    gp_Pnt2d p2 = end->position();
    const double dx = p2.X() - p1.X();
    const double dy = p2.Y() - p1.Y();
    const double lenSq = dx * dx + dy * dy;
    if (lenSq <= 1e-12) {
        return false;
    }

    auto pointAt = [&](double t) -> Vec2d {
        return {p1.X() + dx * t, p1.Y() + dy * t};
    };

    std::vector<ParamPoint> params;
    addUniqueParam(params, 0.0, {p1.X(), p1.Y()});
    addUniqueParam(params, 1.0, {p2.X(), p2.Y()});

    const auto intersections = collectIntersections(sketch, line);
    for (const auto& pt : intersections) {
        const double t = ((pt.x - p1.X()) * dx + (pt.y - p1.Y()) * dy) / lenSq;
        if (t > kParamEpsilon && t < 1.0 - kParamEpsilon) {
            addUniqueParam(params, t, pointAt(t));
        }
    }

    std::sort(params.begin(), params.end(), [](const ParamPoint& lhs, const ParamPoint& rhs) {
        return lhs.t < rhs.t;
    });

    const double clickT = std::clamp(((click.x - p1.X()) * dx + (click.y - p1.Y()) * dy) / lenSq, 0.0, 1.0);
    auto clickedSpan = findClickedSpan(params, clickT);
    if (!clickedSpan.has_value() || params.size() <= 2) {
        return sketch.removeEntity(lineId);
    }

    std::vector<EntityID> pointIds(params.size());
    pointIds.front() = line->startPointId();
    pointIds.back() = line->endPointId();
    const bool construction = line->isConstruction();
    for (size_t i = 1; i + 1 < params.size(); ++i) {
        pointIds[i] = sketch.addPoint(params[i].point.x, params[i].point.y, construction);
        if (pointIds[i].empty()) {
            return false;
        }
    }

    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (static_cast<int>(i) == *clickedSpan || params[i + 1].t - params[i].t <= kMinSpan) {
            continue;
        }
        if (sketch.addLine(pointIds[i], pointIds[i + 1], construction).empty()) {
            return false;
        }
    }

    return sketch.removeEntity(lineId);
}

bool trimArcAt(Sketch& sketch, const EntityID& arcId, const Vec2d& click) {
    auto* arc = sketch.getEntityAs<SketchArc>(arcId);
    if (!arc || arc->isReferenceLocked()) {
        return false;
    }
    auto* centerPt = sketch.getEntityAs<SketchPoint>(arc->centerPointId());
    if (!centerPt) {
        return false;
    }

    const gp_Pnt2d center = centerPt->position();
    const double startAngle = arc->startAngle();
    const double sweep = arc->sweepAngle();
    if (sweep <= kMinSpan) {
        return false;
    }

    auto pointAtAngle = [&](double angle) -> Vec2d {
        return {center.X() + arc->radius() * std::cos(angle),
                center.Y() + arc->radius() * std::sin(angle)};
    };

    std::vector<ParamPoint> params;
    addUniqueParam(params, 0.0, pointAtAngle(startAngle));
    addUniqueParam(params, 1.0, pointAtAngle(arc->endAngle()));

    const auto intersections = collectIntersections(sketch, arc);
    for (const auto& pt : intersections) {
        const double angle = std::atan2(pt.y - center.Y(), pt.x - center.X());
        const double t = ccwDelta(startAngle, angle) / sweep;
        if (t > kParamEpsilon && t < 1.0 - kParamEpsilon && arc->containsAngle(angle)) {
            addUniqueParam(params, t, pointAtAngle(angle));
        }
    }

    std::sort(params.begin(), params.end(), [](const ParamPoint& lhs, const ParamPoint& rhs) {
        return lhs.t < rhs.t;
    });

    const double clickAngle = std::atan2(click.y - center.Y(), click.x - center.X());
    const double clickT = std::clamp(ccwDelta(startAngle, clickAngle) / sweep, 0.0, 1.0);
    auto clickedSpan = findClickedSpan(params, clickT);
    if (!clickedSpan.has_value() || params.size() <= 2) {
        return sketch.removeEntity(arcId);
    }

    const bool construction = arc->isConstruction();
    const EntityID centerId = arc->centerPointId();
    const double radius = arc->radius();
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (static_cast<int>(i) == *clickedSpan || params[i + 1].t - params[i].t <= kMinSpan) {
            continue;
        }
        const double spanStart = startAngle + params[i].t * sweep;
        const double spanEnd = startAngle + params[i + 1].t * sweep;
        if (sketch.addArc(centerId, radius, spanStart, spanEnd, construction).empty()) {
            return false;
        }
    }

    return sketch.removeEntity(arcId);
}

bool trimCircleAt(Sketch& sketch, const EntityID& circleId, const Vec2d& click) {
    auto* circle = sketch.getEntityAs<SketchCircle>(circleId);
    if (!circle || circle->isReferenceLocked()) {
        return false;
    }
    auto* centerPt = sketch.getEntityAs<SketchPoint>(circle->centerPointId());
    if (!centerPt) {
        return false;
    }

    const gp_Pnt2d center = centerPt->position();
    const auto intersections = collectIntersections(sketch, circle);
    std::vector<ParamPoint> params;
    for (const auto& pt : intersections) {
        const double angle = normalizeAnglePositive(std::atan2(pt.y - center.Y(), pt.x - center.X()));
        addUniqueParam(params, angle / (2.0 * std::numbers::pi_v<double>), pt);
    }
    std::sort(params.begin(), params.end(), [](const ParamPoint& lhs, const ParamPoint& rhs) {
        return lhs.t < rhs.t;
    });

    if (params.size() < 2) {
        return sketch.removeEntity(circleId);
    }

    const double clickAngle = normalizeAnglePositive(std::atan2(click.y - center.Y(), click.x - center.X()));
    const double clickT = clickAngle / (2.0 * std::numbers::pi_v<double>);
    int clickedSpan = -1;
    for (size_t i = 0; i < params.size(); ++i) {
        const double from = params[i].t;
        const double to = (i + 1 < params.size()) ? params[i + 1].t : params.front().t + 1.0;
        const double adjustedClick = clickT < from ? clickT + 1.0 : clickT;
        if (adjustedClick >= from - kParamEpsilon && adjustedClick <= to + kParamEpsilon) {
            clickedSpan = static_cast<int>(i);
            break;
        }
    }
    if (clickedSpan < 0) {
        return false;
    }

    const bool construction = circle->isConstruction();
    const EntityID centerId = circle->centerPointId();
    const double radius = circle->radius();
    for (size_t i = 0; i < params.size(); ++i) {
        if (static_cast<int>(i) == clickedSpan) {
            continue;
        }
        const double from = params[i].t * 2.0 * std::numbers::pi_v<double>;
        const double to = ((i + 1 < params.size()) ? params[i + 1].t : params.front().t + 1.0) *
                          2.0 * std::numbers::pi_v<double>;
        if (to - from <= kMinSpan) {
            continue;
        }
        if (sketch.addArc(centerId, radius, from, to, construction).empty()) {
            return false;
        }
    }

    return sketch.removeEntity(circleId);
}

bool trimEntityAtPosition(Sketch& sketch, const EntityID& entityId, const Vec2d& pos) {
    const auto* entity = sketch.getEntity(entityId);
    if (!entity || entity->type() == EntityType::Point) {
        return false;
    }
    switch (entity->type()) {
        case EntityType::Line:
            return trimLineAt(sketch, entityId, pos);
        case EntityType::Arc:
            return trimArcAt(sketch, entityId, pos);
        case EntityType::Circle:
            return trimCircleAt(sketch, entityId, pos);
        case EntityType::Ellipse:
            return sketch.removeEntity(entityId);
        default:
            return false;
    }
}

} // namespace

TrimTool::TrimTool() {
    // Trim tool is always in Idle state, ready to click
    state_ = State::Idle;
}

void TrimTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton || !sketch_) {
        return;
    }

    entityDeleted_ = false;

    // Find and delete entity at click position
    EntityID entityId = findEntityAtPosition(pos);

    if (!entityId.empty()) {
        if (trimEntityAtPosition(*sketch_, entityId, pos)) {
            entityDeleted_ = true;
            hoverEntityId_.clear();
        }
    }
}

void TrimTool::onMouseMove(const Vec2d& pos) {
    // Update hover entity for visual feedback
    hoverEntityId_ = findEntityAtPosition(pos);
}

void TrimTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Nothing on release
}

void TrimTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void TrimTool::cancel() {
    hoverEntityId_.clear();
    entityDeleted_ = false;
}

void TrimTool::render(SketchRenderer& renderer) {
    // Highlight the entity that will be deleted on click
    if (!hoverEntityId_.empty()) {
        renderer.setHoverEntity(hoverEntityId_);
    } else {
        renderer.setHoverEntity({});
    }
    renderer.clearPreview();
    renderer.clearPreviewDimensions();
}

EntityID TrimTool::findEntityAtPosition(const Vec2d& pos) const {
    if (!sketch_) {
        return {};
    }

    gp_Pnt2d testPoint(pos.x, pos.y);

    // Search through entities (skip points, we only trim lines/arcs/circles)
    for (const auto& entity : sketch_->getAllEntities()) {
        if (entity->type() == EntityType::Point) {
            continue;
        }

        // For lines, check directly
        if (entity->type() == EntityType::Line) {
            auto* line = static_cast<const SketchLine*>(entity.get());
            auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
            auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());

            if (startPt && endPt) {
                gp_Pnt2d p1 = startPt->position();
                gp_Pnt2d p2 = endPt->position();

                // Point-to-line-segment distance
                double dx = p2.X() - p1.X();
                double dy = p2.Y() - p1.Y();
                double lenSq = dx * dx + dy * dy;

                if (lenSq > 1e-10) {
                    double t = ((testPoint.X() - p1.X()) * dx +
                                (testPoint.Y() - p1.Y()) * dy) / lenSq;
                    t = std::max(0.0, std::min(1.0, t));

                    double closestX = p1.X() + t * dx;
                    double closestY = p1.Y() + t * dy;
                    double dist = std::sqrt((testPoint.X() - closestX) * (testPoint.X() - closestX) +
                                            (testPoint.Y() - closestY) * (testPoint.Y() - closestY));

                    if (dist < kPickTolerance) {
                        return entity->id();
                    }
                }
            }
        }
        // For arcs and circles, use isNear if center point is available
        else if (entity->type() == EntityType::Arc) {
            auto* arc = static_cast<const SketchArc*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(arc->centerPointId());
            if (centerPt) {
                if (arc->isNearWithCenter(testPoint, centerPt->position(), kPickTolerance)) {
                    return entity->id();
                }
            }
        }
        else if (entity->type() == EntityType::Circle) {
            auto* circle = static_cast<const SketchCircle*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(circle->centerPointId());
            if (centerPt) {
                if (circle->isNearWithCenter(testPoint, centerPt->position(), kPickTolerance)) {
                    return entity->id();
                }
            }
        }
        else if (entity->type() == EntityType::Ellipse) {
            auto* ellipse = static_cast<const SketchEllipse*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(ellipse->centerPointId());
            if (centerPt) {
                if (ellipse->isNearWithCenter(testPoint, centerPt->position(), kPickTolerance)) {
                    return entity->id();
                }
            }
        }
    }

    return {};
}

} // namespace onecad::core::sketch::tools
