#include "MirrorTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../SketchLine.h"
#include "../SketchArc.h"
#include "../SketchCircle.h"
#include "../SketchEllipse.h"
#include "../SketchPoint.h"

#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>

namespace onecad::core::sketch::tools {

MirrorTool::MirrorTool() = default;

void MirrorTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton || !sketch_) {
        return;
    }

    geometryCreated_ = false;

    if (state_ == State::Idle) {
        // First click - select mirror line
        EntityID lineId = findLineAtPosition(pos);
        if (!lineId.empty()) {
            mirrorLineId_ = lineId;

            // Cache mirror line endpoints
            auto* line = sketch_->getEntityAs<SketchLine>(mirrorLineId_);
            if (line) {
                auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
                auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());
                if (startPt && endPt) {
                    mirrorLineStart_ = {startPt->position().X(), startPt->position().Y()};
                    mirrorLineEnd_ = {endPt->position().X(), endPt->position().Y()};
                    state_ = State::FirstClick;
                }
            }
        }
    } else if (state_ == State::FirstClick) {
        // Mirror line selected - click entities to mirror
        EntityID entityId = findEntityAtPosition(pos);
        if (!entityId.empty()) {
            EntityID mirroredId = createMirroredEntity(entityId);
            if (!mirroredId.empty()) {
                mirroredEntities_.push_back(mirroredId);
                geometryCreated_ = true;
            }
        }
    }
}

void MirrorTool::onMouseMove(const Vec2d& pos) {
    if (state_ == State::Idle) {
        // Show which line would be selected as mirror axis
        hoverEntityId_ = findLineAtPosition(pos);
    } else if (state_ == State::FirstClick) {
        // Show which entity would be mirrored
        hoverEntityId_ = findEntityAtPosition(pos);
    }
}

void MirrorTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Nothing on release
}

void MirrorTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void MirrorTool::cancel() {
    state_ = State::Idle;
    mirrorLineId_.clear();
    hoverEntityId_.clear();
    mirroredEntities_.clear();
    geometryCreated_ = false;
}

void MirrorTool::render(SketchRenderer& renderer) {
    // Highlight the entity under cursor
    renderer.setHoverEntity(hoverEntityId_);

    // If mirror line selected, highlight it as selected
    if (!mirrorLineId_.empty()) {
        renderer.setEntitySelection(mirrorLineId_, SelectionState::Selected);
    }

    renderer.clearPreview();
    renderer.clearPreviewDimensions();
}

EntityID MirrorTool::findLineAtPosition(const Vec2d& pos) const {
    if (!sketch_) {
        return {};
    }

    constexpr double PICK_TOLERANCE = 3.0;
    gp_Pnt2d testPoint(pos.x, pos.y);

    for (const auto& entity : sketch_->getAllEntities()) {
        if (entity->type() != EntityType::Line) {
            continue;
        }

        auto* line = static_cast<const SketchLine*>(entity.get());
        auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
        auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());

        if (startPt && endPt) {
            gp_Pnt2d p1 = startPt->position();
            gp_Pnt2d p2 = endPt->position();

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

                if (dist < PICK_TOLERANCE) {
                    return entity->id();
                }
            }
        }
    }

    return {};
}

EntityID MirrorTool::findEntityAtPosition(const Vec2d& pos) const {
    if (!sketch_) {
        return {};
    }

    constexpr double PICK_TOLERANCE = 3.0;
    gp_Pnt2d testPoint(pos.x, pos.y);

    for (const auto& entity : sketch_->getAllEntities()) {
        // Skip mirror line and points
        if (entity->id() == mirrorLineId_ || entity->type() == EntityType::Point) {
            continue;
        }

        if (entity->type() == EntityType::Line) {
            auto* line = static_cast<const SketchLine*>(entity.get());
            auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
            auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());

            if (startPt && endPt) {
                gp_Pnt2d p1 = startPt->position();
                gp_Pnt2d p2 = endPt->position();

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

                    if (dist < PICK_TOLERANCE) {
                        return entity->id();
                    }
                }
            }
        } else if (entity->type() == EntityType::Circle) {
            auto* circle = static_cast<const SketchCircle*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(circle->centerPointId());
            if (centerPt) {
                if (circle->isNearWithCenter(testPoint, centerPt->position(), PICK_TOLERANCE)) {
                    return entity->id();
                }
            }
        } else if (entity->type() == EntityType::Arc) {
            auto* arc = static_cast<const SketchArc*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(arc->centerPointId());
            if (centerPt) {
                if (arc->isNearWithCenter(testPoint, centerPt->position(), PICK_TOLERANCE)) {
                    return entity->id();
                }
            }
        } else if (entity->type() == EntityType::Ellipse) {
            auto* ellipse = static_cast<const SketchEllipse*>(entity.get());
            auto* centerPt = sketch_->getEntityAs<SketchPoint>(ellipse->centerPointId());
            if (centerPt) {
                if (ellipse->isNearWithCenter(testPoint, centerPt->position(), PICK_TOLERANCE)) {
                    return entity->id();
                }
            }
        }
    }

    return {};
}

Vec2d MirrorTool::mirrorPoint(const Vec2d& point) const {
    // Mirror point across line defined by mirrorLineStart_ and mirrorLineEnd_
    double dx = mirrorLineEnd_.x - mirrorLineStart_.x;
    double dy = mirrorLineEnd_.y - mirrorLineStart_.y;
    double lenSq = dx * dx + dy * dy;

    if (lenSq < 1e-10) {
        return point; // Degenerate line
    }

    // Project point onto line
    double t = ((point.x - mirrorLineStart_.x) * dx +
                (point.y - mirrorLineStart_.y) * dy) / lenSq;

    double projX = mirrorLineStart_.x + t * dx;
    double projY = mirrorLineStart_.y + t * dy;

    // Mirror = projection + (projection - point) = 2*projection - point
    return {2.0 * projX - point.x, 2.0 * projY - point.y};
}

EntityID MirrorTool::createMirroredEntity(EntityID sourceId) {
    if (!sketch_ || sourceId.empty()) {
        return {};
    }

    auto* entity = sketch_->getEntity(sourceId);
    if (!entity) {
        return {};
    }

    if (entity->type() == EntityType::Line) {
        auto* line = static_cast<const SketchLine*>(entity);
        auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
        auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());

        if (!startPt || !endPt) {
            return {};
        }

        Vec2d start{startPt->position().X(), startPt->position().Y()};
        Vec2d end{endPt->position().X(), endPt->position().Y()};

        Vec2d mirroredStart = mirrorPoint(start);
        Vec2d mirroredEnd = mirrorPoint(end);

        EntityID newStartId = sketch_->addPoint(mirroredStart.x, mirroredStart.y);
        EntityID newEndId = sketch_->addPoint(mirroredEnd.x, mirroredEnd.y);

        if (newStartId.empty() || newEndId.empty()) {
            // Cleanup any partially created points
            if (!newStartId.empty()) sketch_->removeEntity(newStartId);
            if (!newEndId.empty()) sketch_->removeEntity(newEndId);
            return {};
        }

        EntityID lineId = sketch_->addLine(newStartId, newEndId);
        if (lineId.empty()) {
            // Cleanup orphan points
            sketch_->removeEntity(newStartId);
            sketch_->removeEntity(newEndId);
        }
        return lineId;
    }
    else if (entity->type() == EntityType::Circle) {
        auto* circle = static_cast<const SketchCircle*>(entity);
        auto* centerPt = sketch_->getEntityAs<SketchPoint>(circle->centerPointId());

        if (!centerPt) {
            return {};
        }

        Vec2d center{centerPt->position().X(), centerPt->position().Y()};
        Vec2d mirroredCenter = mirrorPoint(center);

        EntityID newCenterId = sketch_->addPoint(mirroredCenter.x, mirroredCenter.y);
        if (newCenterId.empty()) {
            return {};
        }

        EntityID circleId = sketch_->addCircle(newCenterId, circle->radius());
        if (circleId.empty()) {
            // Cleanup orphan center point
            sketch_->removeEntity(newCenterId);
        }
        return circleId;
    }
    else if (entity->type() == EntityType::Arc) {
        auto* arc = static_cast<const SketchArc*>(entity);
        auto* centerPt = sketch_->getEntityAs<SketchPoint>(arc->centerPointId());

        if (!centerPt) {
            return {};
        }

        Vec2d center{centerPt->position().X(), centerPt->position().Y()};
        Vec2d mirroredCenter = mirrorPoint(center);

        // For arcs, we also need to mirror the angles
        // Mirroring reverses the direction, so start/end angles swap and negate
        double startAngle = arc->startAngle();
        double endAngle = arc->endAngle();

        // Mirror angles across the line direction
        double lineAngle = std::atan2(mirrorLineEnd_.y - mirrorLineStart_.y,
                                       mirrorLineEnd_.x - mirrorLineStart_.x);

        // Reflect angles: new_angle = 2*line_angle - old_angle
        double mirroredStartAngle = 2.0 * lineAngle - endAngle;    // Swap start/end
        double mirroredEndAngle = 2.0 * lineAngle - startAngle;

        EntityID newCenterId = sketch_->addPoint(mirroredCenter.x, mirroredCenter.y);
        if (newCenterId.empty()) {
            return {};
        }

        return sketch_->addArc(newCenterId, arc->radius(),
                               mirroredStartAngle, mirroredEndAngle);
    }
    else if (entity->type() == EntityType::Ellipse) {
        auto* ellipse = static_cast<const SketchEllipse*>(entity);
        auto* centerPt = sketch_->getEntityAs<SketchPoint>(ellipse->centerPointId());

        if (!centerPt) {
            return {};
        }

        Vec2d center{centerPt->position().X(), centerPt->position().Y()};
        Vec2d mirroredCenter = mirrorPoint(center);

        double lineAngle = std::atan2(mirrorLineEnd_.y - mirrorLineStart_.y,
                                       mirrorLineEnd_.x - mirrorLineStart_.x);
        double mirroredRotation = 2.0 * lineAngle - ellipse->rotation();

        EntityID newCenterId = sketch_->addPoint(mirroredCenter.x, mirroredCenter.y);
        if (newCenterId.empty()) {
            return {};
        }

        return sketch_->addEllipse(newCenterId, ellipse->majorRadius(),
                                   ellipse->minorRadius(), mirroredRotation);
    }

    return {};
}

} // namespace onecad::core::sketch::tools
