#include "CircleTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../AutoConstrainer.h"
#include "../IntersectionManager.h"
#include "../SketchCircle.h"
#include "../SketchArc.h"
#include "../constraints/Constraints.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

namespace onecad::core::sketch::tools {

CircleTool::CircleTool() = default;

std::optional<Vec2d> CircleTool::getReferencePoint() const {
    if (state_ == State::FirstClick) {
        return centerPoint_;
    }
    return std::nullopt;
}

std::vector<Vec2d> CircleTool::getCommittedAnchorPoints() const {
    if (state_ == State::FirstClick) {
        return {centerPoint_};
    }
    return {};
}

void CircleTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton) {
        return;
    }

    circleCreated_ = false;

    if (state_ == State::Idle) {
        // First click - record center
        centerPoint_ = pos;
        currentPoint_ = pos;
        currentRadius_ = 0.0;
        hasRadiusLock_ = false;
        lockedRadius_ = 0.0;
        fallbackDirection_ = {1.0, 0.0};
        centerPointId_.clear();
        if (snapResult_.snapped && !snapResult_.pointId.empty()) {
            centerPointId_ = snapResult_.pointId;
        }
        state_ = State::FirstClick;
    } else if (state_ == State::FirstClick) {
        // Second click - create circle
        if (!sketch_) {
            return;
        }

        updatePreviewFromDraftRadius(pos);
        double radius = currentRadius_;

        if (radius < constants::MIN_GEOMETRY_SIZE) {
            // Too small, ignore
            return;
        }

        // Create circle
        EntityID circleId;
        if (!centerPointId_.empty()) {
            circleId = sketch_->addCircle(centerPointId_, radius);
        } else {
            circleId = sketch_->addCircle(centerPoint_.x, centerPoint_.y, radius);
        }

        if (!circleId.empty()) {
            circleCreated_ = true;

            // Process intersections - split existing entities at intersection points
            if (snapManager_) {
                IntersectionManager intersectionMgr;
                intersectionMgr.processIntersections(circleId, *sketch_, *snapManager_);
            }

            auto resolveCenterPointId = [this](const EntityID& entityId) -> EntityID {
                if (const auto* circle = sketch_->getEntityAs<SketchCircle>(entityId)) {
                    return circle->centerPointId();
                }
                if (const auto* arc = sketch_->getEntityAs<SketchArc>(entityId)) {
                    return arc->centerPointId();
                }
                return {};
            };

            // Apply inferred constraints
            if (autoConstrainer_ && autoConstrainer_->isEnabled()) {
                DrawingContext context;
                context.activeEntity = circleId;
                context.startPoint = centerPoint_;
                context.currentPoint = currentPoint_;

                auto constraints = autoConstrainer_->inferCircleConstraints(
                    centerPoint_, radius, circleId, *sketch_, context);

                // Filter and apply high-confidence constraints
                auto toApply = autoConstrainer_->filterForAutoApply(constraints);
                for (const auto& constraint : toApply) {
                    if (constraint.type == ConstraintType::Coincident) {
                        EntityID centerId = resolveCenterPointId(circleId);
                        if (!centerId.empty() && !constraint.entity1.empty() &&
                            centerId != constraint.entity1) {
                            sketch_->addCoincident(centerId, constraint.entity1);
                        }
                    } else if (constraint.type == ConstraintType::Concentric && constraint.entity2 &&
                               !constraint.entity2->empty()) {
                        EntityID centerId = resolveCenterPointId(circleId);
                        EntityID otherCenterId = resolveCenterPointId(*constraint.entity2);
                        if (!centerId.empty() && !otherCenterId.empty() &&
                            centerId != otherCenterId) {
                            sketch_->addCoincident(centerId, otherCenterId);
                        }
                    } else if (constraint.type == ConstraintType::Equal && constraint.entity2 &&
                               !constraint.entity2->empty()) {
                        sketch_->addConstraint(
                            std::make_unique<constraints::EqualConstraint>(circleId, *constraint.entity2));
                    }
                }
            }
        }

        // Return to idle state (circle tool doesn't chain like line tool)
        state_ = State::Idle;
        currentRadius_ = 0.0;
        centerPointId_.clear();
        hasRadiusLock_ = false;
        lockedRadius_ = 0.0;
        fallbackDirection_ = {1.0, 0.0};
    }
}

void CircleTool::onMouseMove(const Vec2d& pos) {
    if (state_ == State::FirstClick) {
        updatePreviewFromDraftRadius(pos);
    } else {
        currentPoint_ = pos;
    }
}

void CircleTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Circle tool uses click-click, not drag
}

void CircleTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void CircleTool::cancel() {
    state_ = State::Idle;
    currentRadius_ = 0.0;
    circleCreated_ = false;
    centerPointId_.clear();
    hasRadiusLock_ = false;
    lockedRadius_ = 0.0;
    fallbackDirection_ = {1.0, 0.0};
}

void CircleTool::render(SketchRenderer& renderer) {
    if (state_ == State::FirstClick && currentRadius_ > constants::MIN_GEOMETRY_SIZE) {
        // Show preview circle
        renderer.setPreviewCircle(centerPoint_, currentRadius_);

        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "R: %.2f", currentRadius_);
        
        // Place label at midpoint of the radius line (from center to cursor)
        Vec2d labelPos = {
            (centerPoint_.x + currentPoint_.x) * 0.5,
            (centerPoint_.y + currentPoint_.y) * 0.5
        };

        renderer.setPreviewDimensions({{labelPos,
                                        std::string(buffer),
                                        "circle_radius",
                                        currentRadius_,
                                        "mm"}});
    } else {
        renderer.clearPreview();
    }
}

PreviewDimensionApplyResult CircleTool::applyPreviewDimensionValue(const std::string& id, double value) {
    if (state_ != State::FirstClick) {
        return {false, "Set the circle center point first"};
    }
    if (id != "circle_radius") {
        return {false, "Unknown circle draft parameter"};
    }
    if (!std::isfinite(value)) {
        return {false, "Value must be finite"};
    }
    if (value <= constants::MIN_GEOMETRY_SIZE) {
        return {false, "Radius must be greater than minimum geometry size"};
    }

    hasRadiusLock_ = true;
    lockedRadius_ = value;
    updatePreviewFromDraftRadius(currentPoint_);
    return {true, {}};
}

void CircleTool::updatePreviewFromDraftRadius(const Vec2d& cursorPos) {
    const double dx = cursorPos.x - centerPoint_.x;
    const double dy = cursorPos.y - centerPoint_.y;
    const double rawRadius = std::sqrt(dx * dx + dy * dy);

    if (rawRadius > 1e-9) {
        fallbackDirection_ = {dx / rawRadius, dy / rawRadius};
    }

    currentRadius_ = hasRadiusLock_ ? lockedRadius_ : rawRadius;
    currentPoint_.x = centerPoint_.x + fallbackDirection_.x * currentRadius_;
    currentPoint_.y = centerPoint_.y + fallbackDirection_.y * currentRadius_;
}

} // namespace onecad::core::sketch::tools
