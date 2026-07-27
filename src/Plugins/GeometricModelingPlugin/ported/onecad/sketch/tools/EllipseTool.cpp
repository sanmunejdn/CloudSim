#include "EllipseTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../SketchEllipse.h"

#include <cmath>
#include <format>
#include <numbers>

namespace onecad::core::sketch::tools {

EllipseTool::EllipseTool() = default;

std::optional<Vec2d> EllipseTool::getReferencePoint() const {
    if (state_ == State::FirstClick || state_ == State::Drawing) {
        return centerPoint_;
    }
    return std::nullopt;
}

std::vector<Vec2d> EllipseTool::getCommittedAnchorPoints() const {
    if (state_ == State::FirstClick || state_ == State::Drawing) {
        return {centerPoint_};
    }
    return {};
}

void EllipseTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton) {
        return;
    }

    ellipseCreated_ = false;

    if (state_ == State::Idle) {
        // First click - record center
        centerPoint_ = pos;
        currentPoint_ = pos;
        majorRadius_ = 0.0;
        minorRadius_ = 0.0;
        rotation_ = 0.0;
        centerPointId_.clear();
        if (snapResult_.snapped && !snapResult_.pointId.empty()) {
            centerPointId_ = snapResult_.pointId;
        }
        state_ = State::FirstClick;
    } else if (state_ == State::FirstClick) {
        // Second click - set major axis endpoint
        double dx = pos.x - centerPoint_.x;
        double dy = pos.y - centerPoint_.y;
        majorRadius_ = std::sqrt(dx * dx + dy * dy);

        if (majorRadius_ < constants::MIN_GEOMETRY_SIZE) {
            return;  // Too small
        }

        rotation_ = std::atan2(dy, dx);
        minorRadius_ = majorRadius_ * 0.5;  // Initial preview
        state_ = State::Drawing;
    } else if (state_ == State::Drawing) {
        // Third click - set minor radius and create ellipse
        if (!sketch_) {
            return;
        }

        // Calculate minor radius from perpendicular distance to major axis
        currentPoint_ = pos;
        double dx = currentPoint_.x - centerPoint_.x;
        double dy = currentPoint_.y - centerPoint_.y;

        // Project onto perpendicular of major axis
        double cosR = std::cos(rotation_);
        double sinR = std::sin(rotation_);

        // Local coords: major axis is X, minor axis is Y
        double localY = -dx * sinR + dy * cosR;
        minorRadius_ = std::abs(localY);

        if (minorRadius_ < constants::MIN_GEOMETRY_SIZE) {
            minorRadius_ = constants::MIN_GEOMETRY_SIZE;
        }

        // Ensure major >= minor
        if (minorRadius_ > majorRadius_) {
            std::swap(majorRadius_, minorRadius_);
            rotation_ += std::numbers::pi / 2.0;
            // Normalize rotation to [-π, π]
            constexpr double PI = std::numbers::pi;
            if (rotation_ > PI) {
                rotation_ -= 2.0 * PI;
            } else if (rotation_ <= -PI) {
                rotation_ += 2.0 * PI;
            }
        }

        // Create ellipse
        EntityID ellipseId;
        if (!centerPointId_.empty()) {
            ellipseId = sketch_->addEllipse(centerPointId_, majorRadius_,
                                             minorRadius_, rotation_);
        } else {
            EntityID centerId = sketch_->addPoint(centerPoint_.x, centerPoint_.y);
            if (!centerId.empty()) {
                ellipseId = sketch_->addEllipse(centerId, majorRadius_,
                                                 minorRadius_, rotation_);
            }
        }

        if (!ellipseId.empty()) {
            ellipseCreated_ = true;
        }

        // Reset to idle
        state_ = State::Idle;
        majorRadius_ = 0.0;
        minorRadius_ = 0.0;
        rotation_ = 0.0;
        centerPointId_.clear();
    }
}

void EllipseTool::onMouseMove(const Vec2d& pos) {
    currentPoint_ = pos;

    if (state_ == State::FirstClick) {
        // Preview major axis length
        double dx = pos.x - centerPoint_.x;
        double dy = pos.y - centerPoint_.y;
        majorRadius_ = std::sqrt(dx * dx + dy * dy);
        rotation_ = std::atan2(dy, dx);
        minorRadius_ = majorRadius_;  // Circle preview until second point
    } else if (state_ == State::Drawing) {
        // Calculate minor radius from perpendicular distance
        double dx = pos.x - centerPoint_.x;
        double dy = pos.y - centerPoint_.y;

        double cosR = std::cos(rotation_);
        double sinR = std::sin(rotation_);
        double localY = -dx * sinR + dy * cosR;
        minorRadius_ = std::abs(localY);

        if (minorRadius_ < constants::MIN_GEOMETRY_SIZE) {
            minorRadius_ = constants::MIN_GEOMETRY_SIZE;
        }

        // Apply same swap logic as final creation for consistent preview
        double previewMajor = majorRadius_;
        double previewMinor = minorRadius_;
        double previewRot = rotation_;
        if (previewMinor > previewMajor) {
            std::swap(previewMajor, previewMinor);
            previewRot += std::numbers::pi / 2.0;
            // Normalize rotation
            constexpr double PI = std::numbers::pi;
            if (previewRot > PI) {
                previewRot -= 2.0 * PI;
            } else if (previewRot <= -PI) {
                previewRot += 2.0 * PI;
            }
        }
        // Update members to match what will be created
        majorRadius_ = previewMajor;
        minorRadius_ = previewMinor;
        rotation_ = previewRot;
    }
}

void EllipseTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Ellipse tool uses click-click-click
}

void EllipseTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void EllipseTool::cancel() {
    state_ = State::Idle;
    majorRadius_ = 0.0;
    minorRadius_ = 0.0;
    rotation_ = 0.0;
    ellipseCreated_ = false;
    centerPointId_.clear();
}

void EllipseTool::render(SketchRenderer& renderer) {
    if (state_ == State::FirstClick && majorRadius_ > constants::MIN_GEOMETRY_SIZE) {
        // Show circle preview (major = minor until second click)
        renderer.setPreviewEllipse(centerPoint_, majorRadius_, majorRadius_, rotation_);

        std::string label = std::format("a: {:.2f}", majorRadius_);

        Vec2d labelPos = {
            centerPoint_.x + majorRadius_ * 0.5 * std::cos(rotation_),
            centerPoint_.y + majorRadius_ * 0.5 * std::sin(rotation_)
        };
        renderer.setPreviewDimensions({{labelPos, label}});
    } else if (state_ == State::Drawing && majorRadius_ > constants::MIN_GEOMETRY_SIZE) {
        // Show ellipse preview
        renderer.setPreviewEllipse(centerPoint_, majorRadius_, minorRadius_, rotation_);

        std::string label1 = std::format("a: {:.2f}", majorRadius_);
        std::string label2 = std::format("b: {:.2f}", minorRadius_);

        Vec2d labelPos1 = {
            centerPoint_.x + majorRadius_ * 0.5 * std::cos(rotation_),
            centerPoint_.y + majorRadius_ * 0.5 * std::sin(rotation_)
        };
        Vec2d labelPos2 = {
            centerPoint_.x + minorRadius_ * 0.5 * std::cos(rotation_ + M_PI / 2.0),
            centerPoint_.y + minorRadius_ * 0.5 * std::sin(rotation_ + M_PI / 2.0)
        };
        renderer.setPreviewDimensions({
            {labelPos1, label1},
            {labelPos2, label2}
        });
    } else {
        renderer.clearPreview();
        renderer.clearPreviewDimensions();
    }
}

} // namespace onecad::core::sketch::tools
