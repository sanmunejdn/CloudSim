#include "RectangleTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../AutoConstrainer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>

namespace onecad::core::sketch::tools {

RectangleTool::RectangleTool() = default;

std::string RectangleTool::name() const {
    return "Rectangle";
}

std::optional<Vec2d> RectangleTool::getReferencePoint() const {
    if (state_ == State::FirstClick) {
        return corner1_;
    }
    return std::nullopt;
}

std::vector<Vec2d> RectangleTool::getCommittedAnchorPoints() const {
    if (state_ == State::FirstClick) {
        return {corner1_};
    }
    return {};
}

void RectangleTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton) {
        return;
    }

    rectangleCreated_ = false;

    if (state_ == State::Idle) {
        // First click - record first corner
        corner1_ = pos;
        corner2_ = pos;
        hasWidthLock_ = false;
        lockedWidth_ = 0.0;
        hasHeightLock_ = false;
        lockedHeight_ = 0.0;
        widthSign_ = 1.0;
        heightSign_ = 1.0;
        state_ = State::FirstClick;
    } else if (state_ == State::FirstClick) {
        // Second click - create rectangle
        if (!sketch_) {
            return;
        }

        updateSecondCornerFromDraftLocks(pos);

        // Check minimum size
        double width = std::abs(corner2_.x - corner1_.x);
        double height = std::abs(corner2_.y - corner1_.y);

        if (width < constants::MIN_GEOMETRY_SIZE || height < constants::MIN_GEOMETRY_SIZE) {
            // Too small, ignore
            return;
        }

        createRectangle(corner1_, corner2_);
        rectangleCreated_ = true;

        // Return to idle state
        state_ = State::Idle;
        hasWidthLock_ = false;
        lockedWidth_ = 0.0;
        hasHeightLock_ = false;
        lockedHeight_ = 0.0;
    }
}

void RectangleTool::onMouseMove(const Vec2d& pos) {
    if (state_ == State::FirstClick) {
        updateSecondCornerFromDraftLocks(pos);
    }
}

void RectangleTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Rectangle tool uses click-click, not drag
}

void RectangleTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void RectangleTool::cancel() {
    state_ = State::Idle;
    hasWidthLock_ = false;
    lockedWidth_ = 0.0;
    hasHeightLock_ = false;
    lockedHeight_ = 0.0;
    widthSign_ = 1.0;
    heightSign_ = 1.0;
    rectangleCreated_ = false;
}

void RectangleTool::render(SketchRenderer& renderer) {
    if (state_ == State::FirstClick) {
        // Show preview rectangle as 4 lines
        double width = std::abs(corner2_.x - corner1_.x);
        double height = std::abs(corner2_.y - corner1_.y);

        if (width > constants::MIN_GEOMETRY_SIZE || height > constants::MIN_GEOMETRY_SIZE) {
            renderer.setPreviewRectangle(corner1_, corner2_);

            std::vector<SketchRenderer::PreviewDimension> dims;
            char buffer[32];

            // Width dimension (top edge)
            if (width > constants::MIN_GEOMETRY_SIZE) {
                std::snprintf(buffer, sizeof(buffer), "%.2f", width);
                double midX = (corner1_.x + corner2_.x) * 0.5;
                double maxY = std::max(corner1_.y, corner2_.y);
                dims.push_back({{midX, maxY}, std::string(buffer), "rect_width", width, "mm"});
            }

            // Height dimension (right edge)
            if (height > constants::MIN_GEOMETRY_SIZE) {
                std::snprintf(buffer, sizeof(buffer), "%.2f", height);
                double maxX = std::max(corner1_.x, corner2_.x);
                double midY = (corner1_.y + corner2_.y) * 0.5;
                dims.push_back({{maxX, midY}, std::string(buffer), "rect_height", height, "mm"});
            }

            renderer.setPreviewDimensions(dims);
        } else {
            renderer.clearPreview();
        }
    } else {
        renderer.clearPreview();
    }
}

PreviewDimensionApplyResult RectangleTool::applyPreviewDimensionValue(const std::string& id, double value) {
    if (state_ != State::FirstClick) {
        return {false, "Set the rectangle first corner first"};
    }
    if (!std::isfinite(value)) {
        return {false, "Value must be finite"};
    }
    if (value <= constants::MIN_GEOMETRY_SIZE) {
        return {false, "Size must be greater than minimum geometry size"};
    }

    if (id == "rect_width") {
        hasWidthLock_ = true;
        lockedWidth_ = value;
        updateSecondCornerFromDraftLocks(corner2_);
        return {true, {}};
    }
    if (id == "rect_height") {
        hasHeightLock_ = true;
        lockedHeight_ = value;
        updateSecondCornerFromDraftLocks(corner2_);
        return {true, {}};
    }

    return {false, "Unknown rectangle draft parameter"};
}

void RectangleTool::updateSecondCornerFromDraftLocks(const Vec2d& cursorPos) {
    if (state_ != State::FirstClick) {
        return;
    }

    const double dx = cursorPos.x - corner1_.x;
    const double dy = cursorPos.y - corner1_.y;

    if (std::abs(dx) > constants::MIN_GEOMETRY_SIZE) {
        widthSign_ = (dx >= 0.0) ? 1.0 : -1.0;
    }
    if (std::abs(dy) > constants::MIN_GEOMETRY_SIZE) {
        heightSign_ = (dy >= 0.0) ? 1.0 : -1.0;
    }

    const double width = hasWidthLock_ ? lockedWidth_ : std::abs(dx);
    const double height = hasHeightLock_ ? lockedHeight_ : std::abs(dy);

    corner2_.x = corner1_.x + widthSign_ * width;
    corner2_.y = corner1_.y + heightSign_ * height;
}

void RectangleTool::createRectangle(const Vec2d& c1, const Vec2d& c2) {
    if (!sketch_) return;

    // Order corners (min/max)
    double minX = std::min(c1.x, c2.x);
    double maxX = std::max(c1.x, c2.x);
    double minY = std::min(c1.y, c2.y);
    double maxY = std::max(c1.y, c2.y);

    // Create 4 corner points
    EntityID p1 = sketch_->addPoint(minX, minY);  // bottom-left
    EntityID p2 = sketch_->addPoint(maxX, minY);  // bottom-right
    EntityID p3 = sketch_->addPoint(maxX, maxY);  // top-right
    EntityID p4 = sketch_->addPoint(minX, maxY);  // top-left

    if (p1.empty() || p2.empty() || p3.empty() || p4.empty()) {
        return;
    }

    // Create 4 lines connecting the points
    EntityID lineBottom = sketch_->addLine(p1, p2);  // bottom (horizontal)
    EntityID lineRight = sketch_->addLine(p2, p3);   // right (vertical)
    EntityID lineTop = sketch_->addLine(p3, p4);     // top (horizontal)
    EntityID lineLeft = sketch_->addLine(p4, p1);    // left (vertical)

    if (lineBottom.empty() || lineRight.empty() || lineTop.empty() || lineLeft.empty()) {
        return;
    }

    // Add constraints: horizontal for top/bottom, vertical for left/right
    sketch_->addHorizontal(lineBottom);
    sketch_->addHorizontal(lineTop);
    sketch_->addVertical(lineLeft);
    sketch_->addVertical(lineRight);
}

} // namespace onecad::core::sketch::tools
