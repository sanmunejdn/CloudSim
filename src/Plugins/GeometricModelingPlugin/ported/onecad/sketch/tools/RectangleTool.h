/**
 * @file RectangleTool.h
 * @brief Rectangle drawing tool (creates 4 lines with constraints)
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_RECTANGLETOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_RECTANGLETOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for drawing rectangles as 4 constrained lines
 *
 * Creates 4 lines forming a rectangle with:
 * - Horizontal constraints on top/bottom edges
 * - Vertical constraints on left/right edges
 * - Coincident endpoints (implicit via shared points)
 *
 * State machine:
 * - Idle: waiting for first click (corner 1)
 * - FirstClick: corner 1 set, showing preview rectangle
 * - Click again: creates rectangle, returns to Idle
 * - ESC: cancels current operation, returns to Idle
 */
class RectangleTool : public SketchTool {
public:
    RectangleTool();
    ~RectangleTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override;
    std::optional<Vec2d> getReferencePoint() const override;
    std::vector<Vec2d> getCommittedAnchorPoints() const override;
    PreviewDimensionApplyResult applyPreviewDimensionValue(const std::string& id,
                                                           double value) override;

    /**
     * @brief Check if a rectangle was just created
     */
    bool wasRectangleCreated() const { return rectangleCreated_; }
    void clearRectangleCreatedFlag() { rectangleCreated_ = false; }

private:
    void createRectangle(const Vec2d& corner1, const Vec2d& corner2);
    void updateSecondCornerFromDraftLocks(const Vec2d& cursorPos);

    Vec2d corner1_{0, 0};
    Vec2d corner2_{0, 0};
    bool hasWidthLock_ = false;
    double lockedWidth_ = 0.0;
    bool hasHeightLock_ = false;
    double lockedHeight_ = 0.0;
    double widthSign_ = 1.0;
    double heightSign_ = 1.0;
    bool rectangleCreated_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_RECTANGLETOOL_H
