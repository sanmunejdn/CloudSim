/**
 * @file CircleTool.h
 * @brief Circle drawing tool (center + radius)
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_CIRCLETOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_CIRCLETOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for drawing circles by center and radius
 *
 * State machine:
 * - Idle: waiting for first click (center)
 * - FirstClick: center set, showing preview circle
 * - Click again: creates circle, returns to Idle
 * - ESC: cancels current operation, returns to Idle
 */
class CircleTool : public SketchTool {
public:
    CircleTool();
    ~CircleTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override { return "Circle"; }
    std::optional<Vec2d> getReferencePoint() const override;
    std::vector<Vec2d> getCommittedAnchorPoints() const override;
    PreviewDimensionApplyResult applyPreviewDimensionValue(const std::string& id,
                                                           double value) override;

    /**
     * @brief Check if a circle was just created
     */
    bool wasCircleCreated() const { return circleCreated_; }
    void clearCircleCreatedFlag() { circleCreated_ = false; }

private:
    void updatePreviewFromDraftRadius(const Vec2d& cursorPos);

    Vec2d centerPoint_{0, 0};
    Vec2d currentPoint_{0, 0};
    EntityID centerPointId_;
    double currentRadius_ = 0.0;
    bool hasRadiusLock_ = false;
    double lockedRadius_ = 0.0;
    Vec2d fallbackDirection_{1.0, 0.0};
    bool circleCreated_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_CIRCLETOOL_H
