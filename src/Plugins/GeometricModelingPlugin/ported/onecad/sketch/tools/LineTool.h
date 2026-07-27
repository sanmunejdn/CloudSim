/**
 * @file LineTool.h
 * @brief Line drawing tool with polyline mode
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_LINETOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_LINETOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for drawing lines in polyline mode
 *
 * State machine:
 * - Idle: waiting for first click
 * - FirstClick: first point set, showing preview line
 * - Click again: creates line, continues from endpoint (polyline)
 * - ESC: cancels current operation, returns to Idle
 * - Right-click: finishes polyline, returns to Idle
 */
class LineTool : public SketchTool {
public:
    enum class RejectReason {
        None,
        TooShort,
        InvalidEndpoints,
        SameEndpoint
    };

    LineTool();
    ~LineTool() override = default;

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
     * @brief Check if a line was just created (for signal emission)
     */
    bool wasLineCreated() const { return lineCreated_; }
    void clearLineCreatedFlag() { lineCreated_ = false; }
    RejectReason lastRejectReason() const { return lastRejectReason_; }

private:
    void updateCurrentPointFromDraftLocks(const Vec2d& cursorPos);
    static double normalizeAngleDegrees(double angleDegrees);

    /**
     * @brief Apply inferred constraints to sketch
     */
    void applyInferredConstraints(const std::vector<InferredConstraint>& constraints,
                                   EntityID lineId);

    /**
     * @brief Update inferred constraints for preview
     */
    void updateInferredConstraints();

    Vec2d startPoint_{0, 0};
    Vec2d currentPoint_{0, 0};
    EntityID startPointId_;       // Existing point for line start (if snapped)
    EntityID lastPointId_;         // For coincident constraint on polyline continuation
    EntityID lastCreatedLineId_;   // For polyline perpendicular inference
    bool hasLengthLock_ = false;
    double lockedLength_ = 0.0;
    bool hasAngleLock_ = false;
    double lockedAngleDeg_ = 0.0;
    Vec2d fallbackDirection_{1.0, 0.0};
    bool lineCreated_ = false;
    RejectReason lastRejectReason_ = RejectReason::None;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_LINETOOL_H
