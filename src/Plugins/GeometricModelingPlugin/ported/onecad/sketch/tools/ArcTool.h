/**
 * @file ArcTool.h
 * @brief 3-point arc drawing tool
 *
 * Per SPECIFICATION.md and SKETCH_IMPLEMENTATION_PLAN.md:
 * - Primary mode: 3-Point Arc (start → point-on-arc → end)
 * - Auto-tangent when starting from line endpoint
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_ARCTOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_ARCTOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for drawing arcs using 3-point method
 *
 * State machine:
 * - Idle: waiting for first click (start point)
 * - FirstClick: start point set, waiting for middle point
 * - Drawing: start + middle set, waiting for end point
 * - ESC: cancels current operation, returns to Idle
 */
class ArcTool : public SketchTool {
public:
    ArcTool();
    ~ArcTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override;
    std::optional<Vec2d> getReferencePoint() const override;
    std::vector<Vec2d> getCommittedAnchorPoints() const override;

    /**
     * @brief Check if an arc was just created
     */
    bool wasArcCreated() const { return arcCreated_; }
    void clearArcCreatedFlag() { arcCreated_ = false; }

private:
    /**
     * @brief Calculate arc center and radius from 3 points
     * @return true if valid arc can be formed
     */
    bool calculateArcFromThreePoints(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                                     Vec2d& center, double& radius,
                                     double& startAngle, double& endAngle) const;

    /**
     * @brief Calculate preview arc geometry
     */
    void updatePreviewArc();

    /**
     * @brief Apply inferred constraints to the created arc
     */
    void applyInferredConstraints(EntityID arcId, EntityID centerPointId);

    Vec2d startPoint_{0, 0};
    Vec2d middlePoint_{0, 0};
    Vec2d currentPoint_{0, 0};

    // Snap info for coincident constraints
    EntityID startPointId_;
    EntityID middlePointId_;

    // Preview arc geometry
    Vec2d previewCenter_{0, 0};
    double previewRadius_ = 0.0;
    double previewStartAngle_ = 0.0;
    double previewEndAngle_ = 0.0;
    bool previewValid_ = false;

    bool arcCreated_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_ARCTOOL_H
