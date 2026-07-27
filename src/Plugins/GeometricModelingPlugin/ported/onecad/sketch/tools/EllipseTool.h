/**
 * @file EllipseTool.h
 * @brief Ellipse drawing tool (center → major radius → minor radius)
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_ELLIPSETOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_ELLIPSETOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for drawing ellipses by center, major, and minor radii
 *
 * State machine:
 * - Idle: waiting for first click (center)
 * - FirstClick: center set, defining major axis endpoint
 * - Drawing: major axis set, defining minor radius
 * - Click again: creates ellipse, returns to Idle
 * - ESC: cancels current operation, returns to Idle
 */
class EllipseTool : public SketchTool {
public:
    EllipseTool();
    ~EllipseTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override { return "Ellipse"; }
    std::optional<Vec2d> getReferencePoint() const override;
    std::vector<Vec2d> getCommittedAnchorPoints() const override;

    bool wasEllipseCreated() const { return ellipseCreated_; }
    void clearEllipseCreatedFlag() { ellipseCreated_ = false; }

private:
    Vec2d centerPoint_{0, 0};
    Vec2d currentPoint_{0, 0};
    EntityID centerPointId_{};

    double majorRadius_ = 0.0;
    double minorRadius_ = 0.0;
    double rotation_ = 0.0;

    bool ellipseCreated_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_ELLIPSETOOL_H
