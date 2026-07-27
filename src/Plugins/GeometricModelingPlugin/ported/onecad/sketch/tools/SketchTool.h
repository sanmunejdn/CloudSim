/**
 * @file SketchTool.h
 * @brief Base class for interactive sketch drawing tools
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOL_H

#include "../SketchTypes.h"
#include "../SnapManager.h"
#include "../AutoConstrainer.h"

#include <Qt>
#include <string>
#include <vector>
#include <optional>

namespace onecad::core::sketch {

class Sketch;
class SketchRenderer;

namespace tools {

struct PreviewDimensionApplyResult {
    bool applied = false;
    std::string errorMessage;
};

/**
 * @brief Abstract base class for sketch drawing tools
 *
 * Tools handle mouse/keyboard input and create geometry in the sketch.
 * Each tool implements a state machine for multi-click operations.
 */
class SketchTool {
public:
    enum class State {
        Idle,       // Waiting for first input
        FirstClick, // First point recorded, waiting for second
        Drawing     // Actively drawing (for continuous tools)
    };

    virtual ~SketchTool() = default;

    /**
     * @brief Handle mouse press event
     * @param pos Position in sketch coordinates
     * @param button Which mouse button was pressed
     */
    virtual void onMousePress(const Vec2d& pos, Qt::MouseButton button) = 0;

    /**
     * @brief Handle mouse move event
     * @param pos Position in sketch coordinates
     */
    virtual void onMouseMove(const Vec2d& pos) = 0;

    /**
     * @brief Handle mouse release event
     * @param pos Position in sketch coordinates
     * @param button Which mouse button was released
     */
    virtual void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) = 0;

    /**
     * @brief Handle key press event
     * @param key The key that was pressed
     */
    virtual void onKeyPress(Qt::Key key) = 0;

    /**
     * @brief Cancel current operation (typically called on ESC)
     *
     * Should clear any preview and return to Idle state.
     */
    virtual void cancel() = 0;

    /**
     * @brief Render preview geometry
     * @param renderer The sketch renderer to draw preview on
     */
    virtual void render(SketchRenderer& renderer) = 0;

    /**
     * @brief Get tool name for UI display
     */
    virtual std::string name() const = 0;

    /**
     * @brief Apply value to an editable draft preview dimension field.
     * @param id Stable preview dimension id (tool-defined)
     * @param value Parsed numeric value from inline editor
     * @return Result with success flag and optional error message
     */
    virtual PreviewDimensionApplyResult applyPreviewDimensionValue(const std::string& /*id*/,
                                                                   double /*value*/) {
        return {};
    }

    /**
     * @brief Get reference point for snapping/inference (e.g. start point of line)
     */
    virtual std::optional<Vec2d> getReferencePoint() const { return std::nullopt; }

    /**
     * @brief Get committed anchor points that should remain visible during preview.
     */
    virtual std::vector<Vec2d> getCommittedAnchorPoints() const { return {}; }

    /**
     * @brief Set the sketch this tool operates on
     */
    void setSketch(Sketch* sketch) { sketch_ = sketch; }

    /**
     * @brief Get current tool state
     */
    State state() const { return state_; }

    /**
     * @brief Check if tool is currently active (not idle)
     */
    bool isActive() const { return state_ != State::Idle; }

    // ========== Snap & Auto-Constraint Info ==========

    /**
     * @brief Set current snap result (called by ToolManager before onMouseMove)
     */
    void setSnapResult(const SnapResult& result) { snapResult_ = result; }
    const SnapResult& snapResult() const { return snapResult_; }

    /**
     * @brief Set inferred constraints (called by ToolManager)
     */
    void setInferredConstraints(const std::vector<InferredConstraint>& constraints) {
        inferredConstraints_ = constraints;
    }
    const std::vector<InferredConstraint>& inferredConstraints() const {
        return inferredConstraints_;
    }

    /**
     * @brief Get snapped position (snap point if snapped, raw otherwise)
     * @param rawPos Original cursor position
     */
    Vec2d getSnappedPos(const Vec2d& rawPos) const {
        return snapResult_.snapped ? snapResult_.position : rawPos;
    }

    /**
     * @brief Set auto-constrainer (called by ToolManager)
     */
    void setAutoConstrainer(AutoConstrainer* constrainer) { autoConstrainer_ = constrainer; }

    /**
     * @brief Set snap manager (called by ToolManager)
     */
    void setSnapManager(SnapManager* manager) { snapManager_ = manager; }

protected:
    Sketch* sketch_ = nullptr;
    State state_ = State::Idle;
    SnapResult snapResult_;
    std::vector<InferredConstraint> inferredConstraints_;
    AutoConstrainer* autoConstrainer_ = nullptr;
    SnapManager* snapManager_ = nullptr;
};

} // namespace tools
} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOL_H
