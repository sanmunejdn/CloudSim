/**
 * @file TrimTool.h
 * @brief Trim tool for deleting sketch segments
 *
 * Per SPECIFICATION.md and SKETCH_IMPLEMENTATION_PLAN.md:
 * - Click segment to delete (removes portion between intersections)
 * - Multiple trim: Can click multiple segments in sequence
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_TRIMTOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_TRIMTOOL_H

#include "SketchTool.h"

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for trimming/deleting sketch entities
 *
 * Current implementation: Click to delete entire entity
 * Future: Intersection-based segment deletion
 *
 * State machine:
 * - Idle: Ready to click entities
 * - Hover preview shows which entity will be deleted
 * - Click deletes the entity
 * - ESC exits trim mode
 */
class TrimTool : public SketchTool {
public:
    TrimTool();
    ~TrimTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override { return "Trim"; }

    /**
     * @brief Check if an entity was just deleted
     */
    bool wasEntityDeleted() const { return entityDeleted_; }
    void clearDeletedFlag() { entityDeleted_ = false; }

private:
    /**
     * @brief Find entity at position using hit testing
     */
    EntityID findEntityAtPosition(const Vec2d& pos) const;

    EntityID hoverEntityId_{};  // Empty when no hover target
    bool entityDeleted_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_TRIMTOOL_H
