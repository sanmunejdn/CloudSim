/**
 * @file MirrorTool.h
 * @brief Mirror tool for creating symmetrical geometry
 *
 * Per SPECIFICATION.md and SKETCH_IMPLEMENTATION_PLAN.md:
 * - Select mirror line first
 * - Then click entities to mirror
 * - Creates mirrored copies with symmetric constraints
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_MIRRORTOOL_H
#define ONECAD_CORE_SKETCH_TOOLS_MIRRORTOOL_H

#include "SketchTool.h"
#include <string>
#include <vector>

namespace onecad::core::sketch::tools {

/**
 * @brief Tool for mirroring sketch entities across a line
 *
 * State machine:
 * - Idle: Waiting for mirror line selection
 * - FirstClick: Mirror line selected, click entities to mirror
 * - ESC: Cancels and returns to Idle
 */
class MirrorTool : public SketchTool {
public:
    MirrorTool();
    ~MirrorTool() override = default;

    void onMousePress(const Vec2d& pos, Qt::MouseButton button) override;
    void onMouseMove(const Vec2d& pos) override;
    void onMouseRelease(const Vec2d& pos, Qt::MouseButton button) override;
    void onKeyPress(Qt::Key key) override;
    void cancel() override;
    void render(SketchRenderer& renderer) override;
    std::string name() const override { return "Mirror"; }

    /**
     * @brief Check if geometry was just created
     */
    bool wasGeometryCreated() const { return geometryCreated_; }
    void clearCreatedFlag() { geometryCreated_ = false; }

private:
    /**
     * @brief Find line entity at position
     */
    EntityID findLineAtPosition(const Vec2d& pos) const;

    /**
     * @brief Find any entity at position (excluding mirror line)
     */
    EntityID findEntityAtPosition(const Vec2d& pos) const;

    /**
     * @brief Mirror a point across the mirror line
     */
    Vec2d mirrorPoint(const Vec2d& point) const;

    /**
     * @brief Create mirrored copy of an entity
     */
    EntityID createMirroredEntity(EntityID sourceId);

    EntityID mirrorLineId_{};     // Empty when no mirror axis selected
    Vec2d mirrorLineStart_{0, 0}; // Cached mirror line endpoints
    Vec2d mirrorLineEnd_{0, 0};
    EntityID hoverEntityId_{};    // Empty when no hover target
    std::vector<EntityID> mirroredEntities_; // Track created entities
    bool geometryCreated_ = false;
};

} // namespace onecad::core::sketch::tools

#endif // ONECAD_CORE_SKETCH_TOOLS_MIRRORTOOL_H
