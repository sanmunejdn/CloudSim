/**
 * @file IntersectionManager.h
 * @brief Manages automatic intersection point materialization and entity splitting
 *
 * When new entities are created that intersect existing geometry, this manager:
 * 1. Detects all intersection points
 * 2. Creates SketchPoint entities at intersections
 * 3. Splits intersected lines/arcs into segments
 * 4. Enables segment-based trim operations (like Shapr3D)
 */
#ifndef ONECAD_CORE_SKETCH_INTERSECTION_MANAGER_H
#define ONECAD_CORE_SKETCH_INTERSECTION_MANAGER_H

#include "SketchTypes.h"
#include <vector>

namespace onecad::core::sketch {

// Forward declarations
class Sketch;
class SnapManager;
class SketchEntity;

/**
 * @brief Result of processing intersections for a single entity
 */
struct IntersectionResult {
    size_t pointsCreated = 0;
    size_t entitiesSplit = 0;
    std::vector<EntityID> newSegments;
    std::vector<Vec2d> intersectionPoints;
};

/**
 * @brief Manages automatic intersection detection and line splitting
 *
 * Used by drawing tools (LineTool, ArcTool, etc.) to automatically
 * split geometry at intersection points during creation.
 */
class IntersectionManager {
public:
    IntersectionManager();
    ~IntersectionManager();

    /**
     * @brief Process intersections for a newly created entity
     * @param newEntityId ID of newly created entity
     * @param sketch Sketch containing the entity
     * @param snapManager SnapManager for intersection detection
     * @return Result containing split statistics
     *
     * Finds all intersections between newEntityId and existing entities,
     * creates points at intersections, and splits intersected entities.
     */
    IntersectionResult processIntersections(
        EntityID newEntityId,
        Sketch& sketch,
        const SnapManager& snapManager);

    /**
     * @brief Enable/disable automatic intersection processing
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Set minimum distance between intersection points
     *
     * Intersections closer than this will be merged to single point.
     * Default: 0.01mm
     */
    void setMinimumPointSpacing(double spacing) { minPointSpacing_ = spacing; }
    double getMinimumPointSpacing() const { return minPointSpacing_; }

private:
    bool enabled_ = true;
    double minPointSpacing_ = 0.01;  // mm

    /**
     * @brief Find all intersections between two entities
     */
    std::vector<Vec2d> findIntersections(
        const SketchEntity* e1,
        const SketchEntity* e2,
        const Sketch& sketch,
        const SnapManager& snapManager) const;

    /**
     * @brief Check if point already exists at location (within tolerance)
     */
    EntityID findExistingPointAt(
        const Vec2d& pos,
        const Sketch& sketch,
        double tolerance) const;

    /**
     * @brief Merge nearby intersection points
     */
    std::vector<Vec2d> mergeNearbyPoints(
        const std::vector<Vec2d>& points,
        double tolerance) const;
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_INTERSECTION_MANAGER_H
