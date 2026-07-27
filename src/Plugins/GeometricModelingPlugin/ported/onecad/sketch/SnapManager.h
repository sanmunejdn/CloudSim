/**
 * @file SnapManager.h
 * @brief Snap system for precision drawing in sketch mode
 *
 * Per SPECIFICATION.md §5.14:
 * - 2mm snap radius (constant in sketch coordinates)
 * - Priority order: Vertex > Endpoint > Midpoint > Center > Quadrant > Intersection > OnCurve > Grid
 * - Visual feedback via cursor change to snap icon
 */
#ifndef ONECAD_CORE_SKETCH_SNAP_MANAGER_H
#define ONECAD_CORE_SKETCH_SNAP_MANAGER_H

#include "SketchTypes.h"
#include "SpatialHashGrid.h"
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace onecad::core::sketch {

class Sketch;
class SketchEntity;
class SketchPoint;
class SketchLine;
class SketchArc;
class SketchCircle;

/**
 * @brief Snap type enumeration with priority ordering
 *
 * Lower enum value = higher priority
 */
enum class SnapType {
    None = 0,
    Vertex,       // Snap to existing point (highest priority)
    Endpoint,     // Snap to line/arc endpoint
    Midpoint,     // Snap to line midpoint
    Center,       // Snap to arc/circle center
    Quadrant,     // Snap to circle quadrant points (0, 90, 180, 270 deg)
    Intersection, // Snap to intersection of two entities
    OnCurve,      // Snap to nearest point on curve
    Grid,         // Snap to grid (lowest priority)
    Perpendicular,// Perpendicular inference
    Tangent,      // Tangent inference
    Horizontal,   // Horizontal alignment inference
    Vertical,     // Vertical alignment inference
    SketchGuide,  // Guide lines/extensions
    ActiveLayer3D // 3D Geometry (Vertices/Edges)
};

/**
 * @brief Result of a snap operation
 */
struct SnapResult {
    static constexpr double kOverlapEps = 1e-6;

    enum class GridCandidateKind {
        None = 0,
        Crossing,
        AxisX,
        AxisY
    };

    enum class GuideIntersectionKind {
        None = 0,
        PureGuides,
        MixedWithGrid
    };

    bool snapped = false;
    SnapType type = SnapType::None;
    Vec2d position{0.0, 0.0};
    EntityID entityId;          // Entity snapped to (if any)
    EntityID secondEntityId;    // Second entity (for intersections)
    EntityID pointId;           // Existing point entity (if snap maps to a point)
    double distance = 0.0;      // Distance from cursor to snap point
    Vec2d guideOrigin{0.0, 0.0};  // Reference point for guide line rendering
    bool hasGuide = false;        // Whether to render a guide line
    std::string hintText;         // Snap type label for rendering ("H", "V", etc.)
    GridCandidateKind gridKind = GridCandidateKind::None;
    GuideIntersectionKind guideIntersectionKind = GuideIntersectionKind::None;

    /**
     * @brief Comparison for deterministic overlap arbitration and priority sorting.
     *
     * Overlap Arbitration Contract:
     * 1) Co-location epsilon: two candidates are co-located when both coordinates differ by
     *    <= kOverlapEps (1e-6, sketch coordinates).
     * 2) Co-located ranking: when candidates are co-located, rank by SnapType priority
     *    (lower enum value wins): Vertex > Endpoint > Midpoint > Center > Quadrant >
     *    Intersection > OnCurve > Grid.
     * 3) Same-type co-located tie-break: when SnapType also matches, resolve ties in this exact
     *    order: pointId < entityId < secondEntityId < position.x < position.y <
     *    hasGuide (false wins) < guideOrigin < gridKind < guideIntersectionKind < hintText.
     * 4) Determinism guarantee: for identical input (cursor position, sketch state, exclude set),
     *    findBestSnap() returns the same winner on every call, with no dependency on insertion
     *    order or hash randomness.
     *
     * Current comparator implementation order:
     * type < distance < pointId < entityId < secondEntityId < position <
     * hasGuide(false first) < guideOrigin < gridKind < guideIntersectionKind < hintText.
     */
    bool operator<(const SnapResult& other) const {
        constexpr double kDistanceEps = 1e-9;
        constexpr double kCoordEps = 1e-9;

        if (type != other.type) {
            return static_cast<int>(type) < static_cast<int>(other.type);
        }

        if (std::abs(distance - other.distance) > kDistanceEps) {
            return distance < other.distance;
        }

        if (pointId != other.pointId) {
            return pointId < other.pointId;
        }
        if (entityId != other.entityId) {
            return entityId < other.entityId;
        }
        if (secondEntityId != other.secondEntityId) {
            return secondEntityId < other.secondEntityId;
        }

        if (std::abs(position.x - other.position.x) > kCoordEps) {
            return position.x < other.position.x;
        }
        if (std::abs(position.y - other.position.y) > kCoordEps) {
            return position.y < other.position.y;
        }

        if (hasGuide != other.hasGuide) {
            return !hasGuide && other.hasGuide;
        }
        if (std::abs(guideOrigin.x - other.guideOrigin.x) > kCoordEps) {
            return guideOrigin.x < other.guideOrigin.x;
        }
        if (std::abs(guideOrigin.y - other.guideOrigin.y) > kCoordEps) {
            return guideOrigin.y < other.guideOrigin.y;
        }
        if (gridKind != other.gridKind) {
            return static_cast<int>(gridKind) < static_cast<int>(other.gridKind);
        }
        if (guideIntersectionKind != other.guideIntersectionKind) {
            return static_cast<int>(guideIntersectionKind) <
                   static_cast<int>(other.guideIntersectionKind);
        }

        return hintText < other.hintText;
    }
};

/**
 * @brief Snap manager for precision drawing
 *
 * Provides snap-to-geometry functionality during sketch drawing.
 * Implements priority-based snap selection with configurable snap types.
 */
class SnapManager {
public:
    SnapManager();
    ~SnapManager() = default;

    // Non-copyable, movable
    SnapManager(const SnapManager&) = delete;
    SnapManager& operator=(const SnapManager&) = delete;
    SnapManager(SnapManager&&) = default;
    SnapManager& operator=(SnapManager&&) = default;

    /**
     * @brief Find best snap point near cursor
     * @param cursorPos Cursor position in sketch coordinates
     * @param sketch Sketch to snap to
     * @param excludeEntities Entities to exclude from snapping (e.g., entity being drawn)
     * @return Best snap result, or result with snapped=false if no snap found
     */
    SnapResult findBestSnap(const Vec2d& cursorPos,
                            const Sketch& sketch,
                            const std::unordered_set<EntityID>& excludeEntities = {},
                            std::optional<Vec2d> referencePoint = std::nullopt) const;

    /**
     * @brief Set external 3D geometry for snapping
     * @param points 3D points projected to sketch plane
     * @param edges 3D edges projected to sketch plane lines/curves
     */
    void setExternalGeometry(const std::vector<Vec2d>& points,
                             const std::vector<std::pair<Vec2d, Vec2d>>& lines);

    /**
     * @brief Find all snap points near cursor (for UI feedback)
     */
    std::vector<SnapResult> findAllSnaps(const Vec2d& cursorPos,
                                          const Sketch& sketch,
                                          const std::unordered_set<EntityID>& excludeEntities = {},
                                          std::optional<Vec2d> referencePoint = std::nullopt) const;

    /**
     * @brief Select deterministic best snap from a prepared candidate list.
     */
    SnapResult selectBestSnapFromCandidates(
        const Vec2d& cursorPos,
        const Sketch& sketch,
        const std::vector<SnapResult>& candidates) const;

    /**
     * @brief Last winner-arbitration grid conflict status.
     */
    bool lastGridConflictDetected() const { return lastGridConflictDetected_; }

    // ========== Configuration ==========

    /**
     * @brief Set snap radius in mm (default: 2.0mm per spec)
     */
    void setSnapRadius(double radiusMM) { snapRadius_ = radiusMM; }
    double getSnapRadius() const { return snapRadius_; }

    /**
     * @brief Enable/disable specific snap type
     */
    void setSnapEnabled(SnapType type, bool enabled);
    bool isSnapEnabled(SnapType type) const;

    /**
     * @brief Enable/disable all snap types at once
     */
    void setAllSnapsEnabled(bool enabled);

    /**
     * @brief Enable/disable grid snapping
     */
    void setGridSnapEnabled(bool enabled);
    bool isGridSnapEnabled() const { return gridSnapEnabled_; }

    /**
     * @brief Set grid size for grid snapping
     */
    void setGridSize(double sizeMM) { gridSize_ = sizeMM; }
    double getGridSize() const { return gridSize_; }

    /**
     * @brief Master enable/disable for all snapping
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    void setShowGuidePoints(bool show) { showGuidePoints_ = show; }
    bool showGuidePoints() const { return showGuidePoints_; }

    void setShowSnappingHints(bool show) { showSnappingHints_ = show; }
    bool showSnappingHints() const { return showSnappingHints_; }

    void setSpatialHashEnabled(bool enabled) { spatialHashEnabled_ = enabled; }
    bool isSpatialHashEnabled() const { return spatialHashEnabled_; }

    /**
     * @brief Find intersections between two entities (public for IntersectionManager)
     */
    std::vector<Vec2d> findEntityIntersections(const SketchEntity* e1,
                                                const SketchEntity* e2,
                                                const Sketch& sketch) const;

    // ========== Ambiguity Hooks (Future Tab-cycle UX) ==========

    /**
     * @brief Check if multiple co-located candidates exist at last query position.
     * Currently unused by production code; hook for future Tab-cycle UX.
     */
    bool hasAmbiguity() const;

    /**
     * @brief Get number of co-located candidates.
     * Currently unused by production code; hook for future Tab-cycle UX.
     */
    size_t ambiguityCandidateCount() const;

    /**
     * @brief Cycle to next ambiguity candidate.
     * Currently unused by production code; hook for future Tab-cycle UX.
     */
    void cycleAmbiguity() const;

    /**
     * @brief Reset ambiguity state.
     * Currently unused by production code; hook for future Tab-cycle UX.
     */
    void clearAmbiguity() const;

    /**
     * @brief Clear grid-only snapping state (hysteresis + axis memory)
     */
    void resetGridSnapState() const;

private:
    struct AmbiguityState {
        std::vector<SnapResult> candidates;
        size_t selectedIndex = 0;
        bool active = false;
    };

    double snapRadius_ = constants::SNAP_RADIUS_MM;  // 2.0mm default
    double gridSize_ = 1.0;  // 1mm grid default
    bool gridSnapEnabled_ = true;
    bool enabled_ = true;
    std::unordered_map<SnapType, bool> snapTypeEnabled_;

    bool showGuidePoints_ = true;
    bool showSnappingHints_ = true;
    bool spatialHashEnabled_ = true;

    // External geometry (projected 3D entities)
    std::vector<Vec2d> extPoints_;
    std::vector<std::pair<Vec2d, Vec2d>> extLines_;

    mutable SpatialHashGrid spatialHash_;
    mutable AmbiguityState ambiguityState_;
    mutable bool lastGridConflictDetected_ = false;

    struct GridStickyState {
        SnapResult::GridCandidateKind kind = SnapResult::GridCandidateKind::None;
        double gridX = 0.0;
        double gridY = 0.0;
    };

    mutable std::optional<GridStickyState> gridStickyState_;
    mutable SnapResult::GridCandidateKind axisTieMemory_ = SnapResult::GridCandidateKind::None;

    static constexpr double kGridAxisTieEpsilonMM = 0.05;
    static constexpr double kGridReleaseRadiusMultiplier = 1.35;

    void rebuildSpatialHash(const Sketch& sketch) const;
    bool shouldConsiderEntity(const EntityID& entityId,
                              const std::unordered_set<EntityID>* candidateSet) const;
    std::optional<SnapResult> pickBestGridCandidate(
        const Vec2d& cursorPos,
        const std::vector<SnapResult>& snaps) const;
    std::optional<SnapResult> buildStickyGridCandidate(const Vec2d& cursorPos) const;
    bool isGridConflict(const std::vector<SnapResult>& snaps,
                        const SnapResult& selectedGrid) const;
    static bool isPointType(SnapType type);
    static bool isNonGridGeometryType(const SnapResult& snap);
    static bool isMixedGridGuideIntersection(const SnapResult& snap);
    SnapResult selectBestSnapCandidate(
        const Vec2d& cursorPos,
        const Sketch& sketch,
        const std::vector<SnapResult>& snaps) const;
    static void applyGridStickyStateFromSelection(
        const SnapResult& selected,
        std::optional<GridStickyState>& stickyState,
        SnapResult::GridCandidateKind& axisTieMemory);

    // ========== Individual Snap Type Finders ==========

    /**
     * @brief Find snaps to existing sketch points
     */
    void findVertexSnaps(const Vec2d& cursorPos,
                         const Sketch& sketch,
                         const std::unordered_set<EntityID>& excludeEntities,
                         const std::unordered_set<EntityID>* candidateSet,
                         double radiusSq,
                         std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to line endpoints
     */
    void findEndpointSnaps(const Vec2d& cursorPos,
                           const Sketch& sketch,
                           const std::unordered_set<EntityID>& excludeEntities,
                           const std::unordered_set<EntityID>* candidateSet,
                           double radiusSq,
                           std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to line midpoints
     */
    void findMidpointSnaps(const Vec2d& cursorPos,
                           const Sketch& sketch,
                           const std::unordered_set<EntityID>& excludeEntities,
                           const std::unordered_set<EntityID>* candidateSet,
                           double radiusSq,
                           std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to arc/circle centers
     */
    void findCenterSnaps(const Vec2d& cursorPos,
                         const Sketch& sketch,
                         const std::unordered_set<EntityID>& excludeEntities,
                         const std::unordered_set<EntityID>* candidateSet,
                         double radiusSq,
                         std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to circle quadrant points (0, 90, 180, 270 degrees)
     */
    void findQuadrantSnaps(const Vec2d& cursorPos,
                           const Sketch& sketch,
                           const std::unordered_set<EntityID>& excludeEntities,
                           const std::unordered_set<EntityID>* candidateSet,
                           double radiusSq,
                           std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to entity intersections
     */
    void findIntersectionSnaps(const Vec2d& cursorPos,
                               const Sketch& sketch,
                               const std::unordered_set<EntityID>& excludeEntities,
                               const std::unordered_set<EntityID>* candidateSet,
                               double radiusSq,
                               std::vector<SnapResult>& results) const;

    /**
     * @brief Find snaps to nearest point on curves
     */
    void findOnCurveSnaps(const Vec2d& cursorPos,
                          const Sketch& sketch,
                          const std::unordered_set<EntityID>& excludeEntities,
                          const std::unordered_set<EntityID>* candidateSet,
                          double radiusSq,
                          std::vector<SnapResult>& results) const;

    /**
     * @brief Find snap to grid
     */
    void findGridSnaps(const Vec2d& cursorPos,
                       std::vector<SnapResult>& results) const;

    /**
     * @brief Find perpendicular inference snaps
     */
    void findPerpendicularSnaps(const Vec2d& cursorPos,
                                const Sketch& sketch,
                                const std::unordered_set<EntityID>& excludeEntities,
                                double radiusSq,
                                std::vector<SnapResult>& results) const;

    /**
     * @brief Find tangent inference snaps
     */
    void findTangentSnaps(const Vec2d& cursorPos,
                          const Sketch& sketch,
                          const std::unordered_set<EntityID>& excludeEntities,
                          double radiusSq,
                          std::vector<SnapResult>& results) const;

    /**
     * @brief Find horizontal alignment inference snaps
     */
    void findHorizontalSnaps(const Vec2d& cursorPos,
                             const Sketch& sketch,
                             const std::unordered_set<EntityID>& excludeEntities,
                             std::vector<SnapResult>& results) const;

    /**
     * @brief Find vertical alignment inference snaps
     */
    void findVerticalSnaps(const Vec2d& cursorPos,
                           const Sketch& sketch,
                           const std::unordered_set<EntityID>& excludeEntities,
                           std::vector<SnapResult>& results) const;

    /**
     * @brief Find extension/guide inference snaps
     */
    void findGuideSnaps(const Vec2d& cursorPos,
                        const Sketch& sketch,
                        const std::unordered_set<EntityID>& excludeEntities,
                        double radiusSq,
                        std::vector<SnapResult>& results) const;

    /**
     * @brief Find angular (polar tracking) guide snaps at 15deg increments
     */
    void findAngularSnap(const Vec2d& cursorPos,
                         const Vec2d& referencePoint,
                         double radiusSq,
                         std::vector<SnapResult>& results) const;

    /**
     * @brief Find snap to external geometry (3D)
     */
    void findExternalSnaps(const Vec2d& cursorPos,
                           double radiusSq,
                           std::vector<SnapResult>& results) const;

    // ========== Geometry Helpers ==========

    /**
     * @brief Calculate distance squared from point to point
     */
    static double distanceSquared(const Vec2d& a, const Vec2d& b);

    /**
     * @brief Find nearest point on line segment to given point
     */
    static Vec2d nearestPointOnLine(const Vec2d& point,
                                     const Vec2d& lineStart,
                                     const Vec2d& lineEnd);

    /**
     * @brief Find nearest point on circle to given point
     */
    static Vec2d nearestPointOnCircle(const Vec2d& point,
                                       const Vec2d& center,
                                       double radius);

    /**
     * @brief Find line-line intersection
     * @return Intersection point, or nullopt if lines are parallel
     */
    static std::optional<Vec2d> lineLineIntersection(const Vec2d& p1, const Vec2d& p2,
                                                      const Vec2d& p3, const Vec2d& p4);

    /**
     * @brief Find line-circle intersections
     * @return 0, 1, or 2 intersection points
     */
    static std::vector<Vec2d> lineCircleIntersection(const Vec2d& lineStart,
                                                      const Vec2d& lineEnd,
                                                      const Vec2d& center,
                                                      double radius);

    /**
     * @brief Find circle-circle intersections
     * @return 0, 1, or 2 intersection points
     */
    static std::vector<Vec2d> circleCircleIntersection(const Vec2d& center1,
                                                        double radius1,
                                                        const Vec2d& center2,
                                                        double radius2);
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_SNAP_MANAGER_H
