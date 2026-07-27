/**
 * @file SketchRenderer.h
 * @brief Rendering interface for sketch visualization
 *
 * Per SPECIFICATION.md §5.16, §5.18:
 * - VBO-based OpenGL rendering for performance
 * - Adaptive arc tessellation based on zoom
 * - Constraint icon atlas rendering
 * - DOF indicator visualization
 * - Viewport culling for large sketches
 *
 * IMPLEMENTATION STATUS: COMPLETE
 * Full OpenGL rendering implementation in SketchRenderer.cpp (1897 lines).
 */
#ifndef ONECAD_CORE_SKETCH_SKETCH_RENDERER_H
#define ONECAD_CORE_SKETCH_SKETCH_RENDERER_H

#include "SketchTypes.h"
#include "SnapManager.h"  // For SnapType, SnapResult
#include "AutoConstrainer.h"  // For InferredConstraint
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations for OpenGL types
class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class QMatrix4x4;

namespace onecad::core::sketch {

class Sketch;
class SketchEntity;
class SketchConstraint;

// Note: SnapType and SnapResult are now defined in SnapManager.h

/**
 * @brief Color definitions for sketch rendering
 */
struct SketchColors {
    // Entity colors
    Vec3d normalGeometry{1.0, 1.0, 1.0};       // White
    Vec3d constructionGeometry{0.5, 0.5, 0.9}; // Light blue
    Vec3d selectedGeometry{0.2, 0.6, 1.0};     // Blue
    Vec3d previewGeometry{0.7, 0.7, 0.7};      // Gray
    Vec3d errorGeometry{1.0, 0.3, 0.3};        // Red

    // Constraint colors
    Vec3d constraintIcon{0.9, 0.7, 0.2};       // Orange
    Vec3d dimensionText{0.4, 0.4, 1.0};        // Blue
    Vec3d conflictHighlight{1.0, 0.0, 0.0};    // Red

    // DOF indicator
    Vec3d fullyConstrained{0.1, 0.4, 0.9};     // Blue
    Vec3d underConstrained{1.0, 0.5, 0.0};     // Orange
    Vec3d overConstrained{1.0, 0.0, 0.0};      // Red

    // Background
    Vec3d gridMajor{0.3, 0.3, 0.3};
    Vec3d gridMinor{0.15, 0.15, 0.15};

    // Region fill
    Vec3d regionFill{0.4, 0.7, 1.0};
};

/**
 * @brief Rendering style options
 */
struct SketchRenderStyle {
    // Line widths (in pixels) – 2x thicker for visibility
    float normalLineWidth = 4.0f;
    float constructionLineWidth = 2.5f;
    float selectedLineWidth = 6.0f;
    float previewLineWidth = 2.5f;

    // Point sizes (in pixels)
    float pointSize = 8.0f;
    float selectedPointSize = 12.0f;
    float snapPointSize = 8.0f;
    float midpointPointSize = 5.0f;

    // Constraint icon size (in pixels)
    float constraintIconSize = 16.0f;

    // Dimension text
    float dimensionFontSize = 12.0f;

    // Colors
    SketchColors colors;

    // Arc tessellation
    int minArcSegments = 8;
    int maxArcSegments = 128;
    float arcTessellationAngle = 5.0f; // degrees per segment

    // Dash pattern for construction geometry
    float dashLength = 5.0f;
    float gapLength = 3.0f;

    // Region fill opacity
    float regionOpacity = 0.0f;
    float regionHoverOpacity = 0.22f;
    float regionSelectedOpacity = 0.34f;
};

enum class RegionDisplayMode {
    HoverFirst,
    AssistAll
};

struct RegionPickHit {
    std::string id;
    double area = 0.0;
    int containmentDepth = 0;
    bool hasHoles = false;
};

/**
 * @brief Viewport information for culling
 */
struct Viewport {
    Vec2d center;
    Vec2d size;
    double zoom = 1.0;

    Vec2d getMin() const {
        return {center.x - size.x / 2, center.y - size.y / 2};
    }

    Vec2d getMax() const {
        return {center.x + size.x / 2, center.y + size.y / 2};
    }

    bool contains(const Vec2d& point) const {
        Vec2d min = getMin();
        Vec2d max = getMax();
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y;
    }

    bool intersects(const Vec2d& boundsMin, const Vec2d& boundsMax) const {
        Vec2d vpMin = getMin();
        Vec2d vpMax = getMax();
        return !(boundsMax.x < vpMin.x || boundsMin.x > vpMax.x ||
                 boundsMax.y < vpMin.y || boundsMin.y > vpMax.y);
    }
};

/**
 * @brief Selection state for entities
 */
enum class SelectionState {
    None,
    Hover,       // Mouse hovering over
    Selected,    // Part of selection set
    Dragging     // Being dragged
};

/**
 * @brief Render data for a single entity
 */
struct EntityRenderData {
    EntityID id;
    EntityType type;
    SelectionState selection = SelectionState::None;
    bool isConstruction = false;
    bool hasError = false;

    // Cached geometry for rendering
    std::vector<Vec2d> vertices;    // For lines: 2 points; for arcs: tessellated points
    Vec2d bounds[2];                // Bounding box [min, max]
};

/**
 * @brief Hit result for entity picking
 */
struct EntityPickHit {
    EntityID id;
    EntityType type;
    bool isConstruction = false;
    double distance = 0.0;
};

/**
 * @brief Render data for a constraint icon
 */
struct ConstraintRenderData {
    ConstraintID id;
    ConstraintType type;
    Vec2d position;         // Icon position in sketch space
    bool isConflicting = false;
    bool isRedundant = false;
    bool isSelected = false;
    bool isHovered = false;
    std::vector<EntityID> referencedEntities;

    // For dimensional constraints
    std::optional<double> value;
    std::optional<std::string> expression;
};

/**
 * @brief Sketch renderer class
 *
 * Handles all visual representation of sketch geometry and constraints.
 *
 * IMPLEMENTATION STATUS: COMPLETE
 * Key rendering systems implemented:
 * - VBO management for entity geometry
 * - Adaptive arc tessellation (8-256 segments)
 * - Viewport culling
 * - Selection highlighting
 * - Region rendering with triangulation
 */
// Forward declare PIMPL class
class SketchRendererImpl;

class SketchRenderer {
public:
    SketchRenderer();
    ~SketchRenderer();

    // Non-copyable
    SketchRenderer(const SketchRenderer&) = delete;
    SketchRenderer& operator=(const SketchRenderer&) = delete;

    /**
     * @brief Initialize OpenGL resources
     *
     * Must be called after OpenGL context is available.
     * Creates shaders, VBOs, textures.
     */
    bool initialize();

    /**
     * @brief Release OpenGL resources
     */
    void cleanup();

    /**
     * @brief Set the sketch to render
     */
    void setSketch(Sketch* sketch);

    /**
     * @brief Update render data from sketch
     *
     * Call when sketch geometry changes.
     * Rebuilds VBO data.
     */
    void updateGeometry();

    /**
     * @brief Update only constraint visualization
     *
     * Lighter update when only constraint state changes.
     */
    void updateConstraints();

    /**
     * @brief Render the sketch
     * @param viewMatrix View transformation matrix
     * @param projMatrix Projection matrix
     *
     * Per SPECIFICATION.md §5.18:
     * 1. Frustum cull entities outside viewport
     * 2. Render construction geometry (dashed)
     * 3. Render normal geometry (solid)
     * 4. Render selected geometry (highlighted)
     * 5. Render preview geometry (if any)
     * 6. Render constraint icons
     * 7. Render dimension text
     * 8. Render DOF indicator
     */
    void render(const QMatrix4x4& viewMatrix, const QMatrix4x4& projMatrix);

    // ========== Selection & Highlighting ==========

    /**
     * @brief Set entity selection state
     */
    void setEntitySelection(EntityID id, SelectionState state);

    /**
     * @brief Clear all selections
     */
    void clearSelection();

    /**
     * @brief Get all selected entity IDs
     */
    std::vector<EntityID> getSelectedEntities() const;

    /**
     * @brief Set hover entity
     */
    void setHoverEntity(EntityID id);

    /**
     * @brief Toggle entity selection (add if not selected, remove if selected)
     */
    void toggleEntitySelection(EntityID id);

    /**
     * @brief Set entities to highlight as conflicting
     */
    void setConflictingConstraints(const std::vector<ConstraintID>& ids);

    /**
     * @brief Set selected and hovered constraints for marker feedback.
     */
    void setSelectedConstraint(ConstraintID id);
    void setHoverConstraint(ConstraintID id);

    /**
     * @brief Hide selected marker ids while keeping constraints active.
     */
    void setSuppressedConstraints(const std::vector<ConstraintID>& ids);

    // ========== Region Selection ==========

    /**
     * @brief Find region at position (sketch coordinates)
     */
    std::optional<std::string> pickRegion(const Vec2d& sketchPos) const;

    /**
     * @brief Find all region candidates at position ordered by pick priority.
     */
    std::vector<RegionPickHit> pickRegionCandidates(const Vec2d& sketchPos) const;

    /**
     * @brief Get cached metadata for a region.
     */
    std::optional<RegionPickHit> getRegionPickHit(const std::string& regionId) const;

    void setRegionDisplayMode(RegionDisplayMode mode);
    RegionDisplayMode regionDisplayMode() const { return regionDisplayMode_; }
    void setRegionFillSuppressed(bool suppressed);
    bool isRegionFillSuppressed() const { return suppressRegionFill_; }

    /**
     * @brief Set hovered region
     */
    void setRegionHover(std::optional<std::string> regionId);

    /**
     * @brief Clear hovered region
     */
    void clearRegionHover();

    /**
     * @brief Toggle selection for region
     */
    void toggleRegionSelection(const std::string& regionId);

    /**
     * @brief Clear region selection
     */
    void clearRegionSelection();

    /**
     * @brief Check if region is selected
     */
    bool isRegionSelected(const std::string& regionId) const;

    // ========== Preview Geometry ==========

    /**
     * @brief Set preview line (for drawing tool)
     */
    void setPreviewLine(const Vec2d& start, const Vec2d& end);

    /**
     * @brief Set preview arc
     */
    void setPreviewArc(const Vec2d& center, double radius,
                       double startAngle, double endAngle);

    /**
     * @brief Set preview circle (full arc 0 to 2π)
     */
    void setPreviewCircle(const Vec2d& center, double radius);

    /**
     * @brief Set preview ellipse
     * @param center Center position
     * @param majorRadius Semi-major axis length
     * @param minorRadius Semi-minor axis length
     * @param rotation Rotation angle of major axis (radians)
     */
    void setPreviewEllipse(const Vec2d& center, double majorRadius,
                           double minorRadius, double rotation);

    /**
     * @brief Set preview rectangle (4 lines)
     */
    void setPreviewRectangle(const Vec2d& corner1, const Vec2d& corner2);

    /**
     * @brief Clear preview geometry
     */
    void clearPreview();

    // ========== Preview Dimensions ==========

    struct PreviewDimension {
        Vec2d position;  // Position in sketch coordinates
        std::string text;
        std::string id;  // Stable id for editable draft dimensions (empty = read-only)
        std::optional<double> value;  // Canonical numeric value for editor
        std::string units;  // Display units for editor (e.g. mm, °)
        Vec3d color{0.0, 0.0, 0.0}; // Optional override, defaults to style
    };

    /**
     * @brief Guide line information for snap indicators
     */
    struct GuideLineInfo {
        Vec2d origin;
        Vec2d target;
    };

    /**
     * @brief Set preview dimensions
     */
    void setPreviewDimensions(const std::vector<PreviewDimension>& dimensions);

    /**
     * @brief Get preview dimensions
     */
    const std::vector<PreviewDimension>& getPreviewDimensions() const { return previewDimensions_; }

    /**
     * @brief Get snap indicator state
     */
    const auto& getSnapIndicator() const { return snapIndicator_; }

    /**
     * @brief Get committed anchor indicators state
     */
    const std::vector<Vec2d>& getAnchorIndicators() const { return anchorIndicators_; }

    /**
     * @brief Clear preview dimensions
     */
    void clearPreviewDimensions();

    // ========== Snap Indicators ==========

    /**
     * @brief Set active guide lines for snap indicators
     */
    void setActiveGuides(const std::vector<GuideLineInfo>& guides);

    void setDragGuides(const std::vector<GuideLineInfo>& guides);

    void clearDragGuides();

    /**
     * @brief Show snap indicator at point
     *
     * Per SPECIFICATION.md §5.14:
     * Ghost icon showing snap type (vertex, grid, etc.)
     */
    void showSnapIndicator(const Vec2d& pos, SnapType type,
                           const Vec2d& guideOrigin = {0.0, 0.0},
                           bool hasGuide = false,
                           const std::string& hintText = "");

    /**
     * @brief Hide snap indicator
     */
    void hideSnapIndicator();

    /**
     * @brief Set committed anchor indicators for preview rendering
     */
    void setAnchorIndicators(const std::vector<Vec2d>& positions);

    /**
     * @brief Clear committed anchor indicators
     */
    void clearAnchorIndicators();

    /**
     * @brief Set ghost constraints for preview during drawing
     * Ghost icons show semi-transparent for inferred constraints
     */
    void setGhostConstraints(const std::vector<InferredConstraint>& ghosts);

    /**
     * @brief Clear ghost constraints
     */
    void clearGhostConstraints();

    // ========== Style & Configuration ==========

    /**
     * @brief Set rendering style
     */
    void setStyle(const SketchRenderStyle& style);
    const SketchRenderStyle& getStyle() const { return style_; }

    /**
     * @brief Set viewport for culling
     */
    void setViewport(const Viewport& viewport);

    /**
     * @brief Set pixel-to-sketch scale (for consistent line widths)
     */
    void setPixelScale(double scale);

    // ========== DOF Indicator ==========

    /**
     * @brief Update DOF display
     */
    void setDOF(int dof);

    /**
     * @brief Show/hide DOF indicator
     */
    void setShowDOF(bool show);

    // ========== Hit Testing ==========

    /**
     * @brief Find entity at screen position
     * @param screenPos Position in screen coordinates
     * @param tolerance Pick tolerance in pixels
     * @return EntityID of nearest entity, or empty string if none
     */
    EntityID pickEntity(const Vec2d& screenPos, double tolerance = 5.0) const;
    /**
     * @brief Find all entities at screen position
     * @param screenPos Position in screen coordinates
     * @param tolerance Pick tolerance in pixels
     * @return Vector of hits within tolerance, sorted by distance (nearest first)
     */
    std::vector<EntityPickHit> pickEntities(const Vec2d& screenPos, double tolerance = 5.0) const;

    /**
     * @brief Find constraint at screen position
     */
    ConstraintID pickConstraint(const Vec2d& screenPos, double tolerance = 5.0) const;

private:
    friend class SketchRendererImpl;
    // PIMPL for OpenGL internals
    std::unique_ptr<SketchRendererImpl> impl_;

    Sketch* sketch_ = nullptr;
    SketchRenderStyle style_;
    Viewport viewport_;
    double pixelScale_ = 1.0;

    // Selection state
    std::unordered_map<EntityID, SelectionState> entitySelections_;
    EntityID hoverEntity_;  // Empty string by default
    std::vector<ConstraintID> conflictingConstraints_;
    ConstraintID selectedConstraint_;
    ConstraintID hoverConstraint_;
    std::unordered_set<ConstraintID> suppressedConstraints_;

    // Preview geometry
    struct {
        bool active = false;
        EntityType type = EntityType::Line;
        std::vector<Vec2d> vertices;
    } preview_;

    std::vector<PreviewDimension> previewDimensions_;

    // Snap indicator
    struct {
        bool active = false;
        Vec2d position{0.0, 0.0};
        SnapType type = SnapType::None;
        Vec2d guideOrigin{0.0, 0.0};
        bool hasGuide = false;
        std::string hintText;
    } snapIndicator_;
    std::vector<Vec2d> anchorIndicators_;

    std::vector<GuideLineInfo> activeGuides_;
    std::vector<GuideLineInfo> dragGuides_;

    // Region data
    struct RegionRenderData {
        std::string id;
        std::string signature;
        std::vector<Vec2d> outerPolygon;
        std::vector<std::vector<Vec2d>> holes;
        std::vector<Vec2d> triangles;
        Vec2d boundsMin{0.0, 0.0};
        Vec2d boundsMax{0.0, 0.0};
        double area = 0.0;
        int containmentDepth = 0;
    };
    std::vector<RegionRenderData> regionRenderData_;
    std::unordered_set<std::string> selectedRegions_;
    std::optional<std::string> hoverRegion_;
    RegionDisplayMode regionDisplayMode_ = RegionDisplayMode::HoverFirst;
    bool suppressRegionFill_ = false;

    // DOF indicator
    int currentDOF_ = 0;
    bool showDOF_ = true;

    // Cached render data
    std::vector<EntityRenderData> entityRenderData_;
    std::vector<ConstraintRenderData> constraintRenderData_;
    std::vector<InferredConstraint> ghostConstraints_;
    bool geometryDirty_ = true;
    bool constraintsDirty_ = true;
    bool vboDirty_ = true;

    // OpenGL resources managed via PIMPL (SketchRendererImpl)

    /**
     * @brief Tessellate arc into line segments
     *
     * Per SPECIFICATION.md §5.18:
     * Adaptive tessellation based on zoom level.
     * segments = clamp(arc_angle / tessellation_angle, min, max)
     */
    std::vector<Vec2d> tessellateArc(const Vec2d& center, double radius,
                                      double startAngle, double endAngle) const;
    std::vector<Vec2d> tessellateEllipse(const Vec2d& center, double majorRadius,
                                         double minorRadius, double rotation) const;

    /**
     * @brief Update region render data from loop detection
     */
    void updateRegions();

    /**
     * @brief Build VBO data from entity render data
     */
    void buildVBOs();

    /**
     * @brief Check if entity is visible in current viewport
     */
    bool isEntityVisible(const EntityRenderData& data) const;
    bool shouldRenderConstraintMarker(const ConstraintRenderData& data) const;

    /**
     * @brief Calculate constraint icon position
     *
     * Per SPECIFICATION.md §5.16:
     * Position icons near constrained geometry
     */
    Vec2d calculateConstraintIconPosition(const SketchConstraint* constraint) const;
};

// Note: SnapResult and SnapManager are now in SnapManager.h

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_SKETCH_RENDERER_H
