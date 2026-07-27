/**
 * @file AutoConstrainer.h
 * @brief Automatic constraint inference for sketch drawing
 *
 * Per SPECIFICATION.md §5.9, §5.14 and SKETCH_IMPLEMENTATION_PLAN.md:
 * - Auto-constrain ON by default (Shapr3D style)
 * - Ghost icon opacity 50% during preview
 * - Inference rules:
 *   - Line within ±5° of horizontal → Horizontal
 *   - Line within ±5° of vertical → Vertical
 *   - Endpoint within 2mm of existing point → Coincident
 *   - Arc starts at line endpoint in tangent direction → Tangent
 *   - Lines meet at ~90±5° → Perpendicular
 *   - Drawing parallel to existing line → Parallel
 */
#ifndef ONECAD_CORE_SKETCH_AUTO_CONSTRAINER_H
#define ONECAD_CORE_SKETCH_AUTO_CONSTRAINER_H

#include "SketchTypes.h"
#include <optional>
#include <unordered_map>
#include <vector>

namespace onecad::core::sketch {

class Sketch;
class SketchEntity;

/**
 * @brief Inferred constraint from auto-constraining system
 */
struct InferredConstraint {
    ConstraintType type;
    EntityID entity1;
    std::optional<EntityID> entity2;  // For 2-entity constraints
    double confidence = 1.0;          // 0.0-1.0 for UI preview intensity
    std::optional<double> value;      // For dimensional constraints (distance, angle, radius)

    // For position-based constraints (coincident, on-curve)
    std::optional<Vec2d> position;

    bool operator==(const InferredConstraint& other) const {
        return type == other.type &&
               entity1 == other.entity1 &&
               entity2 == other.entity2;
    }
};

/**
 * @brief Configuration for auto-constraining
 */
struct AutoConstrainerConfig {
    // Angular tolerances (radians)
    double horizontalTolerance = 5.0 * 3.14159265358979323846 / 180.0;  // ±5°
    double verticalTolerance = 5.0 * 3.14159265358979323846 / 180.0;    // ±5°
    double perpendicularTolerance = 5.0 * 3.14159265358979323846 / 180.0; // 90±5°
    double parallelTolerance = 5.0 * 3.14159265358979323846 / 180.0;    // ±5°
    double tangentTolerance = 5.0 * 3.14159265358979323846 / 180.0;     // ±5°

    // Distance tolerances (mm)
    double coincidenceTolerance = 2.0;  // 2mm per spec (same as snap radius)

    // Confidence thresholds
    double autoApplyThreshold = 0.5;  // Auto-apply if confidence >= this

    // Master enable
    bool enabled = true;
};

/**
 * @brief Drawing context for constraint inference
 *
 * Provides information about the current drawing state for smarter inference.
 */
struct DrawingContext {
    EntityID activeEntity;           // Entity currently being drawn
    std::optional<EntityID> previousEntity;  // Last completed entity (for chaining)
    Vec2d startPoint{0, 0};          // Start point of current stroke
    Vec2d currentPoint{0, 0};        // Current cursor position
    bool isFirstPoint = true;        // True if placing first point
    bool isPolylineMode = false;     // True if in polyline/chain mode
};

/**
 * @brief Auto-constrainer for intelligent constraint inference
 *
 * Analyzes cursor position and drawing context to infer constraints that
 * should be applied to geometry as it's drawn.
 *
 * Per SPECIFICATION.md §5.14:
 * Shows ghost icons during drawing, applies on confirmation.
 */
class AutoConstrainer {
public:
    AutoConstrainer();
    explicit AutoConstrainer(const AutoConstrainerConfig& config);
    ~AutoConstrainer() = default;

    // Non-copyable, movable
    AutoConstrainer(const AutoConstrainer&) = delete;
    AutoConstrainer& operator=(const AutoConstrainer&) = delete;
    AutoConstrainer(AutoConstrainer&&) = default;
    AutoConstrainer& operator=(AutoConstrainer&&) = default;

    /**
     * @brief Infer constraints for a point being placed
     * @param point Position of point being placed (in sketch coords)
     * @param sketch Current sketch
     * @param context Drawing context
     * @return Vector of inferred constraints with confidence levels
     *
     * This is the main entry point for constraint inference during drawing.
     */
    std::vector<InferredConstraint> inferConstraints(
        const Vec2d& point,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer constraints for a line being drawn
     * @param startPoint Line start position
     * @param endPoint Line end position (current cursor)
     * @param lineId ID of the line entity (if created)
     * @param sketch Current sketch
     * @param context Drawing context
     * @return Vector of inferred constraints
     */
    std::vector<InferredConstraint> inferLineConstraints(
        const Vec2d& startPoint,
        const Vec2d& endPoint,
        EntityID lineId,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer constraints for a circle being drawn
     * @param center Circle center
     * @param radius Circle radius
     * @param circleId ID of the circle entity (if created)
     * @param sketch Current sketch
     * @param context Drawing context
     * @return Vector of inferred constraints
     */
    std::vector<InferredConstraint> inferCircleConstraints(
        const Vec2d& center,
        double radius,
        EntityID circleId,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer constraints for an arc being drawn
     * @param center Arc center
     * @param radius Arc radius
     * @param startAngle Start angle (radians)
     * @param endAngle End angle (radians)
     * @param arcId ID of the arc entity (if created)
     * @param sketch Current sketch
     * @param context Drawing context
     * @return Vector of inferred constraints
     */
    std::vector<InferredConstraint> inferArcConstraints(
        const Vec2d& center,
        double radius,
        double startAngle,
        double endAngle,
        EntityID arcId,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Filter inferred constraints to only those that should auto-apply
     * @param constraints All inferred constraints
     * @return Constraints with confidence >= autoApplyThreshold
     */
    std::vector<InferredConstraint> filterForAutoApply(
        const std::vector<InferredConstraint>& constraints) const;

    // ========== Configuration ==========

    void setConfig(const AutoConstrainerConfig& config) { config_ = config; }
    const AutoConstrainerConfig& getConfig() const { return config_; }

    /**
     * @brief Master enable/disable
     */
    void setEnabled(bool enabled) { config_.enabled = enabled; }
    bool isEnabled() const { return config_.enabled; }

    /**
     * @brief Enable/disable specific constraint type inference
     */
    void setTypeEnabled(ConstraintType type, bool enabled);
    bool isTypeEnabled(ConstraintType type) const;

    /**
     * @brief Enable all constraint types
     */
    void setAllTypesEnabled(bool enabled);

private:
    AutoConstrainerConfig config_;
    std::unordered_map<ConstraintType, bool> typeEnabled_;

    // ========== Individual Inference Methods ==========

    /**
     * @brief Infer horizontal constraint for a line
     * @param startPoint Line start
     * @param endPoint Line end
     * @param lineId Line entity ID
     * @return Horizontal constraint if applicable, nullopt otherwise
     */
    std::optional<InferredConstraint> inferHorizontal(
        const Vec2d& startPoint,
        const Vec2d& endPoint,
        EntityID lineId) const;

    /**
     * @brief Infer vertical constraint for a line
     * @param startPoint Line start
     * @param endPoint Line end
     * @param lineId Line entity ID
     * @return Vertical constraint if applicable, nullopt otherwise
     */
    std::optional<InferredConstraint> inferVertical(
        const Vec2d& startPoint,
        const Vec2d& endPoint,
        EntityID lineId) const;

    /**
     * @brief Infer coincident constraint for a point
     * @param point Point position
     * @param sketch Current sketch
     * @param excludeEntity Entity to exclude (usually the one being drawn)
     * @return Coincident constraint with nearest point if applicable
     */
    std::optional<InferredConstraint> inferCoincident(
        const Vec2d& point,
        const Sketch& sketch,
        EntityID excludeEntity) const;

    /**
     * @brief Infer perpendicular constraint between two lines
     * @param line1Start First line start
     * @param line1End First line end
     * @param line1Id First line ID
     * @param sketch Current sketch
     * @return Perpendicular constraint with existing line if applicable
     */
    std::optional<InferredConstraint> inferPerpendicular(
        const Vec2d& line1Start,
        const Vec2d& line1End,
        EntityID line1Id,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer parallel constraint with existing lines
     * @param lineStart Line start
     * @param lineEnd Line end
     * @param lineId Line ID
     * @param sketch Current sketch
     * @return Parallel constraint with nearest parallel line if applicable
     */
    std::optional<InferredConstraint> inferParallel(
        const Vec2d& lineStart,
        const Vec2d& lineEnd,
        EntityID lineId,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer tangent constraint for arc starting from line endpoint
     * @param arcCenter Arc center
     * @param arcStartPoint Arc start point
     * @param arcId Arc entity ID
     * @param sketch Current sketch
     * @param context Drawing context
     * @return Tangent constraint if arc starts tangent to a line
     */
    std::optional<InferredConstraint> inferTangent(
        const Vec2d& arcCenter,
        const Vec2d& arcStartPoint,
        EntityID arcId,
        const Sketch& sketch,
        const DrawingContext& context) const;

    /**
     * @brief Infer concentric constraint for circle/arc centers
     * @param center Center point
     * @param entityId Entity ID (circle or arc)
     * @param sketch Current sketch
     * @return Concentric constraint if center matches existing circle/arc center
     */
    std::optional<InferredConstraint> inferConcentric(
        const Vec2d& center,
        EntityID entityId,
        const Sketch& sketch) const;

    /**
     * @brief Infer equal radius constraint for circles/arcs
     * @param radius Radius value
     * @param entityId Entity ID (circle or arc)
     * @param sketch Current sketch
     * @return Equal constraint if radius matches existing circle/arc
     */
    std::optional<InferredConstraint> inferEqualRadius(
        double radius,
        EntityID entityId,
        const Sketch& sketch) const;

    // ========== Geometry Helpers ==========

    /**
     * @brief Calculate angle of line relative to X axis
     * @return Angle in radians [-π, π]
     */
    static double lineAngle(const Vec2d& start, const Vec2d& end);

    /**
     * @brief Calculate angle between two lines
     * @return Angle in radians [0, π]
     */
    static double angleBetweenLines(
        const Vec2d& line1Start, const Vec2d& line1End,
        const Vec2d& line2Start, const Vec2d& line2End);

    /**
     * @brief Check if two lines are approximately perpendicular
     */
    bool areLinesPerpendicular(
        const Vec2d& line1Start, const Vec2d& line1End,
        const Vec2d& line2Start, const Vec2d& line2End) const;

    /**
     * @brief Check if two lines are approximately parallel
     */
    bool areLinesParallel(
        const Vec2d& line1Start, const Vec2d& line1End,
        const Vec2d& line2Start, const Vec2d& line2End) const;

    /**
     * @brief Calculate distance between two points
     */
    static double distance(const Vec2d& a, const Vec2d& b);
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_AUTO_CONSTRAINER_H
