/**
 * @file Constraints.h
 * @brief Concrete constraint implementations for sketch system
 *
 * This file defines constraint types supported in v1.0:
 * - Positional: Coincident, Horizontal, Vertical (TODO: Midpoint)
 * - Relational: Parallel, Perpendicular, Tangent, Equal (TODO: Concentric)
 * - Dimensional: Distance, Angle, Radius (TODO: Diameter)
 * - Symmetry: TODO: Symmetric about line
 */

#ifndef ONECAD_CORE_SKETCH_CONSTRAINTS_H
#define ONECAD_CORE_SKETCH_CONSTRAINTS_H

#include "../SketchConstraint.h"
#include "../SketchTypes.h"

#include <gp_Pnt2d.hxx>
#include <optional>

namespace onecad::core::sketch::constraints {

//==============================================================================
// POSITIONAL CONSTRAINTS
//==============================================================================

/**
 * @brief Fixed constraint - fixes a point to specific coordinates
 *
 * DOF removed: 2 (both X and Y are fixed)
 */
class FixedConstraint : public SketchConstraint {
public:
    FixedConstraint(const PointID& pointId, double x, double y);

    ConstraintType type() const override { return ConstraintType::Fixed; }
    std::string typeName() const override { return "Fixed"; }
    std::string toString() const override;

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 2; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const PointID& pointId() const { return m_pointId; }
    double fixedX() const { return m_fixedX; }
    double fixedY() const { return m_fixedY; }
    const double& fixedXRef() const { return m_fixedX; }
    const double& fixedYRef() const { return m_fixedY; }

    /** Add (dx, dy) to fixed position (e.g. when translating whole sketch). */
    void translate(double dx, double dy);

private:
    friend class onecad::core::sketch::ConstraintFactory;
    FixedConstraint() = default;
    PointID m_pointId;
    double m_fixedX = 0.0;
    double m_fixedY = 0.0;
};

/**
 * @brief Midpoint constraint - constrains a point to lie on the midpoint of a line
 *
 * DOF removed: 2 (the point is fully determined by the line's midpoint)
 */
class MidpointConstraint : public SketchConstraint {
public:
    MidpointConstraint(const PointID& pointId, const EntityID& lineId);

    ConstraintType type() const override { return ConstraintType::Midpoint; }
    std::string typeName() const override { return "Midpoint"; }
    std::string toString() const override { return "Midpoint"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 2; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const PointID& pointId() const { return m_pointId; }
    const EntityID& lineId() const { return m_lineId; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    MidpointConstraint() = default;
    PointID m_pointId;
    EntityID m_lineId;
};

/**
 * @brief Coincident constraint - makes two points occupy the same location
 *
 * DOF removed: 2 (merges two independent points)
 */
class CoincidentConstraint : public SketchConstraint {
public:
    CoincidentConstraint(const PointID& point1, const PointID& point2);

    ConstraintType type() const override { return ConstraintType::Coincident; }
    std::string typeName() const override { return "Coincident"; }
    std::string toString() const override { return "Coincident"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 2; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const PointID& point1() const { return m_point1; }
    const PointID& point2() const { return m_point2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    CoincidentConstraint() = default;
    PointID m_point1;
    PointID m_point2;
};

/**
 * @brief Horizontal constraint - makes line horizontal
 *
 * DOF removed: 1 (fixes Y difference to 0)
 */
class HorizontalConstraint : public SketchConstraint {
public:
    explicit HorizontalConstraint(const EntityID& lineId);

    ConstraintType type() const override { return ConstraintType::Horizontal; }
    std::string typeName() const override { return "Horizontal"; }
    std::string toString() const override { return "Horizontal"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& lineId() const { return m_lineId; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    HorizontalConstraint() = default;
    EntityID m_lineId;
};

/**
 * @brief Vertical constraint - makes line vertical
 *
 * DOF removed: 1 (fixes X difference to 0)
 */
class VerticalConstraint : public SketchConstraint {
public:
    explicit VerticalConstraint(const EntityID& lineId);

    ConstraintType type() const override { return ConstraintType::Vertical; }
    std::string typeName() const override { return "Vertical"; }
    std::string toString() const override { return "Vertical"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& lineId() const { return m_lineId; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    VerticalConstraint() = default;
    EntityID m_lineId;
};

//==============================================================================
// RELATIONAL CONSTRAINTS
//==============================================================================

/**
 * @brief Parallel constraint - makes two lines parallel
 *
 * DOF removed: 1 (fixes angle difference to 0)
 */
class ParallelConstraint : public SketchConstraint {
public:
    ParallelConstraint(const EntityID& line1, const EntityID& line2);

    ConstraintType type() const override { return ConstraintType::Parallel; }
    std::string typeName() const override { return "Parallel"; }
    std::string toString() const override { return "Parallel"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& line1() const { return m_line1; }
    const EntityID& line2() const { return m_line2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    ParallelConstraint() = default;
    EntityID m_line1;
    EntityID m_line2;
};

/**
 * @brief Perpendicular constraint - makes two lines perpendicular
 *
 * DOF removed: 1 (fixes angle difference to 90°)
 */
class PerpendicularConstraint : public SketchConstraint {
public:
    PerpendicularConstraint(const EntityID& line1, const EntityID& line2);

    ConstraintType type() const override { return ConstraintType::Perpendicular; }
    std::string typeName() const override { return "Perpendicular"; }
    std::string toString() const override { return "Perpendicular"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& line1() const { return m_line1; }
    const EntityID& line2() const { return m_line2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    PerpendicularConstraint() = default;
    EntityID m_line1;
    EntityID m_line2;
};

/**
 * @brief Tangent constraint - makes arc/circle tangent to line or other curve
 *
 * DOF removed: 1
 *
 * Per SPECIFICATION.md §23.9:
 * For arc-to-line tangency: distance(center, line) = radius
 */
class TangentConstraint : public SketchConstraint {
public:
    TangentConstraint(const EntityID& entity1, const EntityID& entity2);

    ConstraintType type() const override { return ConstraintType::Tangent; }
    std::string typeName() const override { return "Tangent"; }
    std::string toString() const override { return "Tangent"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& entity1() const { return m_entity1; }
    const EntityID& entity2() const { return m_entity2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    TangentConstraint() = default;
    EntityID m_entity1;
    EntityID m_entity2;
};

/**
 * @brief Equal constraint - makes two entities have equal size
 *
 * DOF removed: 1 (for lines: equal length, for circles/arcs: equal radius)
 */
class EqualConstraint : public SketchConstraint {
public:
    EqualConstraint(const EntityID& entity1, const EntityID& entity2);

    ConstraintType type() const override { return ConstraintType::Equal; }
    std::string typeName() const override { return "Equal"; }
    std::string toString() const override { return "Equal"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& entity1() const { return m_entity1; }
    const EntityID& entity2() const { return m_entity2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    EqualConstraint() = default;
    EntityID m_entity1;
    EntityID m_entity2;
};

//==============================================================================
// DIMENSIONAL CONSTRAINTS
//==============================================================================

/**
 * @brief Distance constraint - fixes distance between two entities
 *
 * DOF removed: 1
 *
 * Supports: point-point, point-line, line-line (parallel distance)
 */
class DistanceConstraint : public DimensionalConstraint {
public:
    DistanceConstraint(const EntityID& entity1, const EntityID& entity2, double distance);

    ConstraintType type() const override { return ConstraintType::Distance; }
    std::string typeName() const override { return "Distance"; }
    std::string toString() const override;
    std::string units() const override { return "mm"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& entity1() const { return m_entity1; }
    const EntityID& entity2() const { return m_entity2; }
    double distance() const { return value(); }
    void setDistance(double d) { setValue(d); }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    DistanceConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_entity1;
    EntityID m_entity2;
};

/**
 * @brief Signed horizontal distance between two points (x2 - x1)
 */
class HorizontalDistanceConstraint : public DimensionalConstraint {
public:
    HorizontalDistanceConstraint(const EntityID& point1, const EntityID& point2, double distance);

    ConstraintType type() const override { return ConstraintType::HorizontalDistance; }
    std::string typeName() const override { return "HorizontalDistance"; }
    std::string toString() const override;
    std::string units() const override { return "mm"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& point1() const { return m_point1; }
    const EntityID& point2() const { return m_point2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    HorizontalDistanceConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_point1;
    EntityID m_point2;
};

/**
 * @brief Signed vertical distance between two points (y2 - y1)
 */
class VerticalDistanceConstraint : public DimensionalConstraint {
public:
    VerticalDistanceConstraint(const EntityID& point1, const EntityID& point2, double distance);

    ConstraintType type() const override { return ConstraintType::VerticalDistance; }
    std::string typeName() const override { return "VerticalDistance"; }
    std::string toString() const override;
    std::string units() const override { return "mm"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& point1() const { return m_point1; }
    const EntityID& point2() const { return m_point2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    VerticalDistanceConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_point1;
    EntityID m_point2;
};

/**
 * @brief Angle constraint - fixes angle between two lines
 *
 * DOF removed: 1
 *
 * Per SPECIFICATION.md §G3: Angle range is -180° to +180° (signed)
 */
class AngleConstraint : public DimensionalConstraint {
public:
    AngleConstraint(const EntityID& line1, const EntityID& line2, double angleRadians);

    ConstraintType type() const override { return ConstraintType::Angle; }
    std::string typeName() const override { return "Angle"; }
    std::string toString() const override;
    std::string units() const override { return "°"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& line1() const { return m_line1; }
    const EntityID& line2() const { return m_line2; }

    double angleRadians() const { return value(); }
    double angleDegrees() const;
    void setAngleRadians(double rad) { setValue(rad); }
    void setAngleDegrees(double deg);

private:
    friend class onecad::core::sketch::ConstraintFactory;
    AngleConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_line1;
    EntityID m_line2;
};

/**
 * @brief Radius constraint - fixes radius of circle or arc
 *
 * DOF removed: 1
 */
class RadiusConstraint : public DimensionalConstraint {
public:
    RadiusConstraint(const EntityID& circleOrArc, double radius);

    ConstraintType type() const override { return ConstraintType::Radius; }
    std::string typeName() const override { return "Radius"; }
    std::string toString() const override;
    std::string units() const override { return "mm"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& entityId() const { return m_entityId; }
    double radius() const { return value(); }
    void setRadius(double r) { setValue(r); }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    RadiusConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_entityId;
};

/**
 * @brief Diameter constraint - fixes diameter of circle or arc
 *
 * DOF removed: 1 (equivalent to radius constraint, but expressed as diameter)
 */
class DiameterConstraint : public DimensionalConstraint {
public:
    DiameterConstraint(const EntityID& circleOrArc, double diameter);

    ConstraintType type() const override { return ConstraintType::Diameter; }
    std::string typeName() const override { return "Diameter"; }
    std::string toString() const override;
    std::string units() const override { return "mm"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 1; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;
    gp_Pnt2d getDimensionTextPosition(const Sketch& sketch) const override;

    const EntityID& entityId() const { return m_entityId; }
    double diameter() const { return value(); }
    void setDiameter(double d) { setValue(d); }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    DiameterConstraint() : DimensionalConstraint(0.0) {}
    EntityID m_entityId;
};

/**
 * @brief Concentric constraint - makes two circles/arcs share the same center
 *
 * DOF removed: 2 (equivalent to coincident on center points)
 */
class ConcentricConstraint : public SketchConstraint {
public:
    ConcentricConstraint(const EntityID& entity1, const EntityID& entity2);

    ConstraintType type() const override { return ConstraintType::Concentric; }
    std::string typeName() const override { return "Concentric"; }
    std::string toString() const override { return "Concentric"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 2; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& entity1() const { return m_entity1; }
    const EntityID& entity2() const { return m_entity2; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    ConcentricConstraint() = default;
    EntityID m_entity1;
    EntityID m_entity2;
};

/**
 * @brief Symmetric constraint - keeps two points mirrored about an axis line
 */
class SymmetricConstraint : public SketchConstraint {
public:
    SymmetricConstraint(const EntityID& point1, const EntityID& point2, const EntityID& axisLine);

    ConstraintType type() const override { return ConstraintType::Symmetric; }
    std::string typeName() const override { return "Symmetric"; }
    std::string toString() const override { return "Symmetric"; }

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override { return 2; }

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& point1() const { return m_point1; }
    const EntityID& point2() const { return m_point2; }
    const EntityID& axisLine() const { return m_axisLine; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    SymmetricConstraint() = default;
    EntityID m_point1;
    EntityID m_point2;
    EntityID m_axisLine;
};

/**
 * @brief Point on curve constraint - constrains point to lie on curve
 *
 * Supports lines, circles, and arcs. Ellipse support is blocked until solver support is wired.
 * Position can be Start, End, or Arbitrary:
 * - Start/End: Point fully constrained to calculated endpoint (currently solver-backed for lines only)
 * - Arbitrary: Point constrained to curve but can slide (DOF removed: 1)
 */
class PointOnCurveConstraint : public SketchConstraint {
public:
    PointOnCurveConstraint(const EntityID& pointId, const EntityID& curveId,
                           CurvePosition position = CurvePosition::Arbitrary);

    ConstraintType type() const override { return ConstraintType::OnCurve; }
    std::string typeName() const override { return "PointOnCurve"; }
    std::string toString() const override;

    std::vector<EntityID> referencedEntities() const override;
    int degreesRemoved() const override;

    bool isSatisfied(const Sketch& sketch, double tolerance) const override;
    double getError(const Sketch& sketch) const override;

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;
    gp_Pnt2d getIconPosition(const Sketch& sketch) const override;

    const EntityID& pointId() const { return m_pointId; }
    const EntityID& curveId() const { return m_curveId; }
    CurvePosition position() const { return m_position; }

private:
    friend class onecad::core::sketch::ConstraintFactory;
    PointOnCurveConstraint() = default;
    EntityID m_pointId;
    EntityID m_curveId;
    CurvePosition m_position = CurvePosition::Arbitrary;
};

} // namespace onecad::core::sketch::constraints

#endif // ONECAD_CORE_SKETCH_CONSTRAINTS_H
