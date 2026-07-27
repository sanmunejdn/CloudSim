/**
 * @file SketchCircle.h
 * @brief Full circle entity for sketch system
 *
 * A circle is a complete 360° arc. It references a center point and has a radius.
 * DOF: 1 (radius only) + 2 from center point = 3 total
 */

#ifndef ONECAD_CORE_SKETCH_CIRCLE_H
#define ONECAD_CORE_SKETCH_CIRCLE_H

#include "SketchEntity.h"
#include <gp_Pnt2d.hxx>
#include <cmath>

namespace onecad::core::sketch {

/**
 * @brief Full circle defined by center point and radius
 *
 * Unlike arcs, circles have no start/end angles - they are complete loops.
 * This simplifies constraint solving and loop detection.
 */
class SketchCircle : public SketchEntity {
public:
    //--------------------------------------------------------------------------
    // Construction
    //--------------------------------------------------------------------------

    /**
     * @brief Default constructor (invalid circle)
     */
    SketchCircle();

    /**
     * @brief Construct circle with center and radius
     * @param centerPointId ID of center point
     * @param radius Circle radius (mm)
     */
    SketchCircle(const PointID& centerPointId, double radius);

    ~SketchCircle() override = default;

    //--------------------------------------------------------------------------
    // Geometry Access
    //--------------------------------------------------------------------------

    /**
     * @brief Get center point ID
     */
    const PointID& centerPointId() const { return m_centerPointId; }

    /**
     * @brief Set center point reference
     */
    void setCenterPointId(const PointID& pointId) { m_centerPointId = pointId; }

    /**
     * @brief Get circle radius
     * @return Radius in mm
     */
    double radius() const { return m_radius; }

    /**
     * @brief Get mutable reference to radius (for solver binding)
     *
     * Solver callers must keep radius non-negative or clamp after solve.
     */
    double& radius() { return m_radius; }

    /**
     * @brief Set circle radius
     * @param radius Radius in mm (must be positive)
     */
    void setRadius(double radius) { m_radius = std::max(0.0, radius); }

    //--------------------------------------------------------------------------
    // Derived Geometry
    //--------------------------------------------------------------------------

    /**
     * @brief Get circle circumference
     * @return Circumference in mm
     */
    double circumference() const { return 2.0 * M_PI * m_radius; }

    /**
     * @brief Get circle area
     * @return Area in mm²
     */
    double area() const { return M_PI * m_radius * m_radius; }

    /**
     * @brief Get diameter
     * @return Diameter in mm
     */
    double diameter() const { return 2.0 * m_radius; }

    /**
     * @brief Calculate point on circle at given angle
     * @param centerPos Center position
     * @param angle Angle in radians (from +X axis)
     * @return Point on circumference
     */
    gp_Pnt2d pointAtAngle(const gp_Pnt2d& centerPos, double angle) const;

    /**
     * @brief Get tangent direction at given angle
     * @param angle Angle in radians
     * @return Unit tangent vector (CCW direction)
     */
    gp_Vec2d tangentAtAngle(double angle) const;

    /**
     * @brief Check if point is inside circle
     * @param centerPos Center position
     * @param point Point to test
     * @return true if point is strictly inside (distance < radius)
     */
    bool containsPoint(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const;

    /**
     * @brief Calculate distance from point to circle edge
     * @param centerPos Center position
     * @param point Test point
     * @return Distance to circumference (negative if inside)
     */
    double distanceToEdge(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const;

    //--------------------------------------------------------------------------
    // SketchEntity Interface
    //--------------------------------------------------------------------------

    EntityType type() const override { return EntityType::Circle; }
    std::string typeName() const override { return "Circle"; }

    /**
     * @brief Base bounds requires center resolution (use boundsWithCenter via Sketch)
     */
    BoundingBox2d bounds() const override;

    /**
     * @brief Base hit test requires center resolution (use isNearWithCenter via Sketch)
     */
    bool isNear(const gp_Pnt2d& point, double tolerance) const override;

    /**
     * @brief Circle DOF: radius (1)
     * @note Center point contributes its own 2 DOF separately
     */
    int degreesOfFreedom() const override { return 1; }

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;

    //--------------------------------------------------------------------------
    // Bounds/hit test with center position (called by Sketch)
    //--------------------------------------------------------------------------

    BoundingBox2d boundsWithCenter(const gp_Pnt2d& centerPos) const;
    bool isNearWithCenter(const gp_Pnt2d& testPoint, const gp_Pnt2d& centerPos,
                          double tolerance) const;

private:
    PointID m_centerPointId;
    double m_radius = 0.0;
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_CIRCLE_H
