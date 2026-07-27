/**
 * @file SketchEllipse.h
 * @brief Ellipse entity for sketch system
 *
 * An ellipse is defined by center, major/minor radii, and rotation angle.
 * DOF: 3 (majorRadius, minorRadius, rotation) + 2 from center point = 5 total
 */

#ifndef ONECAD_CORE_SKETCH_ELLIPSE_H
#define ONECAD_CORE_SKETCH_ELLIPSE_H

#include "SketchEntity.h"
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>
#include <cmath>

namespace onecad::core::sketch {

// Forward declaration for friend access
namespace solver {
class ConstraintSolver;
}

/**
 * @brief Ellipse defined by center, major/minor radii, and rotation
 *
 * The major axis is rotated by m_rotation radians from the +X axis.
 * Major radius >= minor radius (enforced).
 */
class SketchEllipse : public SketchEntity {
    friend class solver::ConstraintSolver;

public:
    //--------------------------------------------------------------------------
    // Construction
    //--------------------------------------------------------------------------

    /**
     * @brief Default constructor (invalid ellipse)
     */
    SketchEllipse();

    /**
     * @brief Construct ellipse with center, radii, and rotation
     * @param centerPointId ID of center point
     * @param majorRadius Semi-major axis length (mm)
     * @param minorRadius Semi-minor axis length (mm)
     * @param rotation Rotation angle of major axis from +X (radians)
     */
    SketchEllipse(const PointID& centerPointId, double majorRadius,
                  double minorRadius, double rotation = 0.0);

    ~SketchEllipse() override = default;

    //--------------------------------------------------------------------------
    // Geometry Access
    //--------------------------------------------------------------------------

    const PointID& centerPointId() const { return m_centerPointId; }
    void setCenterPointId(const PointID& pointId) { m_centerPointId = pointId; }

    double majorRadius() const { return m_majorRadius; }
    void setMajorRadius(double r);

    double minorRadius() const { return m_minorRadius; }
    void setMinorRadius(double r);

    double rotation() const { return m_rotation; }
    void setRotation(double angle) { m_rotation = angle; }

    //--------------------------------------------------------------------------
    // Derived Geometry
    //--------------------------------------------------------------------------

    /**
     * @brief Get ellipse circumference (approximation)
     * Uses Ramanujan's approximation: π(3(a+b) - √((3a+b)(a+3b)))
     */
    double circumference() const;

    /**
     * @brief Get ellipse area
     * @return Area = π * a * b
     */
    double area() const { return M_PI * m_majorRadius * m_minorRadius; }

    /**
     * @brief Calculate point on ellipse at given parametric angle
     * @param centerPos Center position
     * @param t Parametric angle in radians (0 to 2π)
     * @return Point on ellipse
     */
    gp_Pnt2d pointAtParameter(const gp_Pnt2d& centerPos, double t) const;

    /**
     * @brief Get tangent direction at given parametric angle
     * @param t Parametric angle in radians
     * @return Unit tangent vector
     */
    gp_Vec2d tangentAtParameter(double t) const;

    /**
     * @brief Check if point is inside ellipse
     * @param centerPos Center position
     * @param point Point to test
     * @return true if point is strictly inside
     */
    bool containsPoint(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const;

    /**
     * @brief Get foci positions
     * @param centerPos Center position
     * @param focus1 Output: first focus
     * @param focus2 Output: second focus
     */
    void getFoci(const gp_Pnt2d& centerPos, gp_Pnt2d& focus1, gp_Pnt2d& focus2) const;

    //--------------------------------------------------------------------------
    // SketchEntity Interface
    //--------------------------------------------------------------------------

    EntityType type() const override { return EntityType::Ellipse; }
    std::string typeName() const override { return "Ellipse"; }

    BoundingBox2d bounds() const override;
    bool isNear(const gp_Pnt2d& point, double tolerance) const override;

    /**
     * @brief Ellipse DOF: majorRadius, minorRadius, rotation (3)
     * @note Center point contributes its own 2 DOF separately
     */
    int degreesOfFreedom() const override { return 3; }

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;

    //--------------------------------------------------------------------------
    // Bounds/hit test with center position (called by Sketch)
    //--------------------------------------------------------------------------

    BoundingBox2d boundsWithCenter(const gp_Pnt2d& centerPos) const;
    bool isNearWithCenter(const gp_Pnt2d& testPoint, const gp_Pnt2d& centerPos,
                          double tolerance) const;

private:
    /**
     * @brief Direct memory access for constraint solver parameter binding
     * @note These bypass validation - only for internal solver use
     */
    double* majorRadiusPtr() { return &m_majorRadius; }
    double* minorRadiusPtr() { return &m_minorRadius; }
    double* rotationPtr() { return &m_rotation; }

    PointID m_centerPointId;
    double m_majorRadius = 0.0;
    double m_minorRadius = 0.0;
    double m_rotation = 0.0;  // radians, major axis angle from +X
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_ELLIPSE_H
