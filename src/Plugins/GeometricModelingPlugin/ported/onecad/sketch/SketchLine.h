/**
 * @file SketchLine.h
 * @brief Line segment entity for sketch system
 *
 * Lines connect two points. They contribute 0 additional DOF because
 * all their freedom comes from the referenced endpoints.
 *
 * Per SPECIFICATION.md §5.12 (B1):
 * Parameterization: Two endpoints P1(x₁,y₁), P2(x₂,y₂)
 */

#ifndef ONECAD_CORE_SKETCH_LINE_H
#define ONECAD_CORE_SKETCH_LINE_H

#include "SketchEntity.h"
#include <QJsonObject>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

namespace onecad::core::sketch {

/**
 * @brief Line segment between two points
 *
 * Lines are defined by references to two SketchPoints (start and end).
 * This reference-based design enables:
 * - Rubber-band dragging (points move, lines stretch)
 * - Coincident constraints (shared endpoints)
 * - Efficient constraint solving (fewer parameters)
 */
class SketchLine : public SketchEntity {
public:
    //--------------------------------------------------------------------------
    // Construction
    //--------------------------------------------------------------------------

    /**
     * @brief Default constructor (invalid line, needs point assignment)
     */
    SketchLine();

    /**
     * @brief Construct line between two points
     * @param startPointId ID of start point
     * @param endPointId ID of end point
     */
    SketchLine(const PointID& startPointId, const PointID& endPointId);

    ~SketchLine() override = default;

    //--------------------------------------------------------------------------
    // Endpoint Access
    //--------------------------------------------------------------------------

    /**
     * @brief Get start point ID
     */
    const PointID& startPointId() const { return m_startPointId; }

    /**
     * @brief Get end point ID
     */
    const PointID& endPointId() const { return m_endPointId; }

    /**
     * @brief Set start point reference
     * @param pointId ID of start point
     */
    void setStartPointId(const PointID& pointId) { m_startPointId = pointId; }

    /**
     * @brief Set end point reference
     * @param pointId ID of end point
     */
    void setEndPointId(const PointID& pointId) { m_endPointId = pointId; }

    //--------------------------------------------------------------------------
    // Geometry Queries (require Sketch context for point lookup)
    //--------------------------------------------------------------------------

    /**
     * @brief Calculate line length
     * @param startPos Start point position
     * @param endPos End point position
     * @return Length in mm
     */
    static double length(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    /**
     * @brief Get direction vector
     * @param startPos Start point position
     * @param endPos End point position
     * @return Normalized direction vector from start to end
     */
    static gp_Vec2d direction(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    /**
     * @brief Get midpoint
     * @param startPos Start point position
     * @param endPos End point position
     * @return Midpoint coordinates
     */
    static gp_Pnt2d midpoint(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    /**
     * @brief Get angle of line relative to X axis
     * @param startPos Start point position
     * @param endPos End point position
     * @return Angle in radians [-π, π]
     */
    static double angle(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    /**
     * @brief Check if line is horizontal (within tolerance)
     * @param startPos Start point position
     * @param endPos End point position
     * @param tolerance Angular tolerance in radians
     * @return true if horizontal
     */
    static bool isHorizontal(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                             double tolerance = 1e-6);

    /**
     * @brief Check if line is vertical (within tolerance)
     * @param startPos Start point position
     * @param endPos End point position
     * @param tolerance Angular tolerance in radians
     * @return true if vertical
     */
    static bool isVertical(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                           double tolerance = 1e-6);

    /**
     * @brief Calculate distance from point to line segment
     * @param point Test point
     * @param startPos Line start
     * @param endPos Line end
     * @return Perpendicular distance (or distance to nearest endpoint if outside segment)
     */
    static double distanceToPoint(const gp_Pnt2d& point,
                                  const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    //--------------------------------------------------------------------------
    // SketchEntity Interface
    //--------------------------------------------------------------------------

    EntityType type() const override { return EntityType::Line; }
    std::string typeName() const override { return "Line"; }

    /**
     * @brief Calculate bounds (requires external point positions)
     * @note This base implementation returns empty bounds.
     *       Sketch class calculates real bounds using point lookup.
     */
    BoundingBox2d bounds() const override;

    /**
     * @brief Check if near point (requires external point positions)
     * @note This base implementation returns false.
     *       Sketch class performs real hit testing.
     */
    bool isNear(const gp_Pnt2d& point, double tolerance) const override;

    /**
     * @brief Lines contribute 0 additional DOF (endpoints have all the freedom)
     */
    int degreesOfFreedom() const override { return 0; }

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;

    //--------------------------------------------------------------------------
    // Bounds calculation with point positions (called by Sketch)
    //--------------------------------------------------------------------------

    /**
     * @brief Calculate bounds with known point positions
     * @param startPos Start point position
     * @param endPos End point position
     * @return Bounding box
     */
    static BoundingBox2d boundsWithPoints(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos);

    /**
     * @brief Check if near point with known line positions
     * @param testPoint Point to test
     * @param startPos Line start
     * @param endPos Line end
     * @param tolerance Distance tolerance
     * @return true if within tolerance
     */
    static bool isNearWithPoints(const gp_Pnt2d& testPoint,
                                 const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                                 double tolerance);

private:
    PointID m_startPointId;
    PointID m_endPointId;
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_LINE_H
