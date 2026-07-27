/**
 * @file SketchPoint.h
 * @brief 2D point entity for sketch system
 *
 * Points are the fundamental building blocks of sketches. Lines, arcs, and circles
 * reference points for their geometry. Points have 2 degrees of freedom (X, Y).
 */

#ifndef ONECAD_CORE_SKETCH_POINT_H
#define ONECAD_CORE_SKETCH_POINT_H

#include "SketchEntity.h"
#include <gp_Pnt2d.hxx>
#include <vector>

namespace onecad::core::sketch {

/**
 * @brief 2D point in sketch coordinate system
 *
 * Per SPECIFICATION.md §5.12:
 * - Parameterization: (x, y) position
 * - DOF: 2 (can move in X and Y)
 *
 * Points serve as vertices for lines and arcs, and as centers for circles.
 */
class SketchPoint : public SketchEntity {
public:
    //--------------------------------------------------------------------------
    // Construction
    //--------------------------------------------------------------------------

    /**
     * @brief Construct a point at origin
     */
    SketchPoint();

    /**
     * @brief Construct a point at specified position
     * @param position 2D position in sketch coordinates (mm)
     */
    explicit SketchPoint(const gp_Pnt2d& position);

    /**
     * @brief Construct a point at specified coordinates
     * @param x X coordinate (mm)
     * @param y Y coordinate (mm)
     */
    SketchPoint(double x, double y);

    ~SketchPoint() override = default;

    //--------------------------------------------------------------------------
    // Position Access
    //--------------------------------------------------------------------------

    /**
     * @brief Get point position
     * @return Position in sketch-local coordinates (mm)
     */
    const gp_Pnt2d& position() const { return m_position; }

    /**
     * @brief Get mutable reference to position
     * @return Mutable reference for solver direct binding
     *
     * IMPORTANT: This returns a mutable reference for PlaneGCS direct parameter binding.
     * Per SPECIFICATION.md §23.5, the solver holds pointers to these coordinates.
     */
    gp_Pnt2d& position() { return m_position; }

    /**
     * @brief Set point position
     * @param position New position in sketch coordinates (mm)
     */
    void setPosition(const gp_Pnt2d& position) { m_position = position; }

    /**
     * @brief Set position by coordinates
     * @param x X coordinate (mm)
     * @param y Y coordinate (mm)
     */
    void setPosition(double x, double y) { m_position.SetCoord(x, y); }

    /**
     * @brief Get X coordinate
     */
    double x() const { return m_position.X(); }

    /**
     * @brief Get Y coordinate
     */
    double y() const { return m_position.Y(); }

    //--------------------------------------------------------------------------
    // Connectivity (for rubber-band dragging per §5.13)
    //--------------------------------------------------------------------------

    /**
     * @brief Get list of entities connected to this point
     * @return Vector of entity IDs that use this point
     *
     * Used for rubber-band dragging behavior - when point moves,
     * connected edges stretch accordingly.
     */
    const std::vector<EntityID>& connectedEntities() const { return m_connectedEntities; }

    /**
     * @brief Add a connected entity reference
     * @param entityId ID of entity that uses this point
     *
     * No-op if entityId is already connected.
     */
    void addConnectedEntity(const EntityID& entityId);

    /**
     * @brief Remove a connected entity reference
     * @param entityId ID of entity to remove
     *
     * No-op if entityId is not connected.
     */
    void removeConnectedEntity(const EntityID& entityId);

    //--------------------------------------------------------------------------
    // SketchEntity Interface
    //--------------------------------------------------------------------------

    EntityType type() const override { return EntityType::Point; }
    std::string typeName() const override { return "Point"; }

    BoundingBox2d bounds() const override;
    bool isNear(const gp_Pnt2d& point, double tolerance) const override;
    int degreesOfFreedom() const override { return 2; }  // X, Y

    void serialize(QJsonObject& json) const override;
    bool deserialize(const QJsonObject& json) override;

    //--------------------------------------------------------------------------
    // Geometry Operations
    //--------------------------------------------------------------------------

    /**
     * @brief Calculate distance to another point
     * @param other Other point
     * @return Distance in mm
     */
    double distanceTo(const SketchPoint& other) const;

    /**
     * @brief Calculate distance to a raw point
     * @param point Point in sketch coordinates
     * @return Distance in mm
     */
    double distanceTo(const gp_Pnt2d& point) const;

    /**
     * @brief Check if coincident with another point
     * @param other Other point
     * @param tolerance Coincidence tolerance (default: COINCIDENCE_TOLERANCE)
     * @return true if within tolerance
     */
    bool coincidentWith(const SketchPoint& other,
                        double tolerance = constants::COINCIDENCE_TOLERANCE) const;

private:
    gp_Pnt2d m_position;
    std::vector<EntityID> m_connectedEntities;
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_POINT_H
