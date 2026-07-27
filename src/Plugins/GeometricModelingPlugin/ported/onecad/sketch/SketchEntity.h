/**
 * @file SketchEntity.h
 * @brief Base class for all sketch geometry entities
 *
 * All sketch geometry (points, lines, arcs, circles) inherit from this base class.
 * Provides common interface for identification, construction mode, and serialization.
 */

#ifndef ONECAD_CORE_SKETCH_ENTITY_H
#define ONECAD_CORE_SKETCH_ENTITY_H

#include "SketchTypes.h"

#include <gp_Pnt2d.hxx>
#include <QJsonObject>

#include <limits>
#include <string>

namespace onecad::core::sketch {

/**
 * @brief 2D bounding box for sketch entities
 */
struct BoundingBox2d {
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    bool isEmpty() const {
        return minX > maxX || minY > maxY;
    }

    bool contains(const gp_Pnt2d& point) const {
        if (isEmpty()) {
            return false;
        }
        return point.X() >= minX && point.X() <= maxX &&
               point.Y() >= minY && point.Y() <= maxY;
    }

    bool intersects(const BoundingBox2d& other) const {
        if (isEmpty() || other.isEmpty()) {
            return false;
        }
        return !(other.maxX < minX || other.minX > maxX ||
                 other.maxY < minY || other.minY > maxY);
    }

    double width() const { return isEmpty() ? 0.0 : (maxX - minX); }
    double height() const { return isEmpty() ? 0.0 : (maxY - minY); }

    gp_Pnt2d center() const {
        if (isEmpty()) {
            return gp_Pnt2d(0.0, 0.0);
        }
        return gp_Pnt2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
    }
};

/**
 * @brief Abstract base class for all sketch geometry entities
 *
 * Provides:
 * - Unique identification via UUID
 * - Construction geometry flag (per SPECIFICATION.md §5.8)
 * - Bounding box calculation for rendering optimization
 * - Serialization interface for file I/O
 */
class SketchEntity {
public:
    virtual ~SketchEntity() = default;
    SketchEntity(const SketchEntity&) = delete;
    SketchEntity& operator=(const SketchEntity&) = delete;
    SketchEntity(SketchEntity&&) noexcept = default;
    SketchEntity& operator=(SketchEntity&&) noexcept = default;

    //--------------------------------------------------------------------------
    // Identification
    //--------------------------------------------------------------------------

    /**
     * @brief Get the unique identifier for this entity
     */
    EntityID id() const { return m_id; }

    /**
     * @brief Get the type of this entity
     */
    virtual EntityType type() const = 0;

    /**
     * @brief Get human-readable type name
     */
    virtual std::string typeName() const = 0;

    //--------------------------------------------------------------------------
    // Construction Mode (per SPECIFICATION.md §5.8)
    //--------------------------------------------------------------------------

    /**
     * @brief Check if this is construction geometry
     * @return true if construction (dashed, reference only)
     *
     * Per SPECIFICATION.md §6.1: All geometry starts as construction by default.
     * Construction geometry cannot form faces but can be used for constraints.
     */
    bool isConstruction() const { return m_isConstruction; }

    /**
     * @brief Set construction mode
     * @param value true for construction geometry
     */
    void setConstruction(bool value) { m_isConstruction = value; }

    /**
     * @brief Check whether this entity is a locked host-face reference.
     *
     * Locked reference entities are selectable, but sketch edit operations must not
     * modify or delete them.
     */
    bool isReferenceLocked() const { return m_isReferenceLocked; }

    /**
     * @brief Set locked-reference state.
     */
    void setReferenceLocked(bool value) { m_isReferenceLocked = value; }

    //--------------------------------------------------------------------------
    // Geometry Interface
    //--------------------------------------------------------------------------

    /**
     * @brief Calculate bounding box for this entity
     * @return 2D bounding box in sketch-local coordinates
     *
     * Used for:
     * - Viewport culling optimization
     * - Selection hit testing
     * - Loop detection spatial queries
     */
    virtual BoundingBox2d bounds() const = 0;

    /**
     * @brief Check if a point is near this entity (for selection)
     * @param point Point in sketch coordinates
     * @param tolerance Distance tolerance in sketch units (mm)
     * @return true if point is within tolerance of entity
     */
    virtual bool isNear(const gp_Pnt2d& point, double tolerance) const = 0;

    /**
     * @brief Get the degrees of freedom this entity contributes
     * @return DOF count (e.g., Point=2, Line=0 additional, Arc=3)
     *
     * Note: Lines contribute 0 DOF because their DOF comes from endpoints.
     */
    virtual int degreesOfFreedom() const = 0;

    //--------------------------------------------------------------------------
    // Serialization (per SPECIFICATION.md §17.3)
    //--------------------------------------------------------------------------

    /**
     * @brief Serialize entity to JSON
     * @param json Output JSON object
     */
    virtual void serialize(QJsonObject& json) const = 0;

    /**
     * @brief Deserialize entity from JSON
     * @param json Input JSON object
     * @return true if successful
     *
     * Implementations must validate input types and return false without
     * modifying observable state when deserialization fails.
     */
    virtual bool deserialize(const QJsonObject& json) = 0;

protected:
    /**
     * @brief Protected constructor - only derived classes can instantiate
     */
    SketchEntity();

    /**
     * @brief Protected constructor with specific ID (for deserialization)
     */
    explicit SketchEntity(const EntityID& id);

    /**
     * @brief Generate a new UUID for entity identification
     */
    static EntityID generateId();

    EntityID m_id;
    bool m_isConstruction = true;  // Default: construction (per SPECIFICATION.md §6.1)
    bool m_isReferenceLocked = false;
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_ENTITY_H
