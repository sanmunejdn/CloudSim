/**
 * @file SketchTypes.h
 * @brief Core type definitions for the OneCAD sketch system
 *
 * This file contains fundamental types, enums, and forward declarations
 * used throughout the sketch module.
 */

#ifndef ONECAD_CORE_SKETCH_TYPES_H
#define ONECAD_CORE_SKETCH_TYPES_H

#include <string>

namespace onecad::core::sketch {

//==============================================================================
// Entity Types
//==============================================================================

/**
 * @brief Enumeration of sketch entity types
 */
enum class EntityType {
    Point,
    Line,
    Arc,
    Circle,
    Ellipse,
    Spline  // Future: v2
};

/**
 * @brief Enumeration of constraint types
 */
enum class ConstraintType {
    // Positional constraints
    Coincident,
    Horizontal,
    Vertical,
    Fixed,
    Midpoint,
    OnCurve,

    // Relational constraints
    Parallel,
    Perpendicular,
    Tangent,
    Concentric,
    Equal,

    // Dimensional constraints
    Distance,
    HorizontalDistance,
    VerticalDistance,
    Angle,
    Radius,
    Diameter,

    // Symmetry
    Symmetric
};

/**
 * @brief Position specification for PointOnCurve constraints
 */
enum class CurvePosition {
    Start,      // Constrain to arc/curve start point
    End,        // Constrain to arc/curve end point
    Arbitrary   // Constrain to arbitrary point on curve
};

/**
 * @brief Constraint state indicating solve status
 */
enum class ConstraintState {
    UnderConstrained,   // DOF > 0 (Blue in UI)
    FullyConstrained,   // DOF = 0 (Green in UI)
    OverConstrained,    // Redundant constraints (Orange in UI)
    Conflicting         // Cannot be solved (Red in UI)
};

//==============================================================================
// Type Aliases
//==============================================================================

/**
 * @brief Entity identifier - UUID string
 */
using EntityID = std::string;

/**
 * @brief Constraint identifier - UUID string
 */
using ConstraintID = std::string;

/**
 * @brief Point identifier (alias for EntityID)
 */
using PointID = EntityID;

//==============================================================================
// Basic Geometry Types
//==============================================================================

/**
 * @brief Simple 2D vector type for sketch-space math
 */
struct Vec2d {
    double x = 0.0;
    double y = 0.0;
};

/**
 * @brief Simple 3D vector type for world-space math
 */
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

//==============================================================================
// Constants
//==============================================================================

namespace constants {

/// Geometric tolerance for coincidence checks (mm)
constexpr double COINCIDENCE_TOLERANCE = 1e-6;

/// Solver convergence tolerance (mm)
constexpr double SOLVER_TOLERANCE = 1e-4;

/// Maximum solver iterations
constexpr int MAX_SOLVER_ITERATIONS = 100;

/// Minimum geometry size (mm) - prevents degenerate geometry
constexpr double MIN_GEOMETRY_SIZE = 0.01;

/// Snap radius in world coordinates (mm)
constexpr double SNAP_RADIUS_MM = 2.0;

/// Entity selection tolerance (pixels)
constexpr int PICK_TOLERANCE_PIXELS = 10;

/// Loop detection debounce delay (ms)
constexpr int LOOP_DETECTION_DEBOUNCE_MS = 200;

/// Background solver threshold (entity count)
constexpr int BACKGROUND_SOLVER_THRESHOLD = 100;

/// Solver timeout (ms)
constexpr int SOLVER_TIMEOUT_MS = 5000;

} // namespace constants

//==============================================================================
// Forward Declarations
//==============================================================================

class SketchEntity;
class SketchPoint;
class SketchLine;
class SketchArc;
class SketchCircle;
class SketchConstraint;
class Sketch;
class ConstraintSolver;
class LoopDetector;

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_TYPES_H
