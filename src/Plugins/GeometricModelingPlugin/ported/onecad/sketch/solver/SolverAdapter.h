/**
 * @file SolverAdapter.h
 * @brief Converts Sketch entities/constraints into solver inputs
 *
 * Per SPECIFICATION.md §23.5: isolates Sketch -> PlaneGCS translation.
 */
#ifndef GEOMETRICMODELINGPLUGIN_SOLVERADAPTER_H
#define GEOMETRICMODELINGPLUGIN_SOLVERADAPTER_H

#include "../SketchTypes.h"

namespace onecad::core::sketch
{
class Sketch;
class SketchConstraint;
class ConstraintSolver;

/**
 * @brief Sketch -> solver translation helpers
 */
class SolverAdapter
{
public:
	/**
     * @brief Populate solver with all entities and constraints from a sketch
     * @return true if every constraint was translated
     */
	static bool populateSolver(Sketch& sketch, ConstraintSolver& solver);

	/**
     * @brief Add a single constraint to the solver
     */
	static bool addConstraintToSolver(SketchConstraint* constraint, ConstraintSolver& solver);
};

} // namespace onecad::core::sketch

#endif // GEOMETRICMODELINGPLUGIN_SOLVERADAPTER_H
