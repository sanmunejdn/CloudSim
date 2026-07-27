#include "SolverAdapter.h"

#include "../Sketch.h"
#include "ConstraintSolver.h"

#include <QLoggingCategory>
#include <QString>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSolverAdapter, "onecad.core.constraintsolver.adapter")

bool SolverAdapter::populateSolver(Sketch& sketch, ConstraintSolver& solver) {
    solver.clear();

    for (const auto& entity : sketch.getAllEntities()) {
        if (entity && entity->type() == EntityType::Point) {
            solver.addPoint(dynamic_cast<SketchPoint*>(entity.get()));
        }
    }

    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity) {
            continue;
        }

        switch (entity->type()) {
            case EntityType::Line:
                solver.addLine(dynamic_cast<SketchLine*>(entity.get()));
                break;
            case EntityType::Arc:
                solver.addArc(dynamic_cast<SketchArc*>(entity.get()));
                break;
            case EntityType::Circle:
                solver.addCircle(dynamic_cast<SketchCircle*>(entity.get()));
                break;
            default:
                break;
        }
    }

    bool ok = true;
    for (const auto& constraint : sketch.getAllConstraints()) {
        if (!addConstraintToSolver(constraint.get(), solver)) {
            ok = false;
            qCWarning(logSolverAdapter) << "populateSolver:constraint-translation-failed"
                                       << (constraint ? QString::fromStdString(constraint->id()) : QStringLiteral("<null>"));
        }
    }

    return ok;
}

bool SolverAdapter::addConstraintToSolver(SketchConstraint* constraint, ConstraintSolver& solver) {
    if (!constraint) {
        return false;
    }
    return solver.addConstraint(constraint);
}

} // namespace onecad::core::sketch
