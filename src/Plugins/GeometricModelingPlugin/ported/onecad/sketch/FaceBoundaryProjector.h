/**
 * @file FaceBoundaryProjector.h
 * @brief Projects planar host face boundary geometry onto a sketch.
 */
#ifndef ONECAD_CORE_SKETCH_FACEBOUNDARYPROJECTOR_H
#define ONECAD_CORE_SKETCH_FACEBOUNDARYPROJECTOR_H

#include "Sketch.h"

#include <TopoDS_Face.hxx>

#include <string>

namespace onecad::core::sketch {

class FaceBoundaryProjector {
public:
    struct Options {
        bool preferExactPrimitives = true;
        bool lockInsertedBoundaryPoints = true;
        double pointMergeTolerance = 1e-5;
        double radiusTolerance = 1e-5;
        int fallbackSegmentsPerCurve = 24;
    };

    struct ProjectionResult {
        bool success = false;
        int insertedPoints = 0;
        int insertedCurves = 0;
        int insertedFixedConstraints = 0;
        bool hasClosedBoundary = false;
        std::string errorMessage;
    };

    static ProjectionResult projectFaceBoundaries(const TopoDS_Face& face,
                                                  Sketch& sketch,
                                                  const Options& options);
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_FACEBOUNDARYPROJECTOR_H
