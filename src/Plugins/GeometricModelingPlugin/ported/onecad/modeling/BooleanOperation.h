#ifndef ONECAD_CORE_MODELING_BOOLEANOPERATION_H
#define ONECAD_CORE_MODELING_BOOLEANOPERATION_H

#include "../../app/document/OperationRecord.h"
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <optional>
#include <vector>

namespace onecad::core::modeling {

class BooleanOperation {
public:
    /**
     * @brief Performs a boolean operation between a tool shape and target shapes.
     * @param tool The shape being applied (e.g., the extrusion).
     * @param targets The list of shapes to modify (usually just one).
     * @param mode The boolean operation mode.
     * @return The resulting shape, or a null shape if the operation fails.
     *         For v1, we assume single target for Cut/Join/Intersect.
     */
    static TopoDS_Shape perform(const TopoDS_Shape& tool, 
                                const TopoDS_Shape& target, 
                                app::BooleanMode mode);

    /**
     * @brief Detects the most likely boolean mode from where the tool grows.
     *
     * Probe-point solid classification: a point one small step from the
     * profile in the tool's growth direction is classified against each
     * touching target — inside the target means material is being carved out
     * (Cut), outside means material is being added (Add). ON/UNKNOWN falls
     * back to the common-volume heuristic. Not touching any target: NewBody.
     *
     * @param tool       The shape being applied (e.g., the extrusion).
     * @param targets    The potential target shapes in the document.
     * @param probeStart A point strictly inside the profile (see
     *                   interiorPointOnFace).
     * @param probeStep  Displacement of the first material the tool creates:
     *                   extrude = direction * sign(distance) * ~1e-3 mm;
     *                   revolve = (probeStart rotated ~0.5deg about the axis
     *                   in the sweep direction) - probeStart.
     */
    static app::BooleanMode detectMode(const TopoDS_Shape& tool,
                                       const std::vector<TopoDS_Shape>& targets,
                                       const gp_Pnt& probeStart,
                                       const gp_Vec& probeStep);

    /**
     * @brief A point strictly inside a face (holes respected).
     *
     * Tries the UV-box center first, then an interior 7x7 UV grid; every
     * candidate is validated with BRepClass_FaceClassifier so annular or
     * L-shaped profiles never yield a point inside a hole.
     */
    static std::optional<gp_Pnt> interiorPointOnFace(const TopoDS_Face& face);
};

} // namespace onecad::core::modeling

#endif // ONECAD_CORE_MODELING_BOOLEANOPERATION_H
