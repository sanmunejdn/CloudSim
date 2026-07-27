#include "BooleanOperation.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <gp_Pnt2d.hxx>

namespace onecad::core::modeling {

TopoDS_Shape BooleanOperation::perform(const TopoDS_Shape& tool, 
                                       const TopoDS_Shape& target, 
                                       app::BooleanMode mode) {
    if (tool.IsNull() || target.IsNull()) {
        return TopoDS_Shape();
    }

    switch (mode) {
        case app::BooleanMode::Add: {
            BRepAlgoAPI_Fuse fuse(target, tool);
            fuse.Build();
            if (fuse.IsDone()) {
                return fuse.Shape();
            }
            break;
        }
        case app::BooleanMode::Cut: {
            BRepAlgoAPI_Cut cut(target, tool);
            cut.Build();
            if (cut.IsDone()) {
                return cut.Shape();
            }
            break;
        }
        case app::BooleanMode::Intersect: {
            BRepAlgoAPI_Common common(target, tool);
            common.Build();
            if (common.IsDone()) {
                return common.Shape();
            }
            break;
        }
        default:
            break;
    }
    return TopoDS_Shape();
}

app::BooleanMode BooleanOperation::detectMode(const TopoDS_Shape& tool,
                                              const std::vector<TopoDS_Shape>& targets,
                                              const gp_Pnt& probeStart,
                                              const gp_Vec& probeStep) {
    // Where does the tool's FIRST material live relative to the target?
    // Inside the target solid -> the user is carving (Cut); outside -> the
    // user is growing (Add). The previous overlap-only heuristic returned Cut
    // for any volume overlap, misclassifying pull-out extrudes whose tool
    // merely re-covers existing material.
    const gp_Pnt probe = probeStart.Translated(probeStep);

    for (const auto& target : targets) {
        if (target.IsNull()) {
            continue;
        }
        BRepExtrema_DistShapeShape dist(tool, target);
        if (!dist.IsDone() || dist.Value() > 1e-6) {
            continue;  // Not touching this target
        }

        BRepClass3d_SolidClassifier classifier(target);
        classifier.Perform(probe, 1e-7);
        if (classifier.State() == TopAbs_IN) {
            return app::BooleanMode::Cut;
        }
        if (classifier.State() == TopAbs_OUT) {
            return app::BooleanMode::Add;
        }

        // ON/UNKNOWN (probe landed on the boundary): common-volume fallback.
        BRepAlgoAPI_Common common(target, tool);
        common.Build();
        if (common.IsDone() && !common.Shape().IsNull()) {
            GProp_GProps props;
            BRepGProp::VolumeProperties(common.Shape(), props);
            if (props.Mass() > 1e-6) {
                return app::BooleanMode::Cut;
            }
        }
        return app::BooleanMode::Add;
    }

    return app::BooleanMode::NewBody;
}

std::optional<gp_Pnt> BooleanOperation::interiorPointOnFace(const TopoDS_Face& face) {
    if (face.IsNull()) {
        return std::nullopt;
    }
    Standard_Real umin = 0.0;
    Standard_Real umax = 0.0;
    Standard_Real vmin = 0.0;
    Standard_Real vmax = 0.0;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    BRepAdaptor_Surface surface(face);

    const auto tryUv = [&](double u, double v) -> std::optional<gp_Pnt> {
        BRepClass_FaceClassifier classifier(face, gp_Pnt2d(u, v), 1e-7);
        if (classifier.State() == TopAbs_IN) {
            return surface.Value(u, v);
        }
        return std::nullopt;
    };

    if (auto p = tryUv((umin + umax) * 0.5, (vmin + vmax) * 0.5)) {
        return p;
    }
    // Interior 7x7 grid handles annular / L-shaped profiles whose UV center
    // falls in a hole.
    for (int i = 1; i <= 7; ++i) {
        for (int j = 1; j <= 7; ++j) {
            const double u = umin + (umax - umin) * i / 8.0;
            const double v = vmin + (vmax - vmin) * j / 8.0;
            if (auto p = tryUv(u, v)) {
                return p;
            }
        }
    }
    return std::nullopt;
}

} // namespace onecad::core::modeling
