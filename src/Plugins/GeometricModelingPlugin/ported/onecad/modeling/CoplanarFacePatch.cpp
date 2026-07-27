#include "CoplanarFacePatch.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <deque>
#include <utility>

namespace onecad::core::modeling {
namespace {

bool planarFaceData(const TopoDS_Face& face, gp_Pln& planeOut, gp_Dir& normalOut) {
    try {
        if (face.IsNull()) {
            return false;
        }
        BRepAdaptor_Surface surface(face, true);
        if (surface.GetType() != GeomAbs_Plane) {
            return false;
        }
        planeOut = surface.Plane();
        normalOut = planeOut.Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) {
            normalOut.Reverse();
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool isCoplanarWithSeed(const TopoDS_Face& candidate,
                        const gp_Pnt& seedOrigin,
                        const gp_Dir& seedNormal,
                        double normalDotTolerance,
                        double planeDistanceTolerance,
                        bool& needsReverseOut) {
    gp_Pln candidatePlane;
    gp_Dir candidateNormal;
    if (!planarFaceData(candidate, candidatePlane, candidateNormal)) {
        return false;
    }

    const double dot = seedNormal.Dot(candidateNormal);
    if (std::abs(dot) < normalDotTolerance) {
        return false;
    }

    const gp_Vec delta(seedOrigin, candidatePlane.Location());
    const double planeDistance = std::abs(seedNormal.Dot(delta));
    if (planeDistance > planeDistanceTolerance) {
        return false;
    }

    needsReverseOut = dot < 0.0;
    return true;
}

double sanitizeTolerance(double value, double fallback) {
    return value > 0.0 ? value : fallback;
}

} // namespace

std::vector<TopoDS_Face> CoplanarFacePatch::collectConnectedFaces(const TopoDS_Shape& body,
                                                                  const TopoDS_Face& seedFace,
                                                                  const Options& options) {
    std::vector<TopoDS_Face> patch;
    if (seedFace.IsNull()) {
        return patch;
    }

    gp_Pln seedPlane;
    gp_Dir seedNormal;
    if (!planarFaceData(seedFace, seedPlane, seedNormal)) {
        patch.push_back(seedFace);
        return patch;
    }

    if (body.IsNull()) {
        patch.push_back(seedFace);
        return patch;
    }

    const double normalDotTolerance = sanitizeTolerance(options.normalDotTolerance, 0.9999);
    const double planeDistanceTolerance = sanitizeTolerance(options.planeDistanceTolerance, 1e-3);

    try {
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
        TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

        TopTools_MapOfShape visited;
        std::deque<TopoDS_Face> frontier;

        frontier.push_back(seedFace);
        visited.Add(seedFace);

        while (!frontier.empty()) {
            TopoDS_Face current = frontier.front();
            frontier.pop_front();
            patch.push_back(current);

            for (TopExp_Explorer edgeExplorer(current, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
                const TopoDS_Shape& edge = edgeExplorer.Current();
                const int edgeIndex = edgeFaceMap.FindIndex(edge);
                if (edgeIndex <= 0) {
                    continue;
                }

                const TopTools_ListOfShape& adjacentFaces = edgeFaceMap(edgeIndex);
                for (TopTools_ListIteratorOfListOfShape it(adjacentFaces); it.More(); it.Next()) {
                    TopoDS_Face neighbor = TopoDS::Face(it.Value());
                    if (neighbor.IsNull() || visited.Contains(neighbor)) {
                        continue;
                    }

                    bool needsReverse = false;
                    if (!isCoplanarWithSeed(neighbor,
                                            seedPlane.Location(),
                                            seedNormal,
                                            normalDotTolerance,
                                            planeDistanceTolerance,
                                            needsReverse)) {
                        continue;
                    }

                    if (needsReverse) {
                        neighbor.Reverse();
                    }
                    visited.Add(neighbor);
                    frontier.push_back(neighbor);
                }
            }
        }
    } catch (...) {
        patch.clear();
    }

    if (patch.empty()) {
        patch.push_back(seedFace);
    }
    return patch;
}

std::vector<TopoDS_Face> CoplanarFacePatch::collectConnectedFaces(const TopoDS_Shape& body,
                                                                  const TopoDS_Face& seedFace) {
    return collectConnectedFaces(body, seedFace, Options{});
}

TopoDS_Shape CoplanarFacePatch::makeFaceCompound(const std::vector<TopoDS_Face>& faces) {
    if (faces.empty()) {
        return {};
    }
    if (faces.size() == 1) {
        return faces.front();
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const TopoDS_Face& face : faces) {
        if (!face.IsNull()) {
            builder.Add(compound, face);
        }
    }
    return compound;
}

} // namespace onecad::core::modeling
