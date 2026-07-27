#include "EdgeChainer.h"

#include <BRepAdaptor_Curve.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <BRep_Tool.hxx>
#include <unordered_set>
#include <queue>
#include <cmath>

namespace onecad::core::modeling {

namespace {

// Hash for TopoDS_Edge using its TShape pointer
struct EdgeHash {
    size_t operator()(const TopoDS_Edge& e) const {
        return std::hash<void*>()(e.TShape().get());
    }
};

struct EdgeEqual {
    bool operator()(const TopoDS_Edge& a, const TopoDS_Edge& b) const {
        return a.IsSame(b);
    }
};

} // namespace

EdgeChainer::ChainResult EdgeChainer::buildChain(
    const TopoDS_Edge& seed,
    const TopoDS_Shape& body,
    double tangentTolerance)
{
    ChainResult result;
    if (seed.IsNull() || body.IsNull()) {
        return result;
    }

    // Build vertex-to-edge adjacency map
    TopTools_IndexedDataMapOfShapeListOfShape vertexEdgeMap;
    TopExp::MapShapesAndAncestors(body, TopAbs_VERTEX, TopAbs_EDGE, vertexEdgeMap);

    std::unordered_set<TopoDS_Edge, EdgeHash, EdgeEqual> visited;
    std::vector<TopoDS_Edge> chain;

    // BFS from seed edge in both directions
    std::queue<TopoDS_Edge> frontier;
    frontier.push(seed);
    visited.insert(seed);
    chain.push_back(seed);

    while (!frontier.empty()) {
        TopoDS_Edge current = frontier.front();
        frontier.pop();

        // Get vertices of current edge
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(current, v1, v2);

        // Check both vertices for tangent-continuous neighbors
        for (const auto& vertex : {v1, v2}) {
            if (vertex.IsNull()) continue;

            int vIndex = vertexEdgeMap.FindIndex(vertex);
            if (vIndex == 0) continue;

            const TopTools_ListOfShape& adjacentList = vertexEdgeMap(vIndex);
            for (TopTools_ListIteratorOfListOfShape it(adjacentList); it.More(); it.Next()) {
                TopoDS_Edge neighbor = TopoDS::Edge(it.Value());

                if (neighbor.IsNull() || neighbor.IsSame(current)) continue;
                if (visited.count(neighbor) > 0) continue;

                // Check tangent continuity
                if (areTangentContinuous(current, neighbor, tangentTolerance)) {
                    visited.insert(neighbor);
                    chain.push_back(neighbor);
                    frontier.push(neighbor);
                }
            }
        }
    }

    result.edges = std::move(chain);

    // Check if chain is closed (first and last edges share a vertex)
    if (result.edges.size() > 1) {
        TopoDS_Vertex shared = findSharedVertex(result.edges.front(), result.edges.back());
        result.isClosed = !shared.IsNull();
    }

    return result;
}

bool EdgeChainer::areTangentContinuous(
    const TopoDS_Edge& e1,
    const TopoDS_Edge& e2,
    double tolerance)
{
    if (e1.IsNull() || e2.IsNull()) return false;

    TopoDS_Vertex shared = findSharedVertex(e1, e2);
    if (shared.IsNull()) return false;

    try {
        gp_Dir tan1 = tangentAtVertex(e1, shared);
        gp_Dir tan2 = tangentAtVertex(e2, shared);

        // Tangents should be either parallel or anti-parallel
        double dot = std::abs(tan1.Dot(tan2));
        return dot >= tolerance;
    } catch (...) {
        return false;
    }
}

std::vector<TopoDS_Edge> EdgeChainer::adjacentEdges(
    const TopoDS_Edge& edge,
    const TopoDS_Shape& body)
{
    std::vector<TopoDS_Edge> result;
    if (edge.IsNull() || body.IsNull()) return result;

    TopTools_IndexedDataMapOfShapeListOfShape vertexEdgeMap;
    TopExp::MapShapesAndAncestors(body, TopAbs_VERTEX, TopAbs_EDGE, vertexEdgeMap);

    TopoDS_Vertex v1, v2;
    TopExp::Vertices(edge, v1, v2);

    std::unordered_set<TopoDS_Edge, EdgeHash, EdgeEqual> seen;
    seen.insert(edge);

    for (const auto& vertex : {v1, v2}) {
        if (vertex.IsNull()) continue;

        int vIndex = vertexEdgeMap.FindIndex(vertex);
        if (vIndex == 0) continue;

        const TopTools_ListOfShape& adjacentList = vertexEdgeMap(vIndex);
        for (TopTools_ListIteratorOfListOfShape it(adjacentList); it.More(); it.Next()) {
            TopoDS_Edge neighbor = TopoDS::Edge(it.Value());
            if (!neighbor.IsNull() && seen.count(neighbor) == 0) {
                seen.insert(neighbor);
                result.push_back(neighbor);
            }
        }
    }

    return result;
}

gp_Dir EdgeChainer::tangentAtVertex(
    const TopoDS_Edge& edge,
    const TopoDS_Vertex& vertex)
{
    BRepAdaptor_Curve curve(edge);
    double first = curve.FirstParameter();
    double last = curve.LastParameter();

    // Determine which end the vertex is at
    gp_Pnt vPnt = BRep_Tool::Pnt(vertex);
    gp_Pnt pFirst = curve.Value(first);
    gp_Pnt pLast = curve.Value(last);

    double distFirst = vPnt.SquareDistance(pFirst);
    double distLast = vPnt.SquareDistance(pLast);

    gp_Pnt p;
    gp_Vec tangent;

    if (distFirst < distLast) {
        // Vertex is at first parameter
        curve.D1(first, p, tangent);
        // Tangent points INTO the edge, flip to point OUT from vertex
        tangent.Reverse();
    } else {
        // Vertex is at last parameter
        curve.D1(last, p, tangent);
        // Tangent already points OUT from edge
    }

    if (tangent.Magnitude() < 1e-10) {
        // Degenerate tangent, try midpoint direction
        gp_Pnt midPt = curve.Value((first + last) / 2.0);
        tangent = gp_Vec(vPnt, midPt);
    }

    return gp_Dir(tangent);
}

TopoDS_Vertex EdgeChainer::findSharedVertex(
    const TopoDS_Edge& e1,
    const TopoDS_Edge& e2)
{
    if (e1.IsNull() || e2.IsNull()) return TopoDS_Vertex();

    TopoDS_Vertex v1a, v1b, v2a, v2b;
    TopExp::Vertices(e1, v1a, v1b);
    TopExp::Vertices(e2, v2a, v2b);

    if (!v1a.IsNull() && (v1a.IsSame(v2a) || v1a.IsSame(v2b))) return v1a;
    if (!v1b.IsNull() && (v1b.IsSame(v2a) || v1b.IsSame(v2b))) return v1b;

    return TopoDS_Vertex();
}

} // namespace onecad::core::modeling
