#include "detail/FeatureDiscretizeInternal.h"
#include "FeatureDiscretizerRegistry.h"

#include "detail/OccIncludes.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
#include <vector>

namespace geoalgo
{
namespace
{

constexpr double kVertexMergeTol = 1e-3;

struct QuantizedVertex
{
	long long x = 0;
	long long y = 0;
	long long z = 0;

	bool operator==(const QuantizedVertex& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
};

struct QuantizedVertexHash
{
	std::size_t operator()(const QuantizedVertex& v) const
	{
		const std::size_t hx = std::hash<long long>{}(v.x);
		const std::size_t hy = std::hash<long long>{}(v.y);
		const std::size_t hz = std::hash<long long>{}(v.z);
		return hx ^ (hy << 1) ^ (hz << 2);
	}
};

QuantizedVertex quantizeVertex(const gp_Pnt& p)
{
	const double inv = 1.0 / kVertexMergeTol;
	return QuantizedVertex{
		static_cast<long long>(std::llround(p.X() * inv)),
		static_cast<long long>(std::llround(p.Y() * inv)),
		static_cast<long long>(std::llround(p.Z() * inv))};
}

class UnionFind
{
public:
	int find(int x)
	{
		if (parent[static_cast<std::size_t>(x)] != x)
		{
			parent[static_cast<std::size_t>(x)] = find(parent[static_cast<std::size_t>(x)]);
		}
		return parent[static_cast<std::size_t>(x)];
	}

	void unite(int a, int b)
	{
		a = find(a);
		b = find(b);
		if (a != b)
		{
			parent[static_cast<std::size_t>(b)] = a;
		}
	}

	void addNode()
	{
		parent.push_back(static_cast<int>(parent.size()));
	}

private:
	std::vector<int> parent;
};

FeatureDiscretizeInput makeInputFromEntry(
	const FeatureListDocument& doc,
	const FeatureEntry& entry,
	const FeatureGeometry& geometryOverride)
{
	FeatureDiscretizeInput input{};
	input.workpiece = doc.workpiece;
	input.geometry = geometryOverride;
	input.params = entry.params;
	input.strategyId = entry.strategyId;
	input.featureId = entry.featureId;
	return input;
}

bool discretizeWithStrategy(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg)
{
	const IFeatureDiscretizer* discretizer = FeatureDiscretizerRegistry::instance().get(input.strategyId);
	if (!discretizer)
	{
		if (errMsg)
		{
			*errMsg = "unknown strategyId: " + input.strategyId;
		}
		return false;
	}
	if (!discretizer->validate(input, errMsg))
	{
		return false;
	}
	RawPath part;
	part.sourceFeatureId = input.featureId;
	if (!discretizer->discretize(shape, input, part, errMsg))
	{
		return false;
	}
	detail::appendRawPath(part, out);
	return true;
}

std::vector<std::vector<int>> connectedEdgeComponents(
	const TopoDS_Shape& shape,
	const std::vector<int>& edgeIndices)
{
	std::vector<int> uniqueEdges = edgeIndices;
	std::sort(uniqueEdges.begin(), uniqueEdges.end());
	uniqueEdges.erase(std::unique(uniqueEdges.begin(), uniqueEdges.end()), uniqueEdges.end());

	UnionFind uf;
	std::unordered_map<QuantizedVertex, int, QuantizedVertexHash> vertexToNode;
	std::vector<std::vector<QuantizedVertex>> edgeVertices;
	edgeVertices.resize(uniqueEdges.size());

	for (std::size_t i = 0; i < uniqueEdges.size(); ++i)
	{
		uf.addNode();
		TopoDS_Edge edge;
		std::string localErr;
		if (!shapeEdgeAtIndex(shape, uniqueEdges[static_cast<std::size_t>(i)], edge, &localErr))
		{
			continue;
		}
		TopTools_IndexedMapOfShape verts;
		TopExp::MapShapes(edge, TopAbs_VERTEX, verts);
		for (int v = 1; v <= verts.Extent(); ++v)
		{
			const TopoDS_Vertex vertex = TopoDS::Vertex(verts.FindKey(v));
			const gp_Pnt p = BRep_Tool::Pnt(vertex);
			const QuantizedVertex q = quantizeVertex(p);
			edgeVertices[i].push_back(q);
			const auto it = vertexToNode.find(q);
			if (it == vertexToNode.end())
			{
				vertexToNode[q] = static_cast<int>(i);
			}
			else
			{
				uf.unite(static_cast<int>(i), it->second);
			}
		}
	}

	std::map<int, std::vector<int>> components;
	for (std::size_t i = 0; i < uniqueEdges.size(); ++i)
	{
		const int root = uf.find(static_cast<int>(i));
		components[root].push_back(uniqueEdges[i]);
	}

	std::vector<std::vector<int>> out;
	out.reserve(components.size());
	for (auto& kv : components)
	{
		out.push_back(std::move(kv.second));
	}
	return out;
}

std::vector<int> unionFaceIndices(const std::vector<FeatureEntry>& entries)
{
	std::vector<int> faces;
	for (const FeatureEntry& entry : entries)
	{
		faces.insert(faces.end(), entry.geometry.faceIndices.begin(), entry.geometry.faceIndices.end());
	}
	std::sort(faces.begin(), faces.end());
	faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
	return faces;
}

std::vector<int> unionEdgeIndices(const std::vector<FeatureEntry>& entries)
{
	std::vector<int> edges;
	for (const FeatureEntry& entry : entries)
	{
		edges.insert(edges.end(), entry.geometry.edgeIndices.begin(), entry.geometry.edgeIndices.end());
	}
	std::sort(edges.begin(), edges.end());
	edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
	return edges;
}

bool processLineConnectivityGroup(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	const std::vector<FeatureEntry>& entries,
	RawPath& out,
	std::string* errMsg)
{
	if (entries.empty())
	{
		return true;
	}
	const std::vector<int> allEdges = unionEdgeIndices(entries);
	const std::vector<std::vector<int>> components = connectedEdgeComponents(shape, allEdges);
	for (const std::vector<int>& component : components)
	{
		const FeatureEntry& templateEntry = entries.front();
		FeatureGeometry geometry{};
		geometry.edgeIndices = component;
		const FeatureDiscretizeInput input = makeInputFromEntry(doc, templateEntry, geometry);
		if (!discretizeWithStrategy(shape, input, out, errMsg))
		{
			return false;
		}
	}
	return true;
}

bool processFaceUnionGroup(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	const std::vector<FeatureEntry>& entries,
	RawPath& out,
	std::string* errMsg)
{
	if (entries.empty())
	{
		return true;
	}
	const FeatureEntry& templateEntry = entries.front();
	FeatureGeometry geometry{};
	geometry.faceIndices = unionFaceIndices(entries);
	const FeatureDiscretizeInput input = makeInputFromEntry(doc, templateEntry, geometry);
	return discretizeWithStrategy(shape, input, out, errMsg);
}

bool processNonePolicyGroup(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	const std::vector<FeatureEntry>& entries,
	RawPath& out,
	std::string* errMsg)
{
	for (const FeatureEntry& entry : entries)
	{
		const FeatureDiscretizeInput input = makeInputFromEntry(doc, entry, entry.geometry);
		if (!discretizeWithStrategy(shape, input, out, errMsg))
		{
			return false;
		}
	}
	return true;
}

bool processStrategyGroup(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	const std::string& strategyId,
	const std::vector<FeatureEntry>& entries,
	RawPath& out,
	std::string* errMsg)
{
	const IFeatureDiscretizer* discretizer = FeatureDiscretizerRegistry::instance().get(strategyId);
	if (!discretizer)
	{
		if (errMsg)
		{
			*errMsg = "unknown strategyId: " + strategyId;
		}
		return false;
	}
	switch (discretizer->mergePolicy())
	{
	case MergePolicy::LineConnectivity:
		return processLineConnectivityGroup(doc, shape, entries, out, errMsg);
	case MergePolicy::FaceUnion:
		return processFaceUnionGroup(doc, shape, entries, out, errMsg);
	case MergePolicy::None:
	default:
		return processNonePolicyGroup(doc, shape, entries, out, errMsg);
	}
}

} // namespace

bool discretizeFeatureListInternal(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	RawPath& out,
	std::string* errMsg)
{
	out = RawPath{};
	if (doc.features.empty())
	{
		if (errMsg)
		{
			*errMsg = "feature list is empty";
		}
		return false;
	}

	std::map<std::string, std::vector<FeatureEntry>> byStrategy;
	for (const FeatureEntry& entry : doc.features)
	{
		byStrategy[entry.strategyId].push_back(entry);
	}

	for (auto& kv : byStrategy)
	{
		if (!processStrategyGroup(doc, shape, kv.first, kv.second, out, errMsg))
		{
			return false;
		}
	}

	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "discretization produced no points";
		}
		return false;
	}
	return true;
}

} // namespace geoalgo
