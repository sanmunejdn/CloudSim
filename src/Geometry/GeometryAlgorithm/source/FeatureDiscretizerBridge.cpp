/// @file FeatureDiscretizerBridge.cpp
/// @brief FeatureDiscretizerBridge 实现

#include "FeatureDiscretizerBridge.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeInternal.h"

#include <cassert>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
void setFeatureDiscretizerRegistry(FeatureDiscretizerRegistry* registry)
{
	assert(registry != nullptr);
	FeatureDiscretizerRegistry::setProcessInstance(registry);
}

FeatureDiscretizerRegistry& featureDiscretizerRegistry()
{
	return FeatureDiscretizerRegistry::instance();
}

FeatureDiscretizerConfigRegistry& featureDiscretizerConfigRegistry()
{
	return FeatureDiscretizerConfigRegistry::instance();
}

bool ensureFeatureDiscretizerConfigsLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerConfigRegistry::instance().ensureLoaded(resourceBaseDir, errMsg);
}

const IFeatureDiscretizer* featureDiscretizerGet(const std::string& strategyId)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerRegistry::instance().get(strategyId);
}

std::vector<std::string> featureDiscretizerListStrategyIds()
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerRegistry::instance().listStrategyIds();
}

FeatureGeometrySchema getFeatureGeometrySchema(const std::string& strategyId)
{
	FeatureGeometrySchema schema;
	auto edgeN = []()
	{
		FeatureGeometrySlotSchema s;
		s.kind = GeometryRefKind::Edge;
		s.minCount = 1;
		s.maxCount = -1;
		return s;
	};
	auto faceN = []()
	{
		FeatureGeometrySlotSchema s;
		s.kind = GeometryRefKind::Face;
		s.minCount = 1;
		s.maxCount = -1;
		return s;
	};
	if (strategyId == "EdgeChain")
	{
		schema.slotDefs.push_back(edgeN());
		return schema;
	}
	if (strategyId == "FaceBoundary" || strategyId == "FaceSection" || strategyId == "FaceParamSurface")
	{
		schema.slotDefs.push_back(faceN());
		return schema;
	}
	if (strategyId == "FaceIntersection")
	{
		FeatureGeometrySlotSchema s;
		s.kind = GeometryRefKind::Face;
		s.minCount = 2;
		s.maxCount = 2;
		schema.slotDefs.push_back(s);
		return schema;
	}
	if (strategyId == "FaceOffsetCurve")
	{
		schema.allowCrossWorkpiece = true;
		schema.primaryRole = "targetFace";
		FeatureGeometrySlotSchema edge;
		edge.role = "sourceEdge";
		edge.kind = GeometryRefKind::Edge;
		edge.minCount = 1;
		edge.maxCount = 1;
		FeatureGeometrySlotSchema face;
		face.role = "targetFace";
		face.kind = GeometryRefKind::Face;
		face.minCount = 1;
		face.maxCount = 1;
		schema.slotDefs.push_back(std::move(edge));
		schema.slotDefs.push_back(std::move(face));
		return schema;
	}
	if (strategyId == "SyntheticPolyline")
	{
		FeatureGeometrySlotSchema s;
		s.kind = GeometryRefKind::Polyline;
		s.minCount = 1;
		s.maxCount = 1;
		schema.slotDefs.push_back(s);
		return schema;
	}
	schema.slotDefs.push_back(edgeN());
	return schema;
}

std::vector<FeatureDiscretizerParamField> featureDiscretizerAllParamFields(const std::string& strategyId)
{
	ensureFeatureDiscretizersRegistered();
	return FeatureDiscretizerConfigRegistry::instance().paramFieldsForStrategy(strategyId);
}

bool discretizeFeatureList(const FeatureListDocument& doc, const ShapeHandle& shapeHandle, RawPath& out,
						   std::string* errMsg)
{
	ensureFeatureDiscretizersRegistered();
	if (!validateFeatureListDocument(doc, errMsg))
	{
		return false;
	}
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	return discretizeFeatureListInternal(doc, shape, out, errMsg);
}

bool discretizeFeatureList(const FeatureListDocument& doc, RawPath& out, std::string* errMsg)
{
	if (!validateFeatureListDocument(doc, errMsg))
	{
		return false;
	}
	if (doc.workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "workpiece.stepPathUtf8 is empty";
		}
		return false;
	}
	ShapeHandle handle;
	if (!readStepIntoHandle(doc.workpiece.stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return discretizeFeatureList(doc, handle, out, errMsg);
}

bool discretizeShapeEdgeIndex(const ShapeHandle& shape, int edgeIndex, double linearDeflectionMm,
							  std::vector<float>& outXyz, std::string* errMsg)
{
	outXyz.clear();
	TopoDS_Shape topo;
	if (!ShapeHandleAccess::nativeShape(shape, &topo) || topo.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	TopoDS_Edge edge;
	if (!shapeEdgeAtIndex(topo, edgeIndex, edge, errMsg))
	{
		return false;
	}
	TessellateParams tess;
	tess.linearDeflectionMm = linearDeflectionMm > 0.0 ? linearDeflectionMm : 0.1;
	Polyline3d poly;
	if (!discretizeEdge(edge, tess, poly, errMsg))
	{
		return false;
	}
	outXyz = std::move(poly.xyz);
	return !outXyz.empty();
}

} // namespace geoalgo
