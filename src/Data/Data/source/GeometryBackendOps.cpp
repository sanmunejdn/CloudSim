#include "pch.h"
#include "GeometryBackendOps.h"
#include "GeometryRef.h"
#include "BrepBackendData.h"
#include "BackendDataManager.h"
#include "MeshBackendData.h"

#include <FeatureSpec.h>
#include <MeshDiscretize.h>
#include <Discretize.h>
#include <ShapeIo.h>
#include <ShapeQuery.h>

namespace geometry_backend_ops
{

bool discretizeStepToMesh(
	const std::string& stepPathUtf8,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg)
{
	return geoalgo::tessellateStepFileToMesh(stepPathUtf8, params, soup, report, errMsg);
}

bool discretizeStepFaceToMesh(
	const std::string& stepPathUtf8,
	int faceIndex,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg)
{
	return geoalgo::discretizeStepFaceToMesh(stepPathUtf8, faceIndex, params, soup, report, errMsg);
}

bool discretizePolylineToMesh(
	const std::vector<float>& polylineXyz,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	geoalgo::Polyline3d poly;
	poly.xyz = polylineXyz;
	return geoalgo::discretizePolylineToMesh(poly, params, soup, errMsg);
}

bool discretizeStepEdgesToPolylines(
	const std::string& stepPathUtf8,
	const geoalgo::TessellateParams& params,
	std::vector<geoalgo::Polyline3d>& outPolylines,
	std::string* errMsg)
{
	return geoalgo::discretizeStepEdgesToPolylines(stepPathUtf8, params, outPolylines, errMsg);
}

bool intersectStepEdges(
	const std::string& stepPathUtf8,
	int edgeIndex1,
	int edgeIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepEdges(stepPathUtf8, edgeIndex1, edgeIndex2, params, result, errMsg);
}

bool intersectStepEdgeFace(
	const std::string& stepPathUtf8,
	int edgeIndex,
	int faceIndex,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepEdgeFace(stepPathUtf8, edgeIndex, faceIndex, params, result, errMsg);
}

bool intersectStepFaces(
	const std::string& stepPathUtf8,
	int faceIndex1,
	int faceIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepFaces(stepPathUtf8, faceIndex1, faceIndex2, params, result, errMsg);
}

bool intersectStepFiles(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepFiles(targetStepPathUtf8, toolStepPathUtf8, params, result, errMsg);
}

bool brepBooleanStepFilesToMesh(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	geoalgo::BrepBooleanOp op,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	return geoalgo::brepBooleanStepFilesToMesh(targetStepPathUtf8, toolStepPathUtf8, op, meshParams, outSoup, errMsg);
}

bool fuseStepEdgesToPolyline(
	const std::string& stepPathUtf8,
	const std::vector<int>& edgeIndices,
	const geoalgo::TessellateParams& disc,
	geoalgo::Polyline3d& out,
	std::string* errMsg)
{
	return geoalgo::fuseStepEdgesToPolyline(stepPathUtf8, edgeIndices, disc, out, errMsg);
}

bool sewStepFacesToMesh(
	const std::string& stepPathUtf8,
	const std::vector<int>& faceIndices,
	double toleranceMm,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	return geoalgo::sewStepFacesToMesh(stepPathUtf8, faceIndices, toleranceMm, meshParams, outSoup, errMsg);
}

void applyQualityPreset(geoalgo::MeshDiscretizeParams& params)
{
	geoalgo::applyQualityPreset(params);
}

void fillMeshReport(const std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report)
{
	geoalgo::fillMeshReport(soup, report);
}

bool resolveGeometryRef(const GeometryRef& ref, geoalgo::WorkpieceRef& out, std::string* errMsg)
{
	if (ref.backendIdUtf8.empty() && ref.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "geometry ref missing backendIdUtf8 and stepPathUtf8";
		}
		return false;
	}
	out.backendIdUtf8 = ref.backendIdUtf8;
	out.stepPathUtf8 = ref.stepPathUtf8;
	out.frameId = ref.frameId.empty() ? "workpiece" : ref.frameId;
	return true;
}

WorkpieceShapeSource resolveWorkpieceShape(
	const std::string& backendIdUtf8,
	BackendDataManager& mgr,
	const std::string& stepPathUtf8Optional,
	geoalgo::ShapeHandle& outShape,
	geoalgo::WorkpieceRef& outRef,
	std::string* errMsg)
{
	outShape = geoalgo::ShapeHandle{};
	outRef = geoalgo::WorkpieceRef{};
	outRef.backendIdUtf8 = backendIdUtf8;
	outRef.frameId = "workpiece";
	outRef.stepPathUtf8 = stepPathUtf8Optional;

	const auto data = mgr.getData(backendIdUtf8);
	if (!data)
	{
		if (errMsg)
		{
			*errMsg = "backend not found";
		}
		return WorkpieceShapeSource::Unavailable;
	}

	if (auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
	{
		if (!brep->hasGeometry())
		{
			if (errMsg)
			{
				*errMsg = "BrepModel has no shape";
			}
			return WorkpieceShapeSource::Unavailable;
		}
		outShape = brep->shapeRef();
		return WorkpieceShapeSource::InMemoryBrep;
	}

	if (!stepPathUtf8Optional.empty())
	{
		if (!geoalgo::readStepIntoHandle(stepPathUtf8Optional, outShape, errMsg))
		{
			return WorkpieceShapeSource::Unavailable;
		}
		return WorkpieceShapeSource::StepFileFallback;
	}

	if (errMsg)
	{
		*errMsg = "workpiece has no in-memory B-rep and no STEP path";
	}
	return WorkpieceShapeSource::Unavailable;
}

bool discretizeFeature(const geoalgo::FeatureSpec& spec, geoalgo::RawPath& out, std::string* errMsg)
{
	return geoalgo::discretizeFeature(spec, out, errMsg);
}

bool discretizeFeature(
	const geoalgo::FeatureSpec& spec,
	const geoalgo::ShapeHandle& shape,
	geoalgo::RawPath& out,
	std::string* errMsg)
{
	return geoalgo::discretizeFeature(spec, shape, out, errMsg);
}

bool discretizeFeatures(
	const std::vector<geoalgo::FeatureSpec>& specs,
	std::vector<geoalgo::RawPath>& out,
	std::string* errMsg)
{
	return geoalgo::discretizeFeatures(specs, out, errMsg);
}

bool validateFeatureSpec(const geoalgo::FeatureSpec& spec, std::string* errMsg)
{
	return geoalgo::validateFeatureSpecWithShape(spec, errMsg);
}

bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, out, errMsg);
}

bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, shape, out, errMsg);
}

bool featureSpecFromJson(const std::string& jsonUtf8, geoalgo::FeatureSpec& out, std::string* errMsg)
{
	return geoalgo::featureSpecFromJson(jsonUtf8, out, errMsg);
}

std::string featureSpecToJson(const geoalgo::FeatureSpec& spec)
{
	return geoalgo::featureSpecToJson(spec);
}

std::string featureCatalogToJson(const geoalgo::FeatureCatalog& catalog)
{
	return geoalgo::featureCatalogToJson(catalog);
}

bool suggestFeaturesFromCatalog(
	const geoalgo::FeatureCatalog& catalog,
	const std::string& intentUtf8,
	std::vector<geoalgo::FeatureSpec>& out,
	std::string* errMsg)
{
	return geoalgo::suggestFeaturesFromCatalog(catalog, intentUtf8, out, errMsg);
}

bool computeFeatureAnchor(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::FeatureRefs& refs,
	geoalgo::FeatureAnchor& out,
	std::string* errMsg)
{
	return geoalgo::computeFeatureAnchor(workpiece, refs, out, errMsg);
}

bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	const bool pickFace,
	const geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg,
	const int knownFaceIndex,
	const int knownEdgeIndex)
{
	out = geoalgo::FeatureSpec{};
	out.workpiece = workpiece;
	if (pickFace)
	{
		int faceIdx = knownFaceIndex;
		if (faceIdx < 0)
		{
			if (!geoalgo::resolveFaceIndexFromModelPoint(shape, modelPointA, faceIdx, 2.0, errMsg))
			{
				return false;
			}
		}
		else if (!geoalgo::validateShapeFaceIndex(shape, faceIdx, errMsg))
		{
			return false;
		}
		out.featureId = "face_" + std::to_string(faceIdx);
		out.kind = faceKindForPick;
		out.refs.faceIndices = {faceIdx};
		if (faceKindForPick == geoalgo::FeatureKind::FaceUVGrid)
		{
			out.refs.uvCountU = 16;
			out.refs.uvCountV = 16;
			out.discretize.stepMm = 0.0;
		}
		else
		{
			out.discretize.stepMm = 2.0;
		}
	}
	else
	{
		int edgeIdx = knownEdgeIndex;
		if (edgeIdx < 0)
		{
			if (!geoalgo::resolveEdgeIndexFromModelPoints(shape, modelPointA, modelPointB, edgeIdx, 2.0, errMsg))
			{
				return false;
			}
		}
		else if (!geoalgo::validateShapeEdgeIndex(shape, edgeIdx, errMsg))
		{
			return false;
		}
		out.featureId = "edge_" + std::to_string(edgeIdx);
		out.kind = geoalgo::FeatureKind::EdgeChain;
		out.refs.edgeIndices = {edgeIdx};
		out.discretize.stepMm = 2.0;
	}
	return geoalgo::validateFeatureSpec(out, errMsg);
}

bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	const bool pickFace,
	const geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg,
	const int knownFaceIndex,
	const int knownEdgeIndex)
{
	if (!workpiece.stepPathUtf8.empty())
	{
		geoalgo::ShapeHandle shape;
		if (!geoalgo::readStepIntoHandle(workpiece.stepPathUtf8, shape, errMsg))
		{
			return false;
		}
		return buildFeatureSpecFromModelPick(
			workpiece, shape, pickFace, faceKindForPick, modelPointA, modelPointB, out, errMsg,
			knownFaceIndex, knownEdgeIndex);
	}
	if (errMsg)
	{
		*errMsg = "buildFeatureSpecFromModelPick requires shape or stepPath";
	}
	return false;
}

} // namespace geometry_backend_ops