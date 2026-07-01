#include "PointCloudBackendOps.h"

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include "Crop.h"
#include "Downsample.h"
#include "Measure.h"
#include "Preprocess.h"
#include "Reconstruction.h"
#include "ReconstructionConfig.h"
#include "RegistrationNonRigid.h"
#include "RegistrationRigid.h"
#include "Transform.h"

#if defined(_WIN64)
#include "MeshReconstruct.h"
#include "MeshRemesh.h"
#include "MeshRepair.h"
#include "MeshSimplify.h"
#include "MeshSmooth.h"
#include "MeshDefectDetect.h"
#endif

namespace point_cloud_backend_ops
{

namespace
{

void writeBack(
	PointCloudBackendData& data,
	std::vector<float> xyz,
	std::vector<float> rgba,
	std::vector<float> normals)
{
	data.setPointBuffers(std::move(xyz), std::move(rgba), std::move(normals));
}

std::vector<float>* rgbaPtr(PointCloudBackendData& data, std::vector<float>& rgba)
{
	return data.hasPerVertexColors() ? &rgba : nullptr;
}

std::vector<float>* normalsPtr(PointCloudBackendData& data, std::vector<float>& normals)
{
	return data.hasPointNormals() ? &normals : nullptr;
}

} // namespace

bool downsamplePointCloudVoxel(
	PointCloudBackendData& data,
	const double voxelSizeMm,
	const unsigned int minPointsPerCell,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::downsampleVoxelGrid(xyz, voxelSizeMm, minPointsPerCell, rgbaPtr(data, rgba)))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "downsampleVoxelGrid failed";
		}
		return false;
	}
	std::vector<float> clearedNormals;
	writeBack(data, std::move(xyz), std::move(rgba), std::move(clearedNormals));
	return true;
}

bool downsamplePointCloudRandom(PointCloudBackendData& data, const double retainedFraction, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::downsampleRandom(xyz, retainedFraction, rgbaPtr(data, rgba)))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "downsampleRandom failed";
		}
		return false;
	}
	std::vector<float> normals;
	if (data.hasPointNormals())
	{
		normals.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool applyRigidTransformToPointCloud(
	PointCloudBackendData& data,
	const Eigen::Isometry3d& transform,
	std::string* errMsg)
{
	(void)errMsg;
	std::vector<float> xyz = data.pointPositionsXyz();
	pclalgo::transformXyzInPlace(xyz, transform);
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (data.hasPointNormals() && !normals.empty())
	{
		const Eigen::Matrix3d rot = transform.linear();
		for (std::size_t i = 0; i + 2 < normals.size(); i += 3)
		{
			const Eigen::Vector3d n(normals[i], normals[i + 1], normals[i + 2]);
			const Eigen::Vector3d out = rot * n;
			normals[i] = static_cast<float>(out.x());
			normals[i + 1] = static_cast<float>(out.y());
			normals[i + 2] = static_cast<float>(out.z());
		}
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool cropPointCloudByBox(PointCloudBackendData& data, const Eigen::AlignedBox3d& box, std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	std::vector<float> outXyz;
	std::vector<float> outRgba;
	std::vector<float> rgba = data.pointVertexRgba();
	if (data.hasPerVertexColors())
	{
		pclalgo::cropXyzByBox(srcXyz, rgba, box, outXyz, outRgba, nullptr);
	}
	else
	{
		pclalgo::cropXyzByBox(srcXyz, box, outXyz, nullptr);
	}
	std::vector<float> normals;
	writeBack(data, std::move(outXyz), std::move(outRgba), std::move(normals));
	return !data.pointPositionsXyz().empty();
}

bool cropPointCloudBySphere(
	PointCloudBackendData& data,
	const Eigen::Vector3d& centerMm,
	const double radiusMm,
	std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	std::vector<float> outXyz;
	std::vector<std::size_t> kept;
	pclalgo::cropXyzBySphere(srcXyz, centerMm, radiusMm, outXyz, &kept);
	std::vector<float> outRgba;
	if (data.hasPerVertexColors())
	{
		const std::vector<float>& rgba = data.pointVertexRgba();
		outRgba.reserve(kept.size() * 4U);
		for (const std::size_t idx : kept)
		{
			const std::size_t base = idx * 4U;
			outRgba.push_back(rgba[base]);
			outRgba.push_back(rgba[base + 1U]);
			outRgba.push_back(rgba[base + 2U]);
			outRgba.push_back(rgba[base + 3U]);
		}
	}
	std::vector<float> outNormals;
	if (data.hasPointNormals())
	{
		const std::vector<float>& normals = data.pointNormalsNxNyNz();
		outNormals.reserve(kept.size() * 3U);
		for (const std::size_t idx : kept)
		{
			const std::size_t base = idx * 3U;
			outNormals.push_back(normals[base]);
			outNormals.push_back(normals[base + 1U]);
			outNormals.push_back(normals[base + 2U]);
		}
	}
	writeBack(data, std::move(outXyz), std::move(outRgba), std::move(outNormals));
	return !data.pointPositionsXyz().empty();
}

bool cropPointCloudByPolyline2D(
	PointCloudBackendData& data,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	const int viewportWidth,
	const int viewportHeight,
	const bool keepInside,
	std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	std::vector<float> outXyz;
	std::vector<float> outRgba;
	std::vector<float> rgba = data.pointVertexRgba();
	if (data.hasPerVertexColors())
	{
		pclalgo::cropXyzByPolyline2D(
			srcXyz,
			rgba,
			polylineScreenXy,
			mvpMatrix,
			modelToWorld,
			viewportWidth,
			viewportHeight,
			keepInside,
			outXyz,
			outRgba,
			nullptr);
	}
	else
	{
		const std::vector<float> emptyRgba;
		pclalgo::cropXyzByPolyline2D(
			srcXyz,
			emptyRgba,
			polylineScreenXy,
			mvpMatrix,
			modelToWorld,
			viewportWidth,
			viewportHeight,
			keepInside,
			outXyz,
			outRgba,
			nullptr);
	}
	std::vector<float> normals;
	writeBack(data, std::move(outXyz), std::move(outRgba), std::move(normals));
	return !data.pointPositionsXyz().empty();
}

bool collectPointCloudIndicesByPolyline2D(
	const PointCloudBackendData& data,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	const int viewportWidth,
	const int viewportHeight,
	const bool keepInside,
	std::vector<std::size_t>& outIndices,
	std::string* errMsg)
{
	(void)errMsg;
	outIndices.clear();
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	if (srcXyz.size() < 3U || polylineScreenXy.size() < 6U)
	{
		return false;
	}
	pclalgo::collectXyzIndicesByPolyline2D(
		srcXyz,
		polylineScreenXy,
		mvpMatrix,
		modelToWorld,
		viewportWidth,
		viewportHeight,
		keepInside,
		outIndices);
	return true;
}

bool collectMeshTriangleIndicesByPolyline2D(
	const MeshBackendData& mesh,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	const int viewportWidth,
	const int viewportHeight,
	const bool keepInside,
	std::vector<int>& outTriangleIndices,
	std::string* errMsg)
{
	(void)errMsg;
	outTriangleIndices.clear();
	const std::vector<float>& soup = mesh.triangleSoup();
	if (soup.size() < 9U || (soup.size() % 9U) != 0U || polylineScreenXy.size() < 6U)
	{
		return false;
	}
	const std::size_t triCount = soup.size() / 9U;
	std::vector<float> centroids;
	centroids.reserve(triCount * 3U);
	for (std::size_t tri = 0; tri < triCount; ++tri)
	{
		const std::size_t b = tri * 9U;
		centroids.push_back((soup[b] + soup[b + 3U] + soup[b + 6U]) / 3.f);
		centroids.push_back((soup[b + 1U] + soup[b + 4U] + soup[b + 7U]) / 3.f);
		centroids.push_back((soup[b + 2U] + soup[b + 5U] + soup[b + 8U]) / 3.f);
	}
	std::vector<std::size_t> kept;
	pclalgo::collectXyzIndicesByPolyline2D(
		centroids,
		polylineScreenXy,
		mvpMatrix,
		modelToWorld,
		viewportWidth,
		viewportHeight,
		keepInside,
		kept);
	outTriangleIndices.reserve(kept.size());
	for (std::size_t idx : kept)
	{
		outTriangleIndices.push_back(static_cast<int>(idx));
	}
	return true;
}

bool measurePointCloud(const PointCloudBackendData& data, PointCloudMeasureResult& out, std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3U)
	{
		return false;
	}
	out.boundingBoxMm = pclalgo::computeBoundingBox(xyz);
	out.centroidMm = pclalgo::computeCentroid(xyz);
	out.averageSpacingMm = pclalgo::computeAverageSpacingMm(xyz);
	return true;
}

bool removePointCloudOutliers(
	PointCloudBackendData& data,
	const double removalPercent,
	const unsigned int kNeighbors,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (!pclalgo::removeOutliers(
			xyz,
			removalPercent,
			kNeighbors,
			normalsPtr(data, normals),
			rgbaPtr(data, rgba),
			errMsg))
	{
		return false;
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool smoothPointCloudBilateral(PointCloudBackendData& data, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (!pclalgo::smoothBilateral(xyz, normalsPtr(data, normals), errMsg))
	{
		return false;
	}
	std::vector<float> rgba = data.pointVertexRgba();
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool estimatePointCloudNormalsPca(PointCloudBackendData& data, const unsigned int kNeighbors, std::string* errMsg)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::estimateNormalsPca(xyz, normals, kNeighbors, errMsg))
	{
		return false;
	}
	data.setPointNormals(std::move(normals));
	return true;
}

bool estimatePointCloudNormalsJet(
	PointCloudBackendData& data,
	const unsigned int kNeighbors,
	const unsigned int degreeFitting,
	std::string* errMsg)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::estimateNormalsJet(xyz, normals, kNeighbors, degreeFitting, errMsg))
	{
		return false;
	}
	data.setPointNormals(std::move(normals));
	return true;
}

bool orientPointCloudNormalsMst(PointCloudBackendData& data, const unsigned int kNeighbors, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::orientNormalsMst(xyz, normals, kNeighbors, rgbaPtr(data, rgba), errMsg))
	{
		return false;
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool preprocessPointCloudForReconstruction(
	PointCloudBackendData& data,
	const double voxelPrefilterMm,
	const double outlierRemovalPercent,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::preprocessForReconstruction(xyz, normals, voxelPrefilterMm, outlierRemovalPercent, errMsg))
	{
		return false;
	}
	std::vector<float> rgba;
	if (data.hasPerVertexColors())
	{
		rgba.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool rigidRegisterPointCloudsIcp(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	PointCloudIcpResult& out,
	const int maxIterations,
	const double convergenceTransMm,
	const double maxPairDistanceMm,
	const std::size_t icpMaxPoints,
	std::string* errMsg)
{
	return pclalgo::rigidRegisterIcp(
		source.pointPositionsXyz(),
		target.pointPositionsXyz(),
		out.sourceToTarget,
		&out.rmseMm,
		maxIterations,
		convergenceTransMm,
		maxPairDistanceMm,
		icpMaxPoints,
		errMsg);
}

bool deformPointCloudTpsFromControls(
	PointCloudBackendData& data,
	const std::vector<std::size_t>& controlPointIndices,
	const std::vector<float>& controlDisplacementXyz,
	const double regularizationLambda,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<double> displacementDouble(controlDisplacementXyz.begin(), controlDisplacementXyz.end());
	if (!pclalgo::tpsDeformFromControls(
			xyz,
			controlPointIndices,
			displacementDouble.data(),
			displacementDouble.size() / 3U,
			regularizationLambda,
			errMsg))
	{
		return false;
	}
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals;
	if (data.hasPointNormals())
	{
		normals.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool deformPointCloudTpsFitAndDeform(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	const std::vector<std::size_t>& correspondenceIndices,
	std::vector<float>& sourceXyzDeformedOut,
	const double regularizationLambda,
	std::string* errMsg)
{
	return pclalgo::tpsFitAndDeform(
		source.pointPositionsXyz(),
		target.pointPositionsXyz(),
		correspondenceIndices,
		sourceXyzDeformedOut,
		regularizationLambda,
		errMsg);
}

bool reconstructMeshPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const double spacingMm,
	const double smAngleDeg,
	const double smRadiusRel,
	const double smDistanceRel,
	std::string* errMsg)
{
	const std::vector<float>& xyz = pointCloud.pointPositionsXyz();
	const std::vector<float>& normals = pointCloud.pointNormalsNxNyNz();
	if (normals.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "Poisson requires normals";
		}
		return false;
	}
	std::vector<float> soup;
	if (!pclalgo::reconstructPoisson(
			xyz, normals, soup, spacingMm, smAngleDeg, smRadiusRel, smDistanceRel, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

bool reconstructMeshFromPointCloudPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const double voxelPrefilterMm,
	std::string* errMsg)
{
	std::vector<float> xyz = pointCloud.pointPositionsXyz();
	std::vector<float> soup;
	if (!pclalgo::reconstructPoissonAuto(std::move(xyz), soup, voxelPrefilterMm, 5.0, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

bool reconstructMeshScaleSpace(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const std::size_t smoothIterations,
	const double meshingRadiusMm,
	std::string* errMsg)
{
	const std::vector<float>& xyz = pointCloud.pointPositionsXyz();
	std::vector<float> soup;
	if (!pclalgo::reconstructScaleSpace(xyz, soup, smoothIterations, meshingRadiusMm, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

bool reconstructMeshFromPointCloudPoissonWithConfig(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const pclalgo::ReconstructionConfig& config,
	std::string* errMsg)
{
	std::vector<float> xyz = pointCloud.pointPositionsXyz();
	std::vector<float> soup;
	if (!pclalgo::reconstructPoissonAutoWithConfig(std::move(xyz), soup, config, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

// === vcglib 网格后处理（x64 链接 VcgAlgorithms.dll） ===

#if defined(_WIN64)

bool simplifyMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	int targetFaceCount,
	double qualityThreshold,
	std::string* errMsg)
{
	::vcgalgo::SimplifyParams params;
	params.targetFaceCount = targetFaceCount;
	params.qualityThreshold = qualityThreshold;
	params.preserveBoundary = true;
	params.preserveTopology = true;
	return ::vcgalgo::simplifyQuadricEdgeCollapse(soupIn, soupOut, params, errMsg);
}

bool smoothMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	int iterations,
	bool useImplicitFairing,
	std::string* errMsg)
{
	if (useImplicitFairing)
	{
		return ::vcgalgo::smoothImplicitFairing(soupIn, 0.2, soupOut, errMsg);
	}
	return ::vcgalgo::smoothLaplacian(soupIn, iterations, soupOut, errMsg);
}

bool repairMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	bool removeDegenerate,
	bool removeDuplicate,
	bool removeNonManifold,
	std::string* errMsg)
{
	::vcgalgo::RepairParams params;
	params.removeDegenerate = removeDegenerate;
	params.removeDuplicate = removeDuplicate;
	params.removeNonManifold = removeNonManifold;
	params.fillHoles = false;
	return ::vcgalgo::repairMesh(soupIn, soupOut, params, errMsg);
}

bool remeshMeshIsotropic(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	double targetEdgeLengthMm,
	int iterations,
	std::string* errMsg)
{
	return ::vcgalgo::isotropicRemesh(soupIn, targetEdgeLengthMm, soupOut, iterations, 30.0, errMsg);
}

bool analyzeMeshDefects(
	const std::vector<float>& soupIn,
	std::vector<int>& defectFaceIndices,
	std::vector<float>& defectScores,
	std::vector<int>& defectKinds,
	int& outTotalFaces,
	int& outDefectFaceCount,
	double& outDefectAreaRatio,
	int& outNeedleCount,
	int& outProtrusionCount,
	int& outBoundarySpikeCount,
	const double sensitivity,
	const int minClusterFaces,
	const bool detectNeedle,
	const bool detectProtrusion,
	const bool detectBoundarySpike,
	std::string* errMsg)
{
	::vcgalgo::DefectDetectParams params;
	params.sensitivity = sensitivity;
	params.minClusterFaces = minClusterFaces;
	params.detectNeedle = detectNeedle;
	params.detectProtrusion = detectProtrusion;
	params.detectBoundarySpike = detectBoundarySpike;

	::vcgalgo::DefectDetectReport report;
	if (!::vcgalgo::detectMeshDefects(soupIn, report, params, errMsg))
	{
		return false;
	}

	outTotalFaces = report.totalFaces;
	outDefectFaceCount = report.defectFaceCount;
	outDefectAreaRatio = report.defectAreaRatio;
	outNeedleCount = report.needleCount;
	outProtrusionCount = report.protrusionCount;
	outBoundarySpikeCount = report.boundarySpikeCount;

	defectFaceIndices.clear();
	defectScores.clear();
	defectKinds.clear();
	defectFaceIndices.reserve(report.defects.size());
	defectScores.reserve(report.defects.size());
	defectKinds.reserve(report.defects.size());
	for (const ::vcgalgo::MeshDefectFace& d : report.defects)
	{
		defectFaceIndices.push_back(d.faceIndex);
		defectScores.push_back(static_cast<float>(d.score));
		defectKinds.push_back(static_cast<int>(d.kind));
	}
	return true;
}

bool reconstructMeshFromPointCloudPoissonAndPostProcess(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	int targetFaceCount,
	bool doRepair,
	bool doSmooth,
	std::string* errMsg)
{
	std::vector<float> xyz = pointCloud.pointPositionsXyz();
	std::vector<float> rawSoup;
	if (!pclalgo::reconstructPoissonAuto(std::move(xyz), rawSoup, 0.0, 5.0, errMsg))
	{
		return false;
	}
	if (rawSoup.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "Poisson reconstruction produced empty mesh";
		}
		return false;
	}

	std::vector<float> processed;
	if (!::vcgalgo::postProcessReconstructedMesh(
			rawSoup, processed, targetFaceCount, doRepair, doSmooth, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(processed));
	return true;
}

#else

bool simplifyMesh(
	const std::vector<float>&,
	std::vector<float>&,
	int,
	double,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh post-processing requires x64 build";
	}
	return false;
}

bool smoothMesh(
	const std::vector<float>&,
	std::vector<float>&,
	int,
	bool,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh post-processing requires x64 build";
	}
	return false;
}

bool repairMesh(
	const std::vector<float>&,
	std::vector<float>&,
	bool,
	bool,
	bool,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh post-processing requires x64 build";
	}
	return false;
}

bool remeshMeshIsotropic(
	const std::vector<float>&,
	std::vector<float>&,
	double,
	int,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh post-processing requires x64 build";
	}
	return false;
}

bool reconstructMeshFromPointCloudPoissonAndPostProcess(
	const PointCloudBackendData&,
	MeshBackendData&,
	int,
	bool,
	bool,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh post-processing requires x64 build";
	}
	return false;
}

bool analyzeMeshDefects(
	const std::vector<float>&,
	std::vector<int>&,
	std::vector<float>&,
	std::vector<int>&,
	int&,
	int&,
	double&,
	int&,
	int&,
	int&,
	double,
	int,
	bool,
	bool,
	bool,
	std::string* errMsg)
{
	if (errMsg != nullptr)
	{
		*errMsg = "mesh defect analysis requires x64 build";
	}
	return false;
}

#endif

} // namespace point_cloud_backend_ops
