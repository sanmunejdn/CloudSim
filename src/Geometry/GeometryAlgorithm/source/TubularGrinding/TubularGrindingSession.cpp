#include "TubularGrinding.h"

#include "CenterlineExtraction.h"
#include "MeshProjection.h"
#include "MeshFpfhRegionPartition.h"
#include "PipeSegmentation.h"
#include "TrajectoryTemplates.h"
#include "TubularGrindingCommon.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace geoalgo
{

struct TubularGrindingSession::Impl
{
	std::vector<float> sourceSoup;
	std::vector<float> sourcePointXyz;
	tg::SkeletonInputKind inputKind = tg::SkeletonInputKind::Mesh;
	tg::IndexedMeshLite mesh;
	bool hasMesh = false;
	std::vector<int> faceSegmentId;
	std::vector<int> faceRingId;
	std::vector<int> faceFpfhRegionId;
	int fpfhRegionCount = 0;
	std::vector<TubularPipeSegment> segments;
	std::vector<TubularCrossSectionRing> rings;
	std::vector<TubularCenterlineSample> centerlineSamples;
	TubularCenterlinePcaAxis centerlinePca;
	std::vector<TubularTemplatePoint> templatePoints;
	std::vector<TubularProjectedPoint> projectedPoints;
	std::vector<tg::OtLcIterationSnapshot> iterationSnapshots;
	TubularGrindingReport report;
	TubularGrindingStage lastCompleted = TubularGrindingStage::None;
	// Phase 1 局部轴线（每面一个，无效面为零向量）
	std::vector<tg::Vec3> faceLocalAxes;
};

TubularGrindingSession::TubularGrindingSession(std::vector<float> sourceSoup)
	: m_impl(std::make_unique<Impl>())
{
	m_impl->sourceSoup = std::move(sourceSoup);
	m_impl->inputKind = tg::SkeletonInputKind::Mesh;
}

TubularGrindingSession::TubularGrindingSession(std::vector<float> sourceSoup, bool fromPointCloud)
	: m_impl(std::make_unique<Impl>())
{
	if (fromPointCloud)
	{
		m_impl->sourcePointXyz = std::move(sourceSoup);
		m_impl->inputKind = tg::SkeletonInputKind::PointCloud;
	}
	else
	{
		m_impl->sourceSoup = std::move(sourceSoup);
		m_impl->inputKind = tg::SkeletonInputKind::Mesh;
	}
}

TubularGrindingSession::~TubularGrindingSession() = default;

TubularGrindingStage TubularGrindingSession::lastCompleted() const
{
	return m_impl->lastCompleted;
}

const TubularGrindingReport& TubularGrindingSession::report() const
{
	return m_impl->report;
}

TubularGrindingReport& TubularGrindingSession::report()
{
	return m_impl->report;
}

const std::vector<TubularPipeSegment>& TubularGrindingSession::pipeSegments() const
{
	return m_impl->segments;
}

const std::vector<TubularCrossSectionRing>& TubularGrindingSession::crossSectionRings() const
{
	return m_impl->rings;
}

const std::vector<TubularCenterlineSample>& TubularGrindingSession::centerlineSamples() const
{
	return m_impl->centerlineSamples;
}

const std::vector<TubularTemplatePoint>& TubularGrindingSession::templatePoints() const
{
	return m_impl->templatePoints;
}

const std::vector<TubularProjectedPoint>& TubularGrindingSession::projectedPoints() const
{
	return m_impl->projectedPoints;
}

TubularGrindingSessionPtr createTubularGrindingSession(std::vector<float> sourceSoup)
{
	return std::make_shared<TubularGrindingSession>(std::move(sourceSoup));
}

TubularGrindingSessionPtr createTubularGrindingSessionFromPointCloud(std::vector<float> pointXyz)
{
	return std::make_shared<TubularGrindingSession>(std::move(pointXyz), true);
}

namespace
{

bool isNextStage(const TubularGrindingStage last, const TubularGrindingStage want)
{
	switch (want)
	{
	case TubularGrindingStage::FpfhRegionPartition:
		return true;
	case TubularGrindingStage::Segment:
		return last == TubularGrindingStage::None || last == TubularGrindingStage::Segment;
	// Centerline is now the first stage (no Segment stage)
	case TubularGrindingStage::Centerline:
		return last == TubularGrindingStage::None;
	case TubularGrindingStage::TemplatePoints:
		return last == TubularGrindingStage::Centerline;
	case TubularGrindingStage::Project:
		return last == TubularGrindingStage::TemplatePoints;
	default:
		return false;
	}
}

void fillRgbaForPipe(
	const int pipeId,
	const int pipeCount,
	float& r,
	float& g,
	float& b,
	float& a)
{
	tg::segmentDisplayRgb(pipeId, pipeCount, r, g, b);
	a = 1.0f;
}

} // namespace

bool runTubularGrindingStage(
	TubularGrindingSession& session,
	const TubularGrindingStage stage,
	const TubularGrindingParams& params,
	std::string* errMsg)
{
	if (stage == TubularGrindingStage::None)
	{
		if (errMsg)
		{
			*errMsg = "invalid stage";
		}
		return false;
	}
	if (!isNextStage(session.m_impl->lastCompleted, stage)
		&& !(session.m_impl->lastCompleted == stage))
	{
		if (errMsg)
		{
			*errMsg = "stage order violation";
		}
		return false;
	}

	const bool isPointCloudInput =
		session.m_impl->inputKind == tg::SkeletonInputKind::PointCloud;
	auto ensureMesh = [&]() -> bool {
		if (session.m_impl->hasMesh)
		{
			return true;
		}
		if (isPointCloudInput)
		{
			if (errMsg)
			{
				*errMsg = "mesh topology required for this stage";
			}
			return false;
		}
		if (!tg::buildIndexedMeshLite(session.m_impl->sourceSoup, session.m_impl->mesh, errMsg))
		{
			return false;
		}
		session.m_impl->hasMesh = true;
		return true;
	};

	switch (stage)
	{
	case TubularGrindingStage::Segment:
	{
		if (!ensureMesh())
		{
			return false;
		}
		int junctionCount = 0;
		int regionCount = 0;
		if (!tg::runPipeSegmentation(
				session.m_impl->mesh,
				params,
				session.m_impl->segments,
				session.m_impl->rings,
				session.m_impl->faceSegmentId,
				junctionCount,
				regionCount,
				errMsg,
				&session.m_impl->faceLocalAxes))
		{
			return false;
		}
		session.m_impl->faceRingId.assign(static_cast<std::size_t>(session.m_impl->mesh.faceCount), -1);
		for (std::size_t ri = 0; ri < session.m_impl->rings.size(); ++ri)
		{
			for (const int f : session.m_impl->rings[ri].faceIndices)
			{
				session.m_impl->faceRingId[static_cast<std::size_t>(f)] = static_cast<int>(ri);
			}
		}
		session.m_impl->report.pipeCount = static_cast<int>(session.m_impl->segments.size());
		session.m_impl->report.ringCount = static_cast<int>(session.m_impl->rings.size());
		session.m_impl->report.junctionFaceCount = junctionCount;
		session.m_impl->report.regionCountBeforeFilter = regionCount;
		break;
	}
	case TubularGrindingStage::Centerline:
	{
		tg::SkeletonInput input;
		input.kind = session.m_impl->inputKind;
		if (session.m_impl->inputKind == tg::SkeletonInputKind::Mesh)
		{
			if (!session.m_impl->hasMesh)
			{
				if (!tg::buildIndexedMeshLite(session.m_impl->sourceSoup, session.m_impl->mesh, errMsg))
				{
					return false;
				}
				session.m_impl->hasMesh = true;
			}
			input.mesh = &session.m_impl->mesh;
		}
		else
		{
			input.pointXyz = &session.m_impl->sourcePointXyz;
		}

		int failCount = 0;
		const bool useOtLc =
			params.centerlineMethod == TubularGrindingCenterlineMethod::OtLc;
		if (useOtLc)
		{
			std::string otErr;
			tg::OtLcGraphDiagnostics graphDiag;
			session.m_impl->iterationSnapshots.clear();
			tg::OtLcIterationCallback snapCb =
				[&snapshots = session.m_impl->iterationSnapshots](
					const tg::OtLcIterationSnapshot& snap)
				{
					snapshots.push_back(snap);
				};
			if (!tg::runOtLcSkeletonCenterline(
					input,
					params,
					session.m_impl->centerlineSamples,
					&session.m_impl->centerlinePca,
					&otErr,
					&session.m_impl->report.centerlinePcaFallback,
					&graphDiag,
					snapCb))
			{
				if (errMsg)
				{
					*errMsg = otErr.empty() ? "otlc centerline failed" : otErr;
				}
				return false;
			}
			session.m_impl->report.centerlineOtLcExtraction = true;
			session.m_impl->report.centerlineOtRootCount = graphDiag.rootSampleCount;
			session.m_impl->report.centerlineOtEdgeCount = graphDiag.undirectedEdgeCount;
			session.m_impl->report.centerlineOtComponentCount = graphDiag.connectedComponentCount;
			session.m_impl->report.centerlineOtKnnFallbackEdges = graphDiag.usedKnnFallbackEdges;
			session.m_impl->report.centerlineOtPathKind = graphDiag.extractPathKind;
		}
		else if (session.m_impl->inputKind == tg::SkeletonInputKind::Mesh)
		{
			if (!tg::runCenterlineExtraction(
					session.m_impl->mesh,
					params,
					session.m_impl->centerlineSamples,
					failCount,
					errMsg,
					&session.m_impl->centerlinePca))
			{
				return false;
			}
		}
		else
		{
			if (errMsg)
			{
				*errMsg = "laplacian centerline requires mesh input";
			}
			return false;
		}
		session.m_impl->report.centerlinePointCount =
			static_cast<int>(session.m_impl->centerlineSamples.size());
		session.m_impl->report.sectionFitFailCount = failCount;
		break;
	}
	case TubularGrindingStage::TemplatePoints:
	{
		if (session.m_impl->centerlineSamples.empty())
		{
			if (errMsg)
			{
				*errMsg = "run centerline stage first";
			}
			return false;
		}
		std::vector<TubularPipeSegment> templateSegments = session.m_impl->segments;
		if (templateSegments.empty())
		{
			std::map<int, bool> pipeIds;
			for (const TubularCenterlineSample& s : session.m_impl->centerlineSamples)
			{
				pipeIds[s.pipeId] = true;
			}
			for (const auto& entry : pipeIds)
			{
				TubularPipeSegment virtualPipe;
				virtualPipe.id = entry.first;
				templateSegments.push_back(virtualPipe);
			}
		}
		if (!tg::runTrajectoryTemplates(
				templateSegments,
				session.m_impl->centerlineSamples,
				params,
				session.m_impl->templatePoints,
				errMsg))
		{
			return false;
		}
		session.m_impl->report.templatePointCount =
			static_cast<int>(session.m_impl->templatePoints.size());
		break;
	}
	case TubularGrindingStage::Project:
	{
		if (session.m_impl->templatePoints.empty())
		{
			if (errMsg)
			{
				*errMsg = "run template stage first";
			}
			return false;
		}
		double hitRate = 0.0;
		const bool projected = isPointCloudInput
			? tg::runPointCloudProjection(
				session.m_impl->sourcePointXyz,
				session.m_impl->templatePoints,
				params,
				session.m_impl->projectedPoints,
				hitRate,
				errMsg)
			: (ensureMesh()
				&& tg::runMeshProjection(
					session.m_impl->mesh,
					session.m_impl->templatePoints,
					params,
					session.m_impl->projectedPoints,
					hitRate,
					errMsg));
		if (!projected)
		{
			return false;
		}
		session.m_impl->report.projectedPointCount =
			static_cast<int>(session.m_impl->projectedPoints.size());
		session.m_impl->report.projectionHitRate = hitRate;
		break;
	}
	case TubularGrindingStage::FpfhRegionPartition:
	{
		if (isPointCloudInput)
		{
			if (errMsg)
			{
				*errMsg = "mesh topology required for this stage";
			}
			return false;
		}
		if (!ensureMesh())
		{
			return false;
		}
		tg::MeshFpfhPartitionParams fpfhParams;
		fpfhParams.featureVoxelMm = params.fpfhFeatureVoxelMm;
		fpfhParams.maxSamplePoints = params.fpfhMaxSamplePoints;
		fpfhParams.fpfhNeighbors = params.fpfhNeighbors;
		fpfhParams.saliencyNeighbors = params.fpfhSaliencyNeighbors;
		fpfhParams.keypointCount = params.fpfhKeypointCount;
		fpfhParams.keypointMinSeparationMm = params.fpfhKeypointMinSeparationMm;
		fpfhParams.regionGrowDist = params.fpfhRegionGrowDist;
		fpfhParams.regionGrowNormalAngleDeg = params.fpfhRegionGrowNormalAngleDeg;
		fpfhParams.minRegionFaces = params.fpfhMinRegionFaces;
		int regionCount = 0;
		int keypointCount = 0;
		if (!tg::runMeshFpfhRegionPartition(
				session.m_impl->mesh,
				fpfhParams,
				session.m_impl->faceFpfhRegionId,
				regionCount,
				keypointCount,
				errMsg))
		{
			return false;
		}
		session.m_impl->fpfhRegionCount = regionCount;
		session.m_impl->report.fpfhRegionCount = regionCount;
		session.m_impl->report.fpfhKeypointCount = keypointCount;
		return true;
	}
	default:
		if (errMsg)
		{
			*errMsg = "unknown stage";
		}
		return false;
	}

	session.m_impl->lastCompleted = stage;
	return true;
}

bool buildSegmentColoredMeshSoup(
	const TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg)
{
	if (session.m_impl->segments.empty() || session.m_impl->faceSegmentId.empty())
	{
		if (errMsg)
		{
			*errMsg = "segment stage not completed";
		}
		return false;
	}
	const std::vector<float>& soup = session.m_impl->sourceSoup;
	outSoup = soup;
	outRgbPerVertex.assign(soup.size(), 0.0f);
	const int pipeCount = static_cast<int>(session.m_impl->segments.size());
	for (int f = 0; f < session.m_impl->mesh.faceCount; ++f)
	{
		const int sid = session.m_impl->faceSegmentId[static_cast<std::size_t>(f)];
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		float a = 1.0f;
		if (sid == -2)
		{
			// 三通/交汇区
			r = 1.0f;
			g = 0.45f;
			b = 0.1f;
		}
		else if (sid >= 0)
		{
			fillRgbaForPipe(sid, pipeCount, r, g, b, a);
		}
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		for (int k = 0; k < 3; ++k)
		{
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 0U] = r;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 1U] = g;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 2U] = b;
		}
	}
	return true;
}

bool buildRingColoredMeshSoup(
	const TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg)
{
	if (session.m_impl->rings.empty() || session.m_impl->faceRingId.empty())
	{
		if (errMsg)
		{
			*errMsg = "segment stage not completed";
		}
		return false;
	}
	const std::vector<float>& soup = session.m_impl->sourceSoup;
	outSoup = soup;
	outRgbPerVertex.assign(soup.size(), 0.0f);
	const int ringCount = static_cast<int>(session.m_impl->rings.size());
	for (int f = 0; f < session.m_impl->mesh.faceCount; ++f)
	{
		const int sid = session.m_impl->faceSegmentId[static_cast<std::size_t>(f)];
		const int rid = session.m_impl->faceRingId[static_cast<std::size_t>(f)];
		float r = 0.45f;
		float g = 0.45f;
		float b = 0.45f;
		if (sid == -2)
		{
			r = 1.0f;
			g = 0.45f;
			b = 0.1f;
		}
		else if (rid >= 0)
		{
			tg::segmentDisplayRgb(rid, ringCount, r, g, b);
		}
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		for (int k = 0; k < 3; ++k)
		{
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 0U] = r;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 1U] = g;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 2U] = b;
		}
	}
	return true;
}

bool buildFpfhRegionColoredMeshSoup(
	const TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg)
{
	if (session.m_impl->faceFpfhRegionId.empty() || session.m_impl->fpfhRegionCount <= 0)
	{
		if (errMsg)
		{
			*errMsg = "fpfh region partition not completed";
		}
		return false;
	}
	const std::vector<float>& soup = session.m_impl->sourceSoup;
	outSoup = soup;
	outRgbPerVertex.assign(soup.size(), 0.75f);
	const int regionCount = session.m_impl->fpfhRegionCount;
	for (int f = 0; f < session.m_impl->mesh.faceCount; ++f)
	{
		const int rid = session.m_impl->faceFpfhRegionId[static_cast<std::size_t>(f)];
		float r = 0.75f;
		float g = 0.75f;
		float b = 0.75f;
		if (rid >= 0)
		{
			tg::segmentDisplayRgb(rid, regionCount, r, g, b);
		}
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		for (int k = 0; k < 3; ++k)
		{
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 0U] = r;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 1U] = g;
			outRgbPerVertex[base + static_cast<std::size_t>(k) * 3U + 2U] = b;
		}
	}
	return true;
}

bool buildRingCenterPointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (session.m_impl->rings.empty())
	{
		if (errMsg)
		{
			*errMsg = "segment stage not completed";
		}
		return false;
	}
	const int ringCount = static_cast<int>(session.m_impl->rings.size());
	outXyz.clear();
	outRgba.clear();
	for (const TubularCrossSectionRing& ring : session.m_impl->rings)
	{
		outXyz.push_back(static_cast<float>(ring.centerMm[0]));
		outXyz.push_back(static_cast<float>(ring.centerMm[1]));
		outXyz.push_back(static_cast<float>(ring.centerMm[2]));
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		tg::segmentDisplayRgb(ring.id, ringCount, r, g, b);
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(1.0f);
	}
	return true;
}

bool buildFaceNormalAxisLineSegments(
	const TubularGrindingSession& session,
	const TubularGrindingParams& params,
	std::vector<float>& outLineXyz,
	std::string* errMsg)
{
	if (!session.m_impl->hasMesh || session.m_impl->mesh.faceCount <= 0)
	{
		if (errMsg)
		{
			*errMsg = "mesh not built";
		}
		return false;
	}
	const tg::IndexedMeshLite& mesh = session.m_impl->mesh;
	outLineXyz.clear();
	const double dx = mesh.bboxMax[0] - mesh.bboxMin[0];
	const double dy = mesh.bboxMax[1] - mesh.bboxMin[1];
	const double dz = mesh.bboxMax[2] - mesh.bboxMin[2];
	const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	double axisLen = params.faceNormalAxisLengthMm;
	if (axisLen <= 0.0)
	{
		axisLen = std::max(2.0, diag * 0.012);
	}
	outLineXyz.reserve(static_cast<std::size_t>(mesh.faceCount) * 6U);
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const tg::Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(f)];
		const tg::Vec3 n = mesh.faceNormals[static_cast<std::size_t>(f)];
		const tg::Vec3 tip = tg::add(c, tg::scale(n, axisLen));
		outLineXyz.push_back(static_cast<float>(c.x));
		outLineXyz.push_back(static_cast<float>(c.y));
		outLineXyz.push_back(static_cast<float>(c.z));
		outLineXyz.push_back(static_cast<float>(tip.x));
		outLineXyz.push_back(static_cast<float>(tip.y));
		outLineXyz.push_back(static_cast<float>(tip.z));
	}
	return !outLineXyz.empty();
}

bool buildLocalAxisLineSegments(
	const TubularGrindingSession& session,
	const TubularGrindingParams& params,
	std::vector<float>& outLineXyz,
	std::string* errMsg)
{
	if (!session.m_impl->hasMesh || session.m_impl->mesh.faceCount <= 0)
	{
		if (errMsg)
		{
			*errMsg = "mesh not built";
		}
		return false;
	}
	if (session.m_impl->faceLocalAxes.empty())
	{
		if (errMsg)
		{
			*errMsg = "local axes not available (feature removed)";
		}
		return false;
	}
	const tg::IndexedMeshLite& mesh = session.m_impl->mesh;
	outLineXyz.clear();
	const double dx = mesh.bboxMax[0] - mesh.bboxMin[0];
	const double dy = mesh.bboxMax[1] - mesh.bboxMin[1];
	const double dz = mesh.bboxMax[2] - mesh.bboxMin[2];
	const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	double axisLen = params.faceNormalAxisLengthMm;
	if (axisLen <= 0.0)
	{
		axisLen = std::max(2.0, diag * 0.012);
	}
	outLineXyz.reserve(static_cast<std::size_t>(mesh.faceCount) * 6U);
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const tg::Vec3 axis = session.m_impl->faceLocalAxes[static_cast<std::size_t>(f)];
		// 跳过无效面（零向量）
		if (tg::length(axis) < 1e-6)
		{
			continue;
		}
		const tg::Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(f)];
		const tg::Vec3 tipA = tg::add(c, tg::scale(axis, axisLen));
		const tg::Vec3 tipB = tg::add(c, tg::scale(axis, -axisLen));
		// 正方向（青色端）
		outLineXyz.push_back(static_cast<float>(c.x));
		outLineXyz.push_back(static_cast<float>(c.y));
		outLineXyz.push_back(static_cast<float>(c.z));
		outLineXyz.push_back(static_cast<float>(tipA.x));
		outLineXyz.push_back(static_cast<float>(tipA.y));
		outLineXyz.push_back(static_cast<float>(tipA.z));
		// 反方向（品红端）
		outLineXyz.push_back(static_cast<float>(c.x));
		outLineXyz.push_back(static_cast<float>(c.y));
		outLineXyz.push_back(static_cast<float>(c.z));
		outLineXyz.push_back(static_cast<float>(tipB.x));
		outLineXyz.push_back(static_cast<float>(tipB.y));
		outLineXyz.push_back(static_cast<float>(tipB.z));
	}
	return !outLineXyz.empty();
}

bool buildCenterlinePointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (session.m_impl->centerlineSamples.empty())
	{
		if (errMsg)
		{
			*errMsg = "centerline stage not completed";
		}
		return false;
	}
	const int pipeCount = std::max(1, session.m_impl->report.pipeCount);
	outXyz.clear();
	outRgba.clear();
	for (const TubularCenterlineSample& s : session.m_impl->centerlineSamples)
	{
		outXyz.push_back(static_cast<float>(s.positionMm[0]));
		outXyz.push_back(static_cast<float>(s.positionMm[1]));
		outXyz.push_back(static_cast<float>(s.positionMm[2]));
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		float a = 1.0f;
		fillRgbaForPipe(s.pipeId, pipeCount, r, g, b, a);
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(a);
	}
	return true;
}

namespace
{

void appendLineSegment3(
	std::vector<float>& outLineXyz,
	const tg::Vec3& a,
	const tg::Vec3& b)
{
	outLineXyz.push_back(static_cast<float>(a.x));
	outLineXyz.push_back(static_cast<float>(a.y));
	outLineXyz.push_back(static_cast<float>(a.z));
	outLineXyz.push_back(static_cast<float>(b.x));
	outLineXyz.push_back(static_cast<float>(b.y));
	outLineXyz.push_back(static_cast<float>(b.z));
}

void appendPcaAxisArrow(
	std::vector<float>& outLineXyz,
	const TubularCenterlinePcaAxis& pca,
	const double bboxDiagMm)
{
	if (!pca.valid)
	{
		return;
	}
	const tg::Vec3 centroid{
		pca.centroidMm[0],
		pca.centroidMm[1],
		pca.centroidMm[2]};
	tg::Vec3 axis{
		pca.axis[0],
		pca.axis[1],
		pca.axis[2]};
	axis = tg::normalizeVec3(axis);
	if (tg::length(axis) < 1e-9)
	{
		return;
	}

	const double span = pca.extentMaxMm - pca.extentMinMm;
	const tg::Vec3 tail = tg::add(centroid, tg::scale(axis, pca.extentMinMm));
	const tg::Vec3 tip = tg::add(centroid, tg::scale(axis, pca.extentMaxMm));
	const double headLen = std::max(2.0, std::min(span * 0.12, bboxDiagMm * 0.035));
	const tg::Vec3 headBase = tg::add(tip, tg::scale(axis, -headLen));

	tg::Vec3 n0 = tg::normalizeVec3(tg::cross(axis, tg::Vec3{0.0, 0.0, 1.0}));
	if (tg::length(n0) < 1e-6)
	{
		n0 = tg::normalizeVec3(tg::cross(axis, tg::Vec3{0.0, 1.0, 0.0}));
	}
	const tg::Vec3 b0 = tg::normalizeVec3(tg::cross(axis, n0));
	const double wing = headLen * 0.45;

	appendLineSegment3(outLineXyz, tail, headBase);
	appendLineSegment3(outLineXyz, headBase, tip);
	appendLineSegment3(outLineXyz, tip, tg::add(headBase, tg::scale(n0, wing)));
	appendLineSegment3(outLineXyz, tip, tg::add(headBase, tg::scale(n0, -wing)));
	appendLineSegment3(outLineXyz, tip, tg::add(headBase, tg::scale(b0, wing)));
	appendLineSegment3(outLineXyz, tip, tg::add(headBase, tg::scale(b0, -wing)));
}

} // namespace

bool buildCenterlinePcaAxisArrowLineSegments(
	const TubularGrindingSession& session,
	std::vector<float>& outLineXyz,
	std::string* errMsg)
{
	if (!session.m_impl->centerlinePca.valid)
	{
		if (errMsg)
		{
			*errMsg = "centerline PCA axis not available";
		}
		return false;
	}
	outLineXyz.clear();
	double bboxDiag = 20.0;
	if (session.m_impl->hasMesh)
	{
		const tg::IndexedMeshLite& mesh = session.m_impl->mesh;
		const double dx = mesh.bboxMax[0] - mesh.bboxMin[0];
		const double dy = mesh.bboxMax[1] - mesh.bboxMin[1];
		const double dz = mesh.bboxMax[2] - mesh.bboxMin[2];
		bboxDiag = std::sqrt(dx * dx + dy * dy + dz * dz);
	}
	appendPcaAxisArrow(outLineXyz, session.m_impl->centerlinePca, bboxDiag);
	if (outLineXyz.empty())
	{
		if (errMsg)
		{
			*errMsg = "failed to build PCA axis arrow";
		}
		return false;
	}
	return true;
}

bool buildCenterlinePolylineXyz(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::string* errMsg)
{
	if (session.m_impl->centerlineSamples.empty())
	{
		if (errMsg)
		{
			*errMsg = "centerline stage not completed";
		}
		return false;
	}
	outXyz.clear();
	int currentPipe = session.m_impl->centerlineSamples.front().pipeId;
	for (const TubularCenterlineSample& s : session.m_impl->centerlineSamples)
	{
		if (s.pipeId != currentPipe && !outXyz.empty())
		{
			currentPipe = s.pipeId;
		}
		outXyz.push_back(static_cast<float>(s.positionMm[0]));
		outXyz.push_back(static_cast<float>(s.positionMm[1]));
		outXyz.push_back(static_cast<float>(s.positionMm[2]));
	}
	return true;
}

bool buildTemplatePointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (session.m_impl->templatePoints.empty())
	{
		if (errMsg)
		{
			*errMsg = "template stage not completed";
		}
		return false;
	}
	const int pipeCount = std::max(1, session.m_impl->report.pipeCount);
	outXyz.clear();
	outRgba.clear();
	for (const TubularTemplatePoint& p : session.m_impl->templatePoints)
	{
		outXyz.push_back(static_cast<float>(p.positionMm[0]));
		outXyz.push_back(static_cast<float>(p.positionMm[1]));
		outXyz.push_back(static_cast<float>(p.positionMm[2]));
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		float a = 1.0f;
		fillRgbaForPipe(p.pipeId, pipeCount, r, g, b, a);
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(a);
	}
	return true;
}

bool buildProjectedPointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (session.m_impl->projectedPoints.empty())
	{
		if (errMsg)
		{
			*errMsg = "project stage not completed";
		}
		return false;
	}
	const int pipeCount = std::max(1, session.m_impl->report.pipeCount);
	outXyz.clear();
	outRgba.clear();
	for (const TubularProjectedPoint& p : session.m_impl->projectedPoints)
	{
		outXyz.push_back(static_cast<float>(p.positionMm[0]));
		outXyz.push_back(static_cast<float>(p.positionMm[1]));
		outXyz.push_back(static_cast<float>(p.positionMm[2]));
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		float a = 1.0f;
		fillRgbaForPipe(p.pipeId, pipeCount, r, g, b, a);
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(a);
	}
	return true;
}

int iterationSnapshotCount(const TubularGrindingSession& session)
{
	return static_cast<int>(session.m_impl->iterationSnapshots.size());
}

int iterationSnapshotIteration(const TubularGrindingSession& session, int snapshotIndex)
{
	if (snapshotIndex < 0 || snapshotIndex >= iterationSnapshotCount(session))
	{
		return 0;
	}
	return session.m_impl->iterationSnapshots[static_cast<std::size_t>(snapshotIndex)].iteration;
}

bool buildIterationSnapshotPointsCloud(
	const TubularGrindingSession& session,
	int snapshotIndex,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (snapshotIndex < 0 || snapshotIndex >= iterationSnapshotCount(session))
	{
		if (errMsg)
		{
			*errMsg = "invalid snapshot index";
		}
		return false;
	}
	const auto& snap = session.m_impl->iterationSnapshots[static_cast<std::size_t>(snapshotIndex)];
	if (snap.samplePositions.empty())
	{
		if (errMsg)
		{
			*errMsg = "snapshot empty";
		}
		return false;
	}
	float r = 0.8f, g = 0.8f, b = 0.8f;
	if (snap.iteration <= 10)
	{
		r = 0.2f; g = 0.8f; b = 0.2f;
	}
	else if (snap.iteration <= 20)
	{
		r = 0.8f; g = 0.8f; b = 0.2f;
	}
	else
	{
		r = 0.8f; g = 0.2f; b = 0.2f;
	}
	outXyz.clear();
	outRgba.clear();
	outXyz.reserve(snap.samplePositions.size() * 3U);
	outRgba.reserve(snap.samplePositions.size() * 4U);
	for (const tg::Vec3& pos : snap.samplePositions)
	{
		outXyz.push_back(static_cast<float>(pos.x));
		outXyz.push_back(static_cast<float>(pos.y));
		outXyz.push_back(static_cast<float>(pos.z));
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(1.0f);
	}
	return true;
}

bool buildIterationSnapshotContractedPointsCloud(
	const TubularGrindingSession& session,
	int snapshotIndex,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	if (snapshotIndex < 0 || snapshotIndex >= iterationSnapshotCount(session))
	{
		if (errMsg)
		{
			*errMsg = "invalid snapshot index";
		}
		return false;
	}
	const auto& snap = session.m_impl->iterationSnapshots[static_cast<std::size_t>(snapshotIndex)];
	if (snap.contractedPositions.empty())
	{
		if (errMsg)
		{
			*errMsg = "contracted snapshot empty";
		}
		return false;
	}
	const float r = 0.25f;
	const float g = 0.55f;
	const float b = 0.95f;
	outXyz.clear();
	outRgba.clear();
	outXyz.reserve(snap.contractedPositions.size() * 3U);
	outRgba.reserve(snap.contractedPositions.size() * 4U);
	for (const tg::Vec3& pos : snap.contractedPositions)
	{
		outXyz.push_back(static_cast<float>(pos.x));
		outXyz.push_back(static_cast<float>(pos.y));
		outXyz.push_back(static_cast<float>(pos.z));
		outRgba.push_back(r);
		outRgba.push_back(g);
		outRgba.push_back(b);
		outRgba.push_back(1.0f);
	}
	return true;
}

bool computeEllipseFittingResidualReport(
	const TubularGrindingSession& session,
	const TubularGrindingParams& params,
	std::vector<double>& outPerRingRmsResiduals,
	std::string& outSummaryText,
	std::string* errMsg)
{
	outPerRingRmsResiduals.clear();
	outSummaryText.clear();

	if (!session.m_impl->hasMesh || session.m_impl->rings.empty())
	{
		if (errMsg)
		{
			*errMsg = "segment stage not completed";
		}
		return false;
	}

	const tg::IndexedMeshLite& mesh = session.m_impl->mesh;
	outPerRingRmsResiduals.reserve(session.m_impl->rings.size());

	double totalSumSq = 0.0;
	int totalCount = 0;

	for (std::size_t ri = 0; ri < session.m_impl->rings.size(); ++ri)
	{
		const TubularCrossSectionRing& ring = session.m_impl->rings[ri];
		if (ring.faceIndices.size() < 3)
		{
			outPerRingRmsResiduals.push_back(0.0);
			continue;
		}

		// 收集环内面的邻域和轴线
		std::vector<tg::Vec3> neighborhoodAxes;
		std::vector<int> neighborhoodFaces;
		for (const int f : ring.faceIndices)
		{
			if (static_cast<std::size_t>(f) < session.m_impl->faceLocalAxes.size())
			{
				const tg::Vec3 ax = session.m_impl->faceLocalAxes[static_cast<std::size_t>(f)];
				if (tg::length(ax) > 1e-6)
				{
					neighborhoodAxes.push_back(ax);
					neighborhoodFaces.push_back(f);
				}
			}
		}

		if (neighborhoodFaces.size() < 3)
		{
			outPerRingRmsResiduals.push_back(0.0);
			continue;
		}

		// 聚合主轴
		tg::Vec3 mainAxis = tg::computeMainAxisFromFaceAxes(neighborhoodAxes);
		mainAxis = tg::normalizeVec3(mainAxis);

		// 构建切平面
		tg::Vec3 n0 = tg::normalizeVec3(tg::cross(mainAxis, tg::Vec3{0.0, 0.0, 1.0}));
		if (tg::length(n0) < 1e-6)
		{
			n0 = tg::normalizeVec3(tg::cross(mainAxis, tg::Vec3{0.0, 1.0, 0.0}));
		}
		const tg::Vec3 b0 = tg::normalizeVec3(tg::cross(mainAxis, n0));

		// 投影到切平面
		tg::Vec3 centerSum{0.0, 0.0, 0.0};
		for (const int f : neighborhoodFaces)
		{
			centerSum = tg::add(centerSum, mesh.faceCentroids[static_cast<std::size_t>(f)]);
		}
		const tg::Vec3 sliceCenter = tg::scale(centerSum, 1.0 / static_cast<double>(neighborhoodFaces.size()));

		std::vector<std::array<double, 2>> projPts;
		projPts.reserve(neighborhoodFaces.size());
		for (const int f : neighborhoodFaces)
		{
			const tg::Vec3 d = tg::sub(mesh.faceCentroids[static_cast<std::size_t>(f)], sliceCenter);
			const double u = tg::dot(d, n0);
			const double v = tg::dot(d, b0);
			projPts.push_back({u, v});
		}

		// 椭圆拟合
		double semiMajor = 0.0, semiMinor = 0.0, cx = 0.0, cy = 0.0, rotRad = 0.0;
		if (!tg::fitEllipse2D(projPts, semiMajor, semiMinor, cx, cy, rotRad))
		{
			outPerRingRmsResiduals.push_back(0.0);
			continue;
		}

		// 计算残差
		std::vector<double> residuals;
		const double rms = tg::computeEllipseFittingResiduals(
			projPts, semiMajor, semiMinor, cx, cy, rotRad, residuals);

		outPerRingRmsResiduals.push_back(rms);
		totalSumSq += rms * rms * static_cast<double>(residuals.size());
		totalCount += static_cast<int>(residuals.size());
	}

	// 生成摘要文本
	const double globalRms = (totalCount > 0)
		? std::sqrt(totalSumSq / static_cast<double>(totalCount)) : 0.0;

	char buf[256];
	std::snprintf(buf, sizeof(buf),
		"椭圆拟合残差: 全局RMS=%.4f mm, %zu 个环",
		globalRms, session.m_impl->rings.size());
	outSummaryText = buf;

	return true;
}

} // namespace geoalgo
