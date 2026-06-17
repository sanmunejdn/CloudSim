#include "TubularGrinding.h"

#include "CenterlineExtraction.h"
#include "MeshProjection.h"
#include "PipeSegmentation.h"
#include "TrajectoryTemplates.h"
#include "TubularGrindingCommon.h"

#include <algorithm>
#include <string>
#include <vector>

namespace geoalgo
{

struct TubularGrindingSession::Impl
{
	std::vector<float> sourceSoup;
	tg::IndexedMeshLite mesh;
	bool hasMesh = false;
	std::vector<int> faceSegmentId;
	std::vector<int> faceRingId;
	std::vector<TubularPipeSegment> segments;
	std::vector<TubularCrossSectionRing> rings;
	std::vector<TubularCenterlineSample> centerlineSamples;
	std::vector<TubularTemplatePoint> templatePoints;
	std::vector<TubularProjectedPoint> projectedPoints;
	TubularGrindingReport report;
	TubularGrindingStage lastCompleted = TubularGrindingStage::None;
};

TubularGrindingSession::TubularGrindingSession(std::vector<float> sourceSoup)
	: m_impl(std::make_unique<Impl>())
{
	m_impl->sourceSoup = std::move(sourceSoup);
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

namespace
{

bool isNextStage(const TubularGrindingStage last, const TubularGrindingStage want)
{
	switch (want)
	{
	case TubularGrindingStage::Segment:
		return last == TubularGrindingStage::None;
	case TubularGrindingStage::Centerline:
		return last == TubularGrindingStage::Segment;
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
	if (!session.m_impl->hasMesh)
	{
		if (!tg::buildIndexedMeshLite(session.m_impl->sourceSoup, session.m_impl->mesh, errMsg))
		{
			return false;
		}
		session.m_impl->hasMesh = true;
	}

	switch (stage)
	{
	case TubularGrindingStage::Segment:
	{
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
				errMsg))
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
		if (session.m_impl->segments.empty())
		{
			if (errMsg)
			{
				*errMsg = "run segment stage first";
			}
			return false;
		}
		int failCount = 0;
		if (!tg::runCenterlineExtraction(
				session.m_impl->mesh,
				session.m_impl->segments,
				params,
				session.m_impl->centerlineSamples,
				failCount,
				errMsg))
		{
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
		if (!tg::runTrajectoryTemplates(
				session.m_impl->segments,
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
		if (!tg::runMeshProjection(
				session.m_impl->mesh,
				session.m_impl->templatePoints,
				params,
				session.m_impl->projectedPoints,
				hitRate,
				errMsg))
		{
			return false;
		}
		session.m_impl->report.projectedPointCount =
			static_cast<int>(session.m_impl->projectedPoints.size());
		session.m_impl->report.projectionHitRate = hitRate;
		break;
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

} // namespace geoalgo
