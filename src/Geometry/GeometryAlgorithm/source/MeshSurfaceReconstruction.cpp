#include "MeshSurfaceReconstruction.h"
#include "MeshSurfaceReconstruction/MeshSurfaceReconstructionInternal.h"
#include "MeshSurfaceReconstruction/MeshSurfaceReconstructionAmrtoPartition.h"
#include "ShapeHandle.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

namespace geoalgo
{
namespace
{

int stageOrder(const MeshSurfaceReconstructStage stage)
{
	switch (stage)
	{
	case MeshSurfaceReconstructStage::None:
		return 0;
	case MeshSurfaceReconstructStage::Partition:
		return 1;
	case MeshSurfaceReconstructStage::Sample:
		return 2;
	case MeshSurfaceReconstructStage::Fit:
		return 3;
	case MeshSurfaceReconstructStage::BoundaryBlend:
		return 4;
	case MeshSurfaceReconstructStage::JunctionBlend:
		return 5;
	case MeshSurfaceReconstructStage::Fair:
		return 6;
	case MeshSurfaceReconstructStage::Assemble:
		return 7;
	}
	return -1;
}

bool isNextGeoStage(const MeshSurfaceReconstructStage lastCompleted, const MeshSurfaceReconstructStage stage)
{
	if (stage == MeshSurfaceReconstructStage::Partition && lastCompleted == MeshSurfaceReconstructStage::None)
	{
		return true;
	}
	return stageOrder(stage) == stageOrder(lastCompleted) + 1;
}

int countShapeFaces(const ShapeHandle& shape)
{
	if (shape.isNull())
	{
		return 0;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		return 0;
	}
	int count = 0;
	for (TopExp_Explorer exp(native, TopAbs_FACE); exp.More(); exp.Next())
	{
		++count;
	}
	return count;
}

void countFitPatchTypes(const std::vector<meshrecon::QuadPatch>& patches, int& outBspline, int& outMeshFallback)
{
	outBspline = 0;
	outMeshFallback = 0;
	for (const meshrecon::QuadPatch& patch : patches)
	{
		if (patch.meshFallback)
		{
			++outMeshFallback;
		}
		else if (!patch.surface.IsNull() && !patch.face.IsNull())
		{
			++outBspline;
		}
	}
}

void updatePartitionStats(MeshSurfaceReconstructReport& report, const std::vector<meshrecon::QuadPatch>& patches)
{
	double sumFaces = 0.0;
	for (const meshrecon::QuadPatch& patch : patches)
	{
		sumFaces += static_cast<double>(patch.faceIndices.size());
	}
	report.patchCount = static_cast<int>(patches.size());
	report.avgFacesPerPatch = patches.empty() ? 0.0 : sumFaces / static_cast<double>(patches.size());
}

void patchDisplayRgb(const int patchIndex, const int patchCount, float& outR, float& outG, float& outB)
{
	const float golden = 0.6180339887f;
	const float hue = std::fmod(golden * static_cast<float>(patchIndex), 1.0f);
	const float sat = 0.72f;
	const float lit = 0.55f;
	const float c = (1.0f - std::fabs(2.0f * lit - 1.0f)) * sat;
	const float x = c * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
	const float m = lit - 0.5f * c;
	float r = 0.0f;
	float g = 0.0f;
	float b = 0.0f;
	if (hue < 1.0f / 6.0f)
	{
		r = c;
		g = x;
	}
	else if (hue < 2.0f / 6.0f)
	{
		r = x;
		g = c;
	}
	else if (hue < 3.0f / 6.0f)
	{
		g = c;
		b = x;
	}
	else if (hue < 4.0f / 6.0f)
	{
		g = x;
		b = c;
	}
	else if (hue < 5.0f / 6.0f)
	{
		r = x;
		b = c;
	}
	else
	{
		r = c;
		b = x;
	}
	outR = r + m;
	outG = g + m;
	outB = b + m;
	(void)patchCount;
}

void updateSampleStats(MeshSurfaceReconstructReport& report, const std::vector<meshrecon::QuadPatch>& patches)
{
	int totalSamples = 0;
	int gridN = 0;
	int gridNuMax = 0;
	int gridNvMax = 0;
	for (const meshrecon::QuadPatch& patch : patches)
	{
		const int nu = patch.gridNu > 0 ? patch.gridNu : patch.gridN;
		const int nv = patch.gridNv > 0 ? patch.gridNv : nu;
		gridNuMax = std::max(gridNuMax, nu);
		gridNvMax = std::max(gridNvMax, nv);
		gridN = std::max(gridN, std::max(nu, nv));
		totalSamples += static_cast<int>(patch.sampleXyz.size() / 3U);
		if (patch.samplingPath == "amrto-harmonic" || patch.samplingPath == "amrto-harmonic-geo")
		{
			++report.amrtoHarmonicSampleCount;
		}
		if (patch.samplingPath == "amrto-harmonic-geo")
		{
			++report.geodesicSquareHarmonicCount;
		}
	}
	report.totalSamplePoints = totalSamples;
	report.gridN = gridN;
	report.gridNuMax = gridNuMax;
	report.gridNvMax = gridNvMax;
}

} // namespace

struct MeshSurfaceReconstructSession::Impl
{
	std::vector<float> sourceSoup;
	meshrecon::IndexedMeshLite mesh;
	bool hasIndexed = false;
	std::vector<meshrecon::QuadPatch> patches;
	meshrecon::QuadMeshLite amrtoQuadMesh;
	meshrecon::GmcgResult amrtoGmcgResult;
	bool hasAmrtoCache = false;
	MeshSurfaceReconstructReport report;
	MeshSurfaceReconstructStage lastCompleted = MeshSurfaceReconstructStage::None;
	bool boundaryBlendOk = false;
};

MeshSurfaceReconstructSession::MeshSurfaceReconstructSession(std::vector<float> sourceSoup)
	: m_impl(std::make_unique<Impl>())
{
	m_impl->sourceSoup = std::move(sourceSoup);
	m_impl->report.inputTriangleCount = static_cast<int>(m_impl->sourceSoup.size() / 9U);
	m_impl->report.repairedTriangleCount = m_impl->report.inputTriangleCount;
}

MeshSurfaceReconstructSession::~MeshSurfaceReconstructSession() = default;

MeshSurfaceReconstructStage MeshSurfaceReconstructSession::lastCompleted() const
{
	return m_impl->lastCompleted;
}

const MeshSurfaceReconstructReport& MeshSurfaceReconstructSession::report() const
{
	return m_impl->report;
}

MeshSurfaceReconstructReport& MeshSurfaceReconstructSession::report()
{
	return m_impl->report;
}

MeshSurfaceReconstructSessionPtr createMeshSurfaceReconstructSession(std::vector<float> sourceSoup)
{
	return std::make_shared<MeshSurfaceReconstructSession>(std::move(sourceSoup));
}

bool runMeshSurfaceReconstructStage(
	MeshSurfaceReconstructSession& session,
	const MeshSurfaceReconstructStage stage,
	const MeshSurfaceReconstructParams& params,
	ShapeHandle* outShape,
	std::string* errMsg)
{
	MeshSurfaceReconstructSession::Impl& s = *session.m_impl;
	if (!isNextGeoStage(s.lastCompleted, stage))
	{
		if (errMsg)
		{
			*errMsg = "surface reconstruction stage out of order";
		}
		return false;
	}

	try
	{
		switch (stage)
		{
		case MeshSurfaceReconstructStage::Partition:
		{
			if (!s.hasIndexed)
			{
				if (!meshrecon::soupToIndexed(s.sourceSoup, s.mesh, errMsg))
				{
					return false;
				}
				s.hasIndexed = true;
			}
			bool partitionOk = false;
			if (params.partitionMode == MeshSurfacePartitionMode::AmrtoImGmcg)
			{
				partitionOk = meshrecon::partitionQuadDomainsAmrtoImGmcg(
					s.mesh,
					params,
					s.patches,
					s.report.junctionCount,
					&s.report,
					errMsg,
					&s.amrtoQuadMesh,
					&s.amrtoGmcgResult);
				s.hasAmrtoCache = partitionOk;
			}
			else
			{
				partitionOk = meshrecon::partitionQuadDomains(
					s.mesh, params, s.patches, s.report.junctionCount, &s.report, errMsg);
			}
			if (!partitionOk)
			{
				return false;
			}
			updatePartitionStats(s.report, s.patches);
			s.lastCompleted = MeshSurfaceReconstructStage::Partition;
			return true;
		}
		case MeshSurfaceReconstructStage::Sample:
			if (!meshrecon::samplePatchGrids(s.mesh, s.patches, params, errMsg))
			{
				return false;
			}
			updateSampleStats(s.report, s.patches);
			s.lastCompleted = MeshSurfaceReconstructStage::Sample;
			return true;
		case MeshSurfaceReconstructStage::Fit:
			if (!meshrecon::buildInitialBsplinePatches(s.mesh, s.patches, params, errMsg))
			{
				return false;
			}
			if (!meshrecon::applyMultiResolutionFit(s.mesh, s.patches, params, &s.report, errMsg))
			{
				return false;
			}
			countFitPatchTypes(s.patches, s.report.bsplinePatchCount, s.report.planeFallbackCount);
			s.report.nurbsPatchCount = s.report.bsplinePatchCount;
			meshrecon::aggregateFitRejectStats(s.report, s.patches);
			s.lastCompleted = MeshSurfaceReconstructStage::Fit;
			return true;
		case MeshSurfaceReconstructStage::BoundaryBlend:
			if (!meshrecon::applyBoundaryC2Blend(s.patches, params, s.boundaryBlendOk, &s.report, errMsg))
			{
				return false;
			}
			s.report.c2BlendSucceeded = s.boundaryBlendOk;
			s.lastCompleted = MeshSurfaceReconstructStage::BoundaryBlend;
			return true;
		case MeshSurfaceReconstructStage::JunctionBlend:
			if (!meshrecon::applyJunctionC2Blend(
					s.patches, s.report.junctionCount, params, &s.report, errMsg))
			{
				return false;
			}
			s.lastCompleted = MeshSurfaceReconstructStage::JunctionBlend;
			return true;
		case MeshSurfaceReconstructStage::Fair:
			if (!meshrecon::fairBsplinePatches(s.patches, params, s.report.globalFairingMetric, errMsg))
			{
				return false;
			}
			s.lastCompleted = MeshSurfaceReconstructStage::Fair;
			return true;
		case MeshSurfaceReconstructStage::Assemble:
		{
			ShapeHandle shape;
			if (!meshrecon::assembleBrepShape(s.mesh, s.patches, shape, errMsg))
			{
				return false;
			}
			if (!meshrecon::validateTessellationSanity(shape, params, errMsg))
			{
				return false;
			}
			s.report.maxDeviationMm = meshrecon::computeMaxDeviationMm(s.sourceSoup, s.patches);
			if (!meshrecon::validateOutputBbox(s.sourceSoup, shape, 3.0, errMsg))
			{
				return false;
			}
			s.report.outputFaceCount = countShapeFaces(shape);
			if (outShape)
			{
				*outShape = shape;
			}
			s.lastCompleted = MeshSurfaceReconstructStage::Assemble;
			return true;
		}
		default:
			if (errMsg)
			{
				*errMsg = "invalid surface reconstruction stage";
			}
			return false;
		}
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "surface reconstruction OCCT/internal error";
		}
		return false;
	}
}

bool buildPartitionColoredMeshSoup(
	const MeshSurfaceReconstructSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg)
{
	const MeshSurfaceReconstructSession::Impl& s = *session.m_impl;
	if (stageOrder(s.lastCompleted) < stageOrder(MeshSurfaceReconstructStage::Partition))
	{
		if (errMsg)
		{
			*errMsg = "partition stage not completed";
		}
		return false;
	}
	if (s.patches.empty() || s.sourceSoup.empty() || s.sourceSoup.size() % 9U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid partition visualization data";
		}
		return false;
	}

	const int faceCount = static_cast<int>(s.sourceSoup.size() / 9U);
	std::vector<int> facePatch(static_cast<std::size_t>(faceCount), 0);
	for (std::size_t pi = 0; pi < s.patches.size(); ++pi)
	{
		for (const int faceIdx : s.patches[pi].faceIndices)
		{
			if (faceIdx >= 0 && faceIdx < faceCount)
			{
				facePatch[static_cast<std::size_t>(faceIdx)] = static_cast<int>(pi);
			}
		}
	}

	outSoup = s.sourceSoup;
	outRgbPerVertex.assign(outSoup.size(), 0.75f);
	const int patchCount = static_cast<int>(s.patches.size());
	for (int faceIdx = 0; faceIdx < faceCount; ++faceIdx)
	{
		float r = 0.75f;
		float g = 0.75f;
		float b = 0.75f;
		patchDisplayRgb(facePatch[static_cast<std::size_t>(faceIdx)], patchCount, r, g, b);
		const std::size_t base = static_cast<std::size_t>(faceIdx) * 9U;
		for (int vert = 0; vert < 3; ++vert)
		{
			const std::size_t off = base + static_cast<std::size_t>(vert) * 3U;
			outRgbPerVertex[off] = r;
			outRgbPerVertex[off + 1U] = g;
			outRgbPerVertex[off + 2U] = b;
		}
	}
	return true;
}

bool buildSamplePointsCloud(
	const MeshSurfaceReconstructSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg)
{
	const MeshSurfaceReconstructSession::Impl& s = *session.m_impl;
	if (stageOrder(s.lastCompleted) < stageOrder(MeshSurfaceReconstructStage::Sample))
	{
		if (errMsg)
		{
			*errMsg = "sample stage not completed";
		}
		return false;
	}

	outXyz.clear();
	outRgba.clear();
	const int patchCount = static_cast<int>(s.patches.size());
	for (int pi = 0; pi < patchCount; ++pi)
	{
		const auto& patch = s.patches[static_cast<std::size_t>(pi)];
		const std::size_t ptCount = patch.sampleXyz.size() / 3U;
		if (ptCount == 0U)
		{
			continue;
		}
		float r = 0.75f;
		float g = 0.75f;
		float b = 0.75f;
		patchDisplayRgb(pi, patchCount, r, g, b);
		for (std::size_t i = 0U; i < ptCount; ++i)
		{
			const std::size_t off = i * 3U;
			outXyz.push_back(patch.sampleXyz[off]);
			outXyz.push_back(patch.sampleXyz[off + 1U]);
			outXyz.push_back(patch.sampleXyz[off + 2U]);
			outRgba.push_back(r);
			outRgba.push_back(g);
			outRgba.push_back(b);
			outRgba.push_back(1.0f);
		}
	}
	return !outXyz.empty();
}

bool buildFitPreviewShape(
	const MeshSurfaceReconstructSession& session,
	ShapeHandle& outShape,
	std::string* errMsg)
{
	const MeshSurfaceReconstructSession::Impl& s = *session.m_impl;
	if (stageOrder(s.lastCompleted) < stageOrder(MeshSurfaceReconstructStage::Fit))
	{
		if (errMsg)
		{
			*errMsg = "fit stage not completed";
		}
		return false;
	}
	outShape = ShapeHandle{};
	return meshrecon::assembleBrepShape(s.mesh, s.patches, outShape, errMsg);
}

bool reconstructBrepFromMeshSoup(
	const std::vector<float>& soup,
	const MeshSurfaceReconstructParams& params,
	ShapeHandle& outShape,
	MeshSurfaceReconstructReport& report,
	std::string* errMsg)
{
	report = MeshSurfaceReconstructReport{};
	outShape = ShapeHandle{};

	auto session = createMeshSurfaceReconstructSession(soup);
	if (!session)
	{
		if (errMsg)
		{
			*errMsg = "failed to create surface reconstruction session";
		}
		return false;
	}

	const MeshSurfaceReconstructStage pipeline[] = {
		MeshSurfaceReconstructStage::Partition,
		MeshSurfaceReconstructStage::Sample,
		MeshSurfaceReconstructStage::Fit,
		MeshSurfaceReconstructStage::BoundaryBlend,
		MeshSurfaceReconstructStage::JunctionBlend,
		MeshSurfaceReconstructStage::Fair,
		MeshSurfaceReconstructStage::Assemble,
	};

	for (const MeshSurfaceReconstructStage step : pipeline)
	{
		ShapeHandle stepShape;
		ShapeHandle* shapeOut = (step == MeshSurfaceReconstructStage::Assemble) ? &stepShape : nullptr;
		if (!runMeshSurfaceReconstructStage(*session, step, params, shapeOut, errMsg))
		{
			outShape = ShapeHandle{};
			return false;
		}
		if (step == MeshSurfaceReconstructStage::Assemble)
		{
			outShape = stepShape;
		}
	}

	report = session->report();
	return true;
}

namespace meshrecon
{

void aggregateFitRejectStats(MeshSurfaceReconstructReport& report, const std::vector<QuadPatch>& patches)
{
	report.fitRejectApprox = 0;
	report.fitRejectPole = 0;
	report.fitRejectFitGrid = 0;
	report.fitRejectFullGrid = 0;
	report.fitRejectMakeFace = 0;
	for (const QuadPatch& patch : patches)
	{
		switch (patch.fitRejectReason)
		{
		case PatchFitRejectReason::Approx:
			++report.fitRejectApprox;
			break;
		case PatchFitRejectReason::Pole:
			++report.fitRejectPole;
			break;
		case PatchFitRejectReason::FitGrid:
			++report.fitRejectFitGrid;
			break;
		case PatchFitRejectReason::FullGrid:
			++report.fitRejectFullGrid;
			break;
		case PatchFitRejectReason::MakeFace:
			++report.fitRejectMakeFace;
			break;
		default:
			break;
		}
	}
}

} // namespace meshrecon

} // namespace geoalgo
