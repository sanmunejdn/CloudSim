#include "MeshSurfaceReconstructionInternal.h"

#include "detail/OccIncludes.h"

#include <TColgp_Array2OfPnt.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

enum class PatchBorder : int
{
	UMin = 0,
	UMax = 1,
	VMin = 2,
	VMax = 3,
};

double boundaryBlendWeight(const double t)
{
	const double s = std::max(0.0, std::min(1.0, t));
	return s * s * s * (s * (s * 6.0 - 15.0) + 10.0);
}

int borderSampleCount(const TColgp_Array2OfPnt& poles, const PatchBorder border)
{
	if (border == PatchBorder::UMin || border == PatchBorder::UMax)
	{
		return poles.UpperCol() - poles.LowerCol() + 1;
	}
	return poles.UpperRow() - poles.LowerRow() + 1;
}

int mapBorderSampleIndex(const int sampleCount, const int i, const int resampleCount, const bool reversed)
{
	if (resampleCount <= 1)
	{
		return 0;
	}
	const int mapped = (sampleCount <= 1) ? 0 : (i * (sampleCount - 1) / (resampleCount - 1));
	return reversed ? (sampleCount - 1 - mapped) : mapped;
}

gp_Pnt borderPole(
	const TColgp_Array2OfPnt& poles,
	const PatchBorder border,
	const int sampleIndex)
{
	const int lr = poles.LowerRow();
	const int ur = poles.UpperRow();
	const int lc = poles.LowerCol();
	const int uc = poles.UpperCol();
	switch (border)
	{
	case PatchBorder::UMin:
		return poles.Value(lr, lc + sampleIndex);
	case PatchBorder::UMax:
		return poles.Value(ur, lc + sampleIndex);
	case PatchBorder::VMin:
		return poles.Value(lr + sampleIndex, lc);
	case PatchBorder::VMax:
		return poles.Value(lr + sampleIndex, uc);
	}
	return poles.Value(lr, lc);
}

gp_Pnt stripPole(
	const TColgp_Array2OfPnt& poles,
	const PatchBorder border,
	const int sampleIndex,
	const int depth)
{
	const int lr = poles.LowerRow();
	const int ur = poles.UpperRow();
	const int lc = poles.LowerCol();
	const int uc = poles.UpperCol();
	switch (border)
	{
	case PatchBorder::UMin:
		return poles.Value(std::min(ur, lr + depth - 1), lc + sampleIndex);
	case PatchBorder::UMax:
		return poles.Value(std::max(lr, ur - depth + 1), lc + sampleIndex);
	case PatchBorder::VMin:
		return poles.Value(lr + sampleIndex, std::min(uc, lc + depth - 1));
	case PatchBorder::VMax:
		return poles.Value(lr + sampleIndex, std::max(lc, uc - depth + 1));
	}
	return poles.Value(lr, lc);
}

void setStripPole(
	TColgp_Array2OfPnt& poles,
	const PatchBorder border,
	const int sampleIndex,
	const int depth,
	const gp_Pnt& point)
{
	const int lr = poles.LowerRow();
	const int ur = poles.UpperRow();
	const int lc = poles.LowerCol();
	const int uc = poles.UpperCol();
	switch (border)
	{
	case PatchBorder::UMin:
		poles.SetValue(std::min(ur, lr + depth - 1), lc + sampleIndex, point);
		break;
	case PatchBorder::UMax:
		poles.SetValue(std::max(lr, ur - depth + 1), lc + sampleIndex, point);
		break;
	case PatchBorder::VMin:
		poles.SetValue(lr + sampleIndex, std::min(uc, lc + depth - 1), point);
		break;
	case PatchBorder::VMax:
		poles.SetValue(lr + sampleIndex, std::max(lc, uc - depth + 1), point);
		break;
	}
}

int maxInwardDepth(const TColgp_Array2OfPnt& poles, const PatchBorder border)
{
	if (border == PatchBorder::UMin || border == PatchBorder::UMax)
	{
		return poles.UpperRow() - poles.LowerRow() + 1;
	}
	return poles.UpperCol() - poles.LowerCol() + 1;
}

double averageBorderDistance(
	const TColgp_Array2OfPnt& pa,
	const PatchBorder borderA,
	const bool reverseA,
	const TColgp_Array2OfPnt& pb,
	const PatchBorder borderB,
	const bool reverseB)
{
	const int countA = borderSampleCount(pa, borderA);
	const int countB = borderSampleCount(pb, borderB);
	const int resampleCount = std::max(2, std::min(countA, countB));
	double sum = 0.0;
	for (int i = 0; i < resampleCount; ++i)
	{
		const int ia = mapBorderSampleIndex(countA, i, resampleCount, reverseA);
		const int ib = mapBorderSampleIndex(countB, i, resampleCount, reverseB);
		sum += borderPole(pa, borderA, ia).Distance(borderPole(pb, borderB, ib));
	}
	return sum / static_cast<double>(resampleCount);
}

struct MatchedBorder
{
	PatchBorder borderA = PatchBorder::UMin;
	PatchBorder borderB = PatchBorder::UMin;
	bool reverseA = false;
	bool reverseB = false;
	double avgDistanceMm = 0.0;
	int resampleCount = 0;
};

double borderPolylineLength(const TColgp_Array2OfPnt& poles, const PatchBorder border)
{
	const int count = borderSampleCount(poles, border);
	if (count <= 1)
	{
		return 0.0;
	}
	double len = 0.0;
	for (int i = 1; i < count; ++i)
	{
		len += borderPole(poles, border, i).Distance(borderPole(poles, border, i - 1));
	}
	return len;
}

bool findBestSharedBorder(
	const TColgp_Array2OfPnt& pa,
	const TColgp_Array2OfPnt& pb,
	MatchedBorder& outMatch)
{
	MatchedBorder best;
	best.avgDistanceMm = std::numeric_limits<double>::max();

	const PatchBorder borders[] = {
		PatchBorder::UMin,
		PatchBorder::UMax,
		PatchBorder::VMin,
		PatchBorder::VMax,
	};

	for (const PatchBorder borderA : borders)
	{
		for (const PatchBorder borderB : borders)
		{
			for (const int revA : {0, 1})
			{
				for (const int revB : {0, 1})
				{
					const double dist = averageBorderDistance(
						pa, borderA, revA != 0, pb, borderB, revB != 0);
					if (dist < best.avgDistanceMm)
					{
						best.borderA = borderA;
						best.borderB = borderB;
						best.reverseA = revA != 0;
						best.reverseB = revB != 0;
						best.avgDistanceMm = dist;
						best.resampleCount = std::max(
							2,
							std::min(borderSampleCount(pa, borderA), borderSampleCount(pb, borderB)));
					}
				}
			}
		}
	}

	if (best.avgDistanceMm >= std::numeric_limits<double>::max() * 0.5)
	{
		return false;
	}

	// 邻接边在 3D 上应足够接近，否则说明 UV 边并非真实公共边
	const double edgeScale = std::min(
		borderPolylineLength(pa, best.borderA),
		borderPolylineLength(pb, best.borderB));
	const double gapThreshold = std::max(1.0, 0.25 * edgeScale);
	if (best.avgDistanceMm > gapThreshold)
	{
		return false;
	}

	outMatch = best;
	return true;
}

bool blendMatchedBorderPair(
	TColgp_Array2OfPnt& pa,
	TColgp_Array2OfPnt& pb,
	const MatchedBorder& match,
	const int stripDepth,
	int& outCtrlPtCount,
	double& outMaxMove)
{
	const int countA = borderSampleCount(pa, match.borderA);
	const int countB = borderSampleCount(pb, match.borderB);
	const int depthLimit = std::min(
		stripDepth,
		std::min(maxInwardDepth(pa, match.borderA), maxInwardDepth(pb, match.borderB)));
	if (depthLimit < 1 || match.resampleCount < 2)
	{
		return false;
	}

	bool moved = false;
	for (int depth = 1; depth <= depthLimit; ++depth)
	{
		const double w = boundaryBlendWeight(static_cast<double>(depth) / static_cast<double>(std::max(1, depthLimit)));
		for (int i = 0; i < match.resampleCount; ++i)
		{
			const int ia = mapBorderSampleIndex(countA, i, match.resampleCount, match.reverseA);
			const int ib = mapBorderSampleIndex(countB, i, match.resampleCount, match.reverseB);
			const gp_Pnt pA = stripPole(pa, match.borderA, ia, depth);
			const gp_Pnt pB = stripPole(pb, match.borderB, ib, depth);
			const gp_Pnt blend(
				pA.X() * w + pB.X() * (1.0 - w),
				pA.Y() * w + pB.Y() * (1.0 - w),
				pA.Z() * w + pB.Z() * (1.0 - w));
			const double dA = pA.Distance(blend);
			const double dB = pB.Distance(blend);
			if (dA > 1e-9 || dB > 1e-9)
			{
				moved = true;
				outCtrlPtCount += 2;
				outMaxMove = std::max(outMaxMove, dA);
				outMaxMove = std::max(outMaxMove, dB);
			}
			setStripPole(pa, match.borderA, ia, depth, blend);
			setStripPole(pb, match.borderB, ib, depth, blend);

			// 二阶导代理：对齐边界内侧一行
			if (depth == 1 && depthLimit >= 2)
			{
				const gp_Pnt bA = borderPole(pa, match.borderA, ia);
				const gp_Pnt bB = borderPole(pb, match.borderB, ib);
				const gp_Pnt innerA = stripPole(pa, match.borderA, ia, 2);
				const gp_Pnt innerB = stripPole(pb, match.borderB, ib, 2);
				const gp_Pnt curvA(
					innerA.X() - 2.0 * bA.X() + pA.X(),
					innerA.Y() - 2.0 * bA.Y() + pA.Y(),
					innerA.Z() - 2.0 * bA.Z() + pA.Z());
				const gp_Pnt curvB(
					innerB.X() - 2.0 * bB.X() + pB.X(),
					innerB.Y() - 2.0 * bB.Y() + pB.Y(),
					innerB.Z() - 2.0 * bB.Z() + pB.Z());
				const gp_Pnt curvBlend(
					curvA.X() * w + curvB.X() * (1.0 - w),
					curvA.Y() * w + curvB.Y() * (1.0 - w),
					curvA.Z() * w + curvB.Z() * (1.0 - w));
				const gp_Pnt innerBlendA(
					2.0 * bA.X() - pA.X() + curvBlend.X(),
					2.0 * bA.Y() - pA.Y() + curvBlend.Y(),
					2.0 * bA.Z() - pA.Z() + curvBlend.Z());
				const gp_Pnt innerBlendB(
					2.0 * bB.X() - pB.X() + curvBlend.X(),
					2.0 * bB.Y() - pB.Y() + curvBlend.Y(),
					2.0 * bB.Z() - pB.Z() + curvBlend.Z());
				setStripPole(pa, match.borderA, ia, 2, innerBlendA);
				setStripPole(pb, match.borderB, ib, 2, innerBlendB);
				outCtrlPtCount += 2;
			}
		}
	}
	return moved;
}

} // namespace

bool applyBoundaryC2Blend(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	bool& outBlendOk,
	MeshSurfaceReconstructReport* report,
	std::string* errMsg)
{
	outBlendOk = true;
	const int stripDepth = params.blendStripDepth > 0 ? params.blendStripDepth : 3;

	int pairCount = 0;
	int ctrlPtCount = 0;
	double maxMove = 0.0;

	for (std::size_t i = 0; i < patches.size(); ++i)
	{
		for (const int nb : patches[i].neighborPatchIds)
		{
			if (nb < 0 || static_cast<std::size_t>(nb) >= patches.size() || static_cast<std::size_t>(nb) <= i)
			{
				continue;
			}
			QuadPatch& a = patches[i];
			QuadPatch& b = patches[static_cast<std::size_t>(nb)];
			if (a.surface.IsNull() || b.surface.IsNull())
			{
				continue;
			}

			TColgp_Array2OfPnt pa = a.surface->Poles();
			TColgp_Array2OfPnt pb = b.surface->Poles();
			MatchedBorder match;
			if (!findBestSharedBorder(pa, pb, match))
			{
				continue;
			}

			bool pairHasMove = blendMatchedBorderPair(pa, pb, match, stripDepth, ctrlPtCount, maxMove);
			if (!tryRebuildBsplineSurface(a.surface, pa, a.surface)
				|| !tryRebuildBsplineSurface(b.surface, pb, b.surface))
			{
				outBlendOk = false;
			}
			if (pairHasMove)
			{
				++pairCount;
			}
		}
	}

	if (report)
	{
		report->boundaryBlendPairCount = pairCount;
		report->boundaryBlendCtrlPtCount = ctrlPtCount;
		report->boundaryBlendMaxMoveMm = maxMove;
	}

	(void)errMsg;
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
