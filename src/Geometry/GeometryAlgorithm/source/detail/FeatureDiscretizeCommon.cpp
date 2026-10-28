/// @file FeatureDiscretizeCommon.cpp
/// @brief FeatureDiscretizeCommon 实现

#include "detail/FeatureDiscretizeCommon.h"

#include "detail/FeatureDiscretizeFrame.h"

#include <algorithm>
#include <cmath>
namespace geoalgo
{
namespace detail
{
DiscretizeParams buildDiscretizeParamsFromInput(const FeatureDiscretizeInput& input)
{
	return buildDiscretizeParams(input.params);
}

TessellateParams toTessellate(const DiscretizeParams& p)
{
	TessellateParams t;
	t.linearDeflectionMm = p.linearDeflectionMm;
	t.linearDeflectionRelative = true;
	return t;
}

namespace
{
bool resampleContiguousByStep(const std::vector<RawPathPoint>& src, bool closed, double stepMm,
							  std::vector<RawPathPoint>& outPts)
{
	outPts.clear();
	if (src.size() < 2U || stepMm <= 0.0)
	{
		outPts = src;
		return true;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < src.size(); ++i)
	{
		const auto& a = src[i - 1U].positionMm;
		const auto& b = src[i].positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
		segLen.push_back(len);
		total += len;
	}
	if (closed && src.size() > 1U)
	{
		const auto& a = src.back().positionMm;
		const auto& b = src.front().positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		segLen.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
		total += segLen.back();
	}
	if (total < stepMm)
	{
		outPts = src;
		return true;
	}
	const int sampleCount = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	for (int s = 0; s < sampleCount; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(sampleCount - 1) * total;
		double acc = 0.0;
		std::size_t seg = 0;
		while (seg < segLen.size() && acc + segLen[seg] < t - 1e-9)
		{
			acc += segLen[seg];
			++seg;
		}
		const double local = (seg < segLen.size() && segLen[seg] > 1e-12) ? (t - acc) / segLen[seg] : 0.0;
		const std::size_t i0 = seg % src.size();
		const std::size_t i1 = (i0 + 1U) % src.size();
		const auto& p0 = src[i0].positionMm;
		const auto& p1 = src[i1].positionMm;
		RawPathPoint rp;
		rp.positionMm.x = p0.x + (p1.x - p0.x) * local;
		rp.positionMm.y = p0.y + (p1.y - p0.y) * local;
		rp.positionMm.z = p0.z + (p1.z - p0.z) * local;
		if (src[i0].hasTangent)
		{
			rp.tangent = src[i0].tangent;
			rp.hasTangent = true;
		}
		if (src[i0].hasNormal)
		{
			rp.normal = src[i0].normal;
			rp.hasNormal = true;
		}
		outPts.push_back(rp);
	}
	return true;
}
} // namespace

bool resampleRawPathByStep(RawPath& path, double stepMm)
{
	if (path.points.size() < 2U || stepMm <= 0.0)
	{
		return true;
	}

	// 按 segmentEndExclusive 分段重采样，禁止跨段抹平
	std::vector<std::size_t> ends = path.segmentEndExclusive;
	std::sort(ends.begin(), ends.end());
	ends.erase(std::unique(ends.begin(), ends.end()), ends.end());
	while (!ends.empty() && ends.back() >= path.points.size())
	{
		ends.pop_back();
	}
	if (!ends.empty() && ends.back() != path.points.size())
	{
		ends.push_back(path.points.size());
	}
	else if (ends.empty())
	{
		ends.push_back(path.points.size());
	}

	RawPath resampled;
	resampled.sourceFeatureId = path.sourceFeatureId;
	resampled.closed = path.closed && ends.size() == 1U;
	std::size_t start = 0;
	for (const std::size_t endExclusive : ends)
	{
		if (endExclusive <= start)
		{
			continue;
		}
		std::vector<RawPathPoint> part(path.points.begin() + static_cast<std::ptrdiff_t>(start),
									   path.points.begin() + static_cast<std::ptrdiff_t>(endExclusive));
		const bool partClosed = path.closed && ends.size() == 1U;
		std::vector<RawPathPoint> resampledPart;
		resampleContiguousByStep(part, partClosed, stepMm, resampledPart);
		if (!resampled.points.empty() && !resampledPart.empty())
		{
			resampled.segmentEndExclusive.push_back(resampled.points.size());
		}
		resampled.points.insert(resampled.points.end(), resampledPart.begin(), resampledPart.end());
		start = endExclusive;
	}
	path = std::move(resampled);
	return true;
}

void appendRawPath(const RawPath& part, RawPath& out)
{
	const std::size_t base = out.points.size();
	out.points.insert(out.points.end(), part.points.begin(), part.points.end());
	for (const std::size_t endExclusive : part.segmentEndExclusive)
	{
		out.segmentEndExclusive.push_back(base + endExclusive);
	}
	if (part.closed)
	{
		out.closed = true;
	}
}

bool shouldResampleAfterDiscretize(const std::string& strategyId, double stepMm)
{
	if (stepMm <= 0.0)
	{
		return false;
	}
	return strategyId != "FaceSection" && strategyId != "FaceParamSurface";
}

void applyPostDiscretizeResample(const std::string& strategyId, const nlohmann::json& params, RawPath& path)
{
	const DiscretizeParams disc = buildDiscretizeParams(params);
	if (shouldResampleAfterDiscretize(strategyId, disc.stepMm))
	{
		resampleRawPathByStep(path, disc.stepMm);
	}
}

} // namespace detail
} // namespace geoalgo
