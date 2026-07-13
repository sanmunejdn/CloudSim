#include "detail/FeatureDiscretizeCommon.h"
#include "detail/FeatureDiscretizeFrame.h"

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

bool resampleRawPathByStep(RawPath& path, double stepMm)
{
	if (path.points.size() < 2U || stepMm <= 0.0)
	{
		return true;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < path.points.size(); ++i)
	{
		const auto& a = path.points[i - 1U].positionMm;
		const auto& b = path.points[i].positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
		segLen.push_back(len);
		total += len;
	}
	if (path.closed && path.points.size() > 1U)
	{
		const auto& a = path.points.back().positionMm;
		const auto& b = path.points.front().positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		segLen.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
		total += segLen.back();
	}
	if (total < stepMm)
	{
		return true;
	}
	const int sampleCount = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	RawPath resampled;
	resampled.sourceFeatureId = path.sourceFeatureId;
	resampled.closed = path.closed;
	resampled.segmentEndExclusive = path.segmentEndExclusive;
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
		const std::size_t i0 = seg % path.points.size();
		const std::size_t i1 = (i0 + 1U) % path.points.size();
		const auto& p0 = path.points[i0].positionMm;
		const auto& p1 = path.points[i1].positionMm;
		RawPathPoint rp;
		rp.positionMm.x = p0.x + (p1.x - p0.x) * local;
		rp.positionMm.y = p0.y + (p1.y - p0.y) * local;
		rp.positionMm.z = p0.z + (p1.z - p0.z) * local;
		if (path.points[i0].hasTangent)
		{
			rp.tangent = path.points[i0].tangent;
			rp.hasTangent = true;
		}
		if (path.points[i0].hasNormal)
		{
			rp.normal = path.points[i0].normal;
			rp.hasNormal = true;
		}
		resampled.points.push_back(rp);
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
