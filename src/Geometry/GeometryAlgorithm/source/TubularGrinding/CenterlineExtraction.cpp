#include "CenterlineExtraction.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

Vec3 toVec3(const std::array<double, 3>& a)
{
	return {a[0], a[1], a[2]};
}

std::array<double, 3> toArray(const Vec3& v)
{
	return {v.x, v.y, v.z};
}

TubularCenterlineSample interpolateSample(
	const TubularCenterlineSample& a,
	const TubularCenterlineSample& b,
	const double t)
{
	TubularCenterlineSample out;
	out.pipeId = a.pipeId;
	out.arcLengthMm = a.arcLengthMm + t * (b.arcLengthMm - a.arcLengthMm);
	out.radiusMm = a.radiusMm + t * (b.radiusMm - a.radiusMm);
	for (int i = 0; i < 3; ++i)
	{
		out.positionMm[static_cast<std::size_t>(i)] =
			a.positionMm[static_cast<std::size_t>(i)]
			+ t * (b.positionMm[static_cast<std::size_t>(i)] - a.positionMm[static_cast<std::size_t>(i)]);
	}
	Vec3 ta = toVec3(a.tangent);
	Vec3 tb = toVec3(b.tangent);
	Vec3 tvec = normalizeVec3(add(scale(ta, 1.0 - t), scale(tb, t)));
	out.tangent = toArray(tvec);
	return out;
}

} // namespace

bool runCenterlineExtraction(
	const IndexedMeshLite& mesh,
	const std::vector<TubularPipeSegment>& segments,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	int& outSectionFitFailCount,
	std::string* errMsg)
{
	outSamples.clear();
	outSectionFitFailCount = 0;
	if (segments.empty())
	{
		if (errMsg)
		{
			*errMsg = "no pipe segments";
		}
		return false;
	}

	const double spacing = std::max(0.5, params.sectionSpacingMm);
	for (const TubularPipeSegment& segment : segments)
	{
		if (segment.faceIndices.empty())
		{
			continue;
		}
		Vec3 axis = toVec3(segment.axisHint);
		axis = normalizeVec3(axis);

		double tMin = std::numeric_limits<double>::max();
		double tMax = -std::numeric_limits<double>::max();
		Vec3 ref = mesh.faceCentroids[static_cast<std::size_t>(segment.faceIndices.front())];
		for (const int fi : segment.faceIndices)
		{
			const Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(fi)];
			const double t = dot(sub(c, ref), axis);
			tMin = std::min(tMin, t);
			tMax = std::max(tMax, t);
		}
		if (tMax <= tMin + spacing * 0.5)
		{
			continue;
		}

		Vec3 n0 = normalizeVec3(cross(axis, Vec3{0.0, 0.0, 1.0}));
		if (length(n0) < 1e-6)
		{
			n0 = normalizeVec3(cross(axis, Vec3{0.0, 1.0, 0.0}));
		}
		Vec3 b0 = normalizeVec3(cross(axis, n0));

		std::vector<TubularCenterlineSample> rawCenters;
		for (double t = tMin; t <= tMax + 1e-6; t += spacing)
		{
			const Vec3 sliceOrigin = add(ref, scale(axis, t));
			const double halfThick = spacing * 0.55;
			std::vector<std::array<double, 2>> sectionPts;
			for (const int fi : segment.faceIndices)
			{
				const Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(fi)];
				const double ts = dot(sub(c, sliceOrigin), axis);
				if (std::fabs(ts) > halfThick)
				{
					continue;
				}
				const Vec3 d = sub(c, sliceOrigin);
				const double u = dot(d, n0);
				const double v = dot(d, b0);
				sectionPts.push_back({u, v});
			}
			if (static_cast<int>(sectionPts.size()) < params.minSectionPoints)
			{
				++outSectionFitFailCount;
				continue;
			}
			double cx = 0.0;
			double cy = 0.0;
			double radius = 0.0;
			if (!fitCircle2d(sectionPts, cx, cy, radius))
			{
				++outSectionFitFailCount;
				continue;
			}
			const Vec3 centerWorld = add(add(sliceOrigin, scale(n0, cx)), scale(b0, cy));
			TubularCenterlineSample sample;
			sample.pipeId = segment.id;
			sample.radiusMm = radius;
			sample.positionMm = toArray(centerWorld);
			sample.tangent = toArray(axis);
			rawCenters.push_back(sample);
		}

		if (rawCenters.size() < 2U)
		{
			continue;
		}

		double arc = 0.0;
		for (std::size_t i = 0; i < rawCenters.size(); ++i)
		{
			if (i > 0U)
			{
				arc += length(sub(toVec3(rawCenters[i].positionMm), toVec3(rawCenters[i - 1].positionMm)));
			}
			rawCenters[i].arcLengthMm = arc;
			if (i > 0U && i + 1U < rawCenters.size())
			{
				Vec3 ta = sub(toVec3(rawCenters[i].positionMm), toVec3(rawCenters[i - 1].positionMm));
				Vec3 tb = sub(toVec3(rawCenters[i + 1].positionMm), toVec3(rawCenters[i].positionMm));
				rawCenters[i].tangent = toArray(normalizeVec3(add(ta, tb)));
			}
		}
		rawCenters.back().tangent = rawCenters[rawCenters.size() - 2U].tangent;

		const double totalArc = rawCenters.back().arcLengthMm;
		const int targetCount = std::max(8, static_cast<int>(totalArc / spacing) + 1);
		for (int si = 0; si < targetCount; ++si)
		{
			const double targetArc = totalArc * static_cast<double>(si) / static_cast<double>(targetCount - 1);
			std::size_t j = 1U;
			while (j < rawCenters.size() && rawCenters[j].arcLengthMm < targetArc)
			{
				++j;
			}
			if (j >= rawCenters.size())
			{
				outSamples.push_back(rawCenters.back());
				continue;
			}
			const TubularCenterlineSample& a = rawCenters[j - 1U];
			const TubularCenterlineSample& b = rawCenters[j];
			const double denom = b.arcLengthMm - a.arcLengthMm;
			const double u = denom > 1e-9 ? (targetArc - a.arcLengthMm) / denom : 0.0;
			outSamples.push_back(interpolateSample(a, b, u));
		}
	}

	if (outSamples.empty())
	{
		if (errMsg)
		{
			*errMsg = "centerline extraction produced no samples";
		}
		return false;
	}

	std::vector<TubularCenterlineSample> framed;
	buildFrenetFrames(outSamples, framed);
	outSamples = std::move(framed);
	return true;
}

} // namespace tg
} // namespace geoalgo
