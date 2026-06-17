#include "TrajectoryTemplates.h"

#include <algorithm>
#include <cmath>
#include <map>
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

const TubularCenterlineSample* sampleAtArc(
	const std::vector<TubularCenterlineSample>& samples,
	const double arcMm)
{
	if (samples.empty())
	{
		return nullptr;
	}
	if (arcMm <= samples.front().arcLengthMm)
	{
		return &samples.front();
	}
	for (std::size_t i = 1; i < samples.size(); ++i)
	{
		if (samples[i].arcLengthMm >= arcMm)
		{
			return &samples[i];
		}
	}
	return &samples.back();
}

TubularTemplatePoint makeTemplatePoint(
	const TubularCenterlineSample& s,
	const double angleRad,
	const double paramT)
{
	const Vec3 n = toVec3(s.normal);
	const Vec3 b = toVec3(s.binormal);
	const Vec3 offset = add(scale(n, std::cos(angleRad)), scale(b, std::sin(angleRad)));
	const Vec3 pos = add(toVec3(s.positionMm), scale(offset, s.radiusMm));
	TubularTemplatePoint tp;
	tp.pipeId = s.pipeId;
	tp.paramT = paramT;
	tp.positionMm = toArray(pos);
	tp.normalMm = toArray(normalizeVec3(offset));
	return tp;
}

void appendHelical(
	const std::vector<TubularCenterlineSample>& samples,
	const int coils,
	std::vector<TubularTemplatePoint>& out)
{
	if (samples.size() < 2U || coils <= 0)
	{
		return;
	}
	const double totalArc = samples.back().arcLengthMm;
	const int steps = std::max(coils * 24, 32);
	for (int i = 0; i <= steps; ++i)
	{
		const double u = static_cast<double>(i) / static_cast<double>(steps);
		const double arc = totalArc * u;
		const TubularCenterlineSample* s = sampleAtArc(samples, arc);
		if (!s)
		{
			continue;
		}
		const double angle = u * static_cast<double>(coils) * 2.0 * kPi;
		out.push_back(makeTemplatePoint(*s, angle, u));
	}
}

void appendCircumferential(
	const std::vector<TubularCenterlineSample>& samples,
	const int rings,
	std::vector<TubularTemplatePoint>& out)
{
	if (samples.empty() || rings <= 0)
	{
		return;
	}
	const int ringCount = std::max(rings, 4);
	for (int r = 0; r < ringCount; ++r)
	{
		const std::size_t idx = static_cast<std::size_t>(r) * (samples.size() - 1U) / static_cast<std::size_t>(ringCount - 1);
		const TubularCenterlineSample& s = samples[idx];
		const int segCount = 24;
		for (int k = 0; k < segCount; ++k)
		{
			const double angle = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(segCount);
			const double paramT = static_cast<double>(r) / static_cast<double>(ringCount);
			out.push_back(makeTemplatePoint(s, angle, paramT));
		}
	}
}

void appendAxialParallel(
	const std::vector<TubularCenterlineSample>& samples,
	const int meridians,
	std::vector<TubularTemplatePoint>& out)
{
	if (samples.size() < 2U || meridians <= 0)
	{
		return;
	}
	const int mCount = std::max(meridians, 4);
	for (int m = 0; m < mCount; ++m)
	{
		const double angle = 2.0 * kPi * static_cast<double>(m) / static_cast<double>(mCount);
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			const double paramT = static_cast<double>(i) / static_cast<double>(samples.size() - 1U);
			out.push_back(makeTemplatePoint(samples[i], angle, paramT));
		}
	}
}

void appendZigzag(
	const std::vector<TubularCenterlineSample>& samples,
	const int passes,
	std::vector<TubularTemplatePoint>& out)
{
	if (samples.size() < 2U || passes <= 0)
	{
		return;
	}
	const int passCount = std::max(passes, 8);
	for (int p = 0; p < passCount; ++p)
	{
		const double u = static_cast<double>(p) / static_cast<double>(passCount - 1);
		const double arc = samples.back().arcLengthMm * u;
		const TubularCenterlineSample* s = sampleAtArc(samples, arc);
		if (!s)
		{
			continue;
		}
		const double angle = (p % 2 == 0) ? 0.0 : kPi;
		out.push_back(makeTemplatePoint(*s, angle, u));
		const double angle2 = (p % 2 == 0) ? kPi : 0.0;
		out.push_back(makeTemplatePoint(*s, angle2, u + 0.001));
	}
}

} // namespace

bool runTrajectoryTemplates(
	const std::vector<TubularPipeSegment>& segments,
	const std::vector<TubularCenterlineSample>& centerlineSamples,
	const TubularGrindingParams& params,
	std::vector<TubularTemplatePoint>& outPoints,
	std::string* errMsg)
{
	outPoints.clear();
	if (centerlineSamples.empty())
	{
		if (errMsg)
		{
			*errMsg = "no centerline samples";
		}
		return false;
	}

	std::map<int, std::vector<TubularCenterlineSample>> byPipe;
	for (const TubularCenterlineSample& s : centerlineSamples)
	{
		byPipe[s.pipeId].push_back(s);
	}

	for (const TubularPipeSegment& segment : segments)
	{
		const auto it = byPipe.find(segment.id);
		if (it == byPipe.end() || it->second.size() < 2U)
		{
			continue;
		}
		const std::vector<TubularCenterlineSample>& samples = it->second;
		TubularGrindingTemplateKind kind = params.templateKind;
		if (kind == TubularGrindingTemplateKind::Auto)
		{
			kind = selectTemplateKind(segment, samples);
		}
		switch (kind)
		{
		case TubularGrindingTemplateKind::Helical:
			appendHelical(samples, params.helicalCoils, outPoints);
			break;
		case TubularGrindingTemplateKind::Circumferential:
			appendCircumferential(samples, params.circumferentialRings, outPoints);
			break;
		case TubularGrindingTemplateKind::AxialParallel:
			appendAxialParallel(samples, params.axialMeridians, outPoints);
			break;
		case TubularGrindingTemplateKind::Zigzag:
			appendZigzag(samples, params.zigzagPasses, outPoints);
			break;
		default:
			appendCircumferential(samples, params.circumferentialRings, outPoints);
			break;
		}
	}

	if (outPoints.empty())
	{
		if (errMsg)
		{
			*errMsg = "template generation produced no points";
		}
		return false;
	}
	return true;
}

} // namespace tg
} // namespace geoalgo
