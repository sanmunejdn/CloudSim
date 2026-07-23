/// @file CircularArcGeometry.cpp
/// @brief 三点定圆与弧采样

#include "CircularArcGeometry.h"

#include <algorithm>
#include <cmath>

namespace robot_kinematics
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

void cross(const double a[3], const double b[3], double out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

double dot(const double a[3], const double b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const double a[3])
{
	return std::sqrt(dot(a, a));
}

bool normalize(double v[3])
{
	const double n = norm(v);
	if (n < 1e-12)
	{
		return false;
	}
	v[0] /= n;
	v[1] /= n;
	v[2] /= n;
	return true;
}

void sub(const double a[3], const double b[3], double out[3])
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

double atan2AngleInPlane(const Circle3Fit& fit, const double p[3])
{
	double d[3];
	sub(p, fit.center, d);
	const double x = dot(d, fit.uAxis);
	const double y = dot(d, fit.vAxis);
	return std::atan2(y, x);
}

/// 从 from 到 to 的 CCW 角增量，落在 [0, 2π)
double ccwDelta(double from, double to)
{
	double d = to - from;
	while (d < 0.0)
	{
		d += kTwoPi;
	}
	while (d >= kTwoPi)
	{
		d -= kTwoPi;
	}
	return d;
}
} // namespace

bool fitCircle3Points(const double p0[3], const double p1[3], const double p2[3], Circle3Fit& out, double minRadiusMm)
{
	double e1[3];
	double e2[3];
	sub(p1, p0, e1);
	sub(p2, p0, e2);
	double n[3];
	cross(e1, e2, n);
	const double nLen = norm(n);
	const double e1Len = norm(e1);
	const double e2Len = norm(e2);
	if (nLen < 1e-9 * std::max(1.0, e1Len * e2Len) || e1Len < 1e-9 || e2Len < 1e-9)
	{
		return false;
	}
	if (!normalize(n))
	{
		return false;
	}

	// 外接圆心：|c-p0|=|c-p1|=|c-p2|，在平面内解线性系
	// 2(p1-p0)·c = |p1|^2-|p0|^2，2(p2-p0)·c = |p2|^2-|p0|^2，(c-p0)·n = 0
	const double d11 = dot(e1, e1);
	const double d12 = dot(e1, e2);
	const double d22 = dot(e2, e2);
	const double rhs1 = 0.5 * d11;
	const double rhs2 = 0.5 * d22;
	const double det = d11 * d22 - d12 * d12;
	if (std::abs(det) < 1e-18)
	{
		return false;
	}
	const double alpha = (rhs1 * d22 - rhs2 * d12) / det;
	const double beta = (rhs2 * d11 - rhs1 * d12) / det;
	out.center[0] = p0[0] + alpha * e1[0] + beta * e2[0];
	out.center[1] = p0[1] + alpha * e1[1] + beta * e2[1];
	out.center[2] = p0[2] + alpha * e1[2] + beta * e2[2];

	double rVec[3];
	sub(p0, out.center, rVec);
	out.radius = norm(rVec);
	if (out.radius < minRadiusMm)
	{
		return false;
	}

	// 校验三点到圆心距离一致
	double r1[3];
	double r2[3];
	sub(p1, out.center, r1);
	sub(p2, out.center, r2);
	if (std::abs(norm(r1) - out.radius) > 1e-3 * std::max(1.0, out.radius) ||
		std::abs(norm(r2) - out.radius) > 1e-3 * std::max(1.0, out.radius))
	{
		return false;
	}

	out.normal[0] = n[0];
	out.normal[1] = n[1];
	out.normal[2] = n[2];
	out.uAxis[0] = rVec[0] / out.radius;
	out.uAxis[1] = rVec[1] / out.radius;
	out.uAxis[2] = rVec[2] / out.radius;
	cross(out.normal, out.uAxis, out.vAxis);
	if (!normalize(out.vAxis))
	{
		return false;
	}

	// n=(p1-p0)×(p2-p0) ⇒ 绕 n 的 CCW 顺序为 p0→p1→p2；禁止短弧拼接，否则可能不经 Via
	out.theta0 = 0.0;
	const double thViaAbs = atan2AngleInPlane(out, p1);
	const double thEndAbs = atan2AngleInPlane(out, p2);
	const double sweepVia = ccwDelta(out.theta0, thViaAbs);
	const double sweepEnd = ccwDelta(thViaAbs, thEndAbs);
	const double totalSweep = sweepVia + sweepEnd;
	if (totalSweep < 1e-12 || totalSweep >= kTwoPi - 1e-9)
	{
		// 三点弧应落在 (0, 2π)；贴满整圆通常是数值退化
		return false;
	}
	out.thetaVia = out.theta0 + sweepVia;
	out.thetaEnd = out.theta0 + totalSweep;
	return true;
}

void pointOnArc(const Circle3Fit& fit, double u01, double outPos[3])
{
	const double u = std::clamp(u01, 0.0, 1.0);
	const double theta = fit.theta0 + u * (fit.thetaEnd - fit.theta0);
	const double c = std::cos(theta);
	const double s = std::sin(theta);
	outPos[0] = fit.center[0] + fit.radius * (c * fit.uAxis[0] + s * fit.vAxis[0]);
	outPos[1] = fit.center[1] + fit.radius * (c * fit.uAxis[1] + s * fit.vAxis[1]);
	outPos[2] = fit.center[2] + fit.radius * (c * fit.uAxis[2] + s * fit.vAxis[2]);
}

double arcLengthMm(const Circle3Fit& fit)
{
	return std::abs(fit.thetaEnd - fit.theta0) * fit.radius;
}

bool sampleArcByChord(const Circle3Fit& fit, double chordMm, int minSamples, int maxSamples,
					  std::vector<double>& outPositionsXyzFlat, std::vector<double>* outU01)
{
	outPositionsXyzFlat.clear();
	if (outU01)
	{
		outU01->clear();
	}
	const double len = arcLengthMm(fit);
	if (len < 1e-9 || fit.radius < 1e-9)
	{
		return false;
	}
	const double sweep = fit.thetaEnd - fit.theta0;
	if (std::abs(sweep) < 1e-12)
	{
		return false;
	}
	const double chord = std::max(chordMm, 1e-3);
	const double dTheta = chord / fit.radius;
	int samples = static_cast<int>(std::ceil(std::abs(sweep) / std::max(dTheta, 1e-9)));
	samples = std::max(minSamples, std::min(maxSamples, samples));

	// 均分参数 + 强制插入 Via，避免粗采样绕过途经点
	std::vector<double> us;
	us.reserve(static_cast<std::size_t>(samples) + 2u);
	for (int i = 1; i <= samples; ++i)
	{
		us.push_back(static_cast<double>(i) / static_cast<double>(samples));
	}
	const double uVia = (fit.thetaVia - fit.theta0) / sweep;
	if (uVia > 1e-6 && uVia < 1.0 - 1e-6)
	{
		us.push_back(uVia);
	}
	std::sort(us.begin(), us.end());
	us.erase(std::unique(us.begin(), us.end(),
						 [](double a, double b) { return std::abs(a - b) < 1e-9; }),
			 us.end());

	outPositionsXyzFlat.reserve(us.size() * 3u);
	if (outU01)
	{
		outU01->reserve(us.size());
	}
	for (double u : us)
	{
		double p[3];
		pointOnArc(fit, u, p);
		outPositionsXyzFlat.push_back(p[0]);
		outPositionsXyzFlat.push_back(p[1]);
		outPositionsXyzFlat.push_back(p[2]);
		if (outU01)
		{
			outU01->push_back(u);
		}
	}
	return outPositionsXyzFlat.size() >= 6u;
}

} // namespace robot_kinematics
