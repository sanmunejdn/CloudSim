#include "BackendPrimitiveGeometry.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

// 外视 CCW；法线 (p1-p0)×(p2-p0)
void pushTri(std::vector<float>& soup,
	float ax, float ay, float az,
	float bx, float by, float bz,
	float cx, float cy, float cz)
{
	soup.insert(soup.end(), { ax, ay, az, bx, by, bz, cx, cy, cz });
}

int clampInt(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }
} // namespace

namespace BackendPrimitiveGeometry {

std::vector<float> makeBoxTriangleSoup(double lengthMm, double widthMm, double heightMm)
{
	const float hx = static_cast<float>(lengthMm * 0.5);
	const float hy = static_cast<float>(widthMm * 0.5);
	const float hz = static_cast<float>(heightMm * 0.5);
	std::vector<float> soup;
	soup.reserve(108);
	// +Z top
	pushTri(soup, -hx, -hy, hz, hx, -hy, hz, hx, hy, hz);
	pushTri(soup, -hx, -hy, hz, hx, hy, hz, -hx, hy, hz);
	// -Z 底面
	pushTri(soup, -hx, -hy, -hz, hx, hy, -hz, hx, -hy, -hz);
	pushTri(soup, -hx, -hy, -hz, -hx, hy, -hz, hx, hy, -hz);
	// -Y
	pushTri(soup, -hx, -hy, -hz, hx, -hy, -hz, hx, -hy, hz);
	pushTri(soup, -hx, -hy, -hz, hx, -hy, hz, -hx, -hy, hz);
	// +Y
	pushTri(soup, -hx, hy, -hz, hx, hy, hz, hx, hy, -hz);
	pushTri(soup, -hx, hy, -hz, -hx, hy, hz, hx, hy, hz);
	// -X
	pushTri(soup, -hx, -hy, -hz, -hx, hy, hz, -hx, hy, -hz);
	pushTri(soup, -hx, -hy, -hz, -hx, -hy, hz, -hx, hy, hz);
	// +X
	pushTri(soup, hx, -hy, -hz, hx, hy, hz, hx, -hy, hz);
	pushTri(soup, hx, -hy, -hz, hx, hy, -hz, hx, hy, hz);
	return soup;
}

std::vector<float> makeCylinderTriangleSoup(double radiusMm, double heightMm, int segments)
{
	segments = clampInt(segments, 8, 128);
	const float r = static_cast<float>(radiusMm);
	const float z0 = static_cast<float>(-heightMm * 0.5);
	const float z1 = static_cast<float>(heightMm * 0.5);
	std::vector<float> soup;
	for (int i = 0; i < segments; ++i)
	{
		const float t0 = static_cast<float>(2.0 * kPi * i / segments);
		const float t1 = static_cast<float>(2.0 * kPi * (i + 1) / segments);
		const float c0 = r * std::cos(t0);
		const float s0 = r * std::sin(t0);
		const float c1 = r * std::cos(t1);
		const float s1 = r * std::sin(t1);
		pushTri(soup, c0, s0, z0, c1, s1, z0, c1, s1, z1);
		pushTri(soup, c0, s0, z0, c1, s1, z1, c0, s0, z1);
		pushTri(soup, 0.f, 0.f, z0, c1, s1, z0, c0, s0, z0);
		pushTri(soup, 0.f, 0.f, z1, c0, s0, z1, c1, s1, z1);
	}
	return soup;
}

std::vector<float> makeConeTriangleSoup(double radiusBottomMm, double radiusTopMm, double heightMm, int segments)
{
	segments = clampInt(segments, 8, 128);
	const float rb = static_cast<float>(radiusBottomMm);
	const float rt = static_cast<float>(std::max(0.0, radiusTopMm));
	const float z0 = static_cast<float>(-heightMm * 0.5);
	const float z1 = static_cast<float>(heightMm * 0.5);
	const bool sharpApex = rt <= 1e-6f;
	std::vector<float> soup;
	for (int i = 0; i < segments; ++i)
	{
		const float t0 = static_cast<float>(2.0 * kPi * i / segments);
		const float t1 = static_cast<float>(2.0 * kPi * (i + 1) / segments);
		const float c0b = rb * std::cos(t0);
		const float s0b = rb * std::sin(t0);
		const float c1b = rb * std::cos(t1);
		const float s1b = rb * std::sin(t1);
		if (sharpApex)
			pushTri(soup, c0b, s0b, z0, c1b, s1b, z0, 0.f, 0.f, z1);
		else
		{
			const float c0t = rt * std::cos(t0);
			const float s0t = rt * std::sin(t0);
			const float c1t = rt * std::cos(t1);
			const float s1t = rt * std::sin(t1);
			pushTri(soup, c0b, s0b, z0, c1b, s1b, z0, c1t, s1t, z1);
			pushTri(soup, c0b, s0b, z0, c1t, s1t, z1, c0t, s0t, z1);
		}
		pushTri(soup, 0.f, 0.f, z0, c1b, s1b, z0, c0b, s0b, z0);
	}
	if (!sharpApex && rt > 1e-6f)
		for (int i = 0; i < segments; ++i)
		{
			const float t0 = static_cast<float>(2.0 * kPi * i / segments);
			const float t1 = static_cast<float>(2.0 * kPi * (i + 1) / segments);
			const float c0t = rt * std::cos(t0);
			const float s0t = rt * std::sin(t0);
			const float c1t = rt * std::cos(t1);
			const float s1t = rt * std::sin(t1);
			pushTri(soup, 0.f, 0.f, z1, c0t, s0t, z1, c1t, s1t, z1);
		}
	return soup;
}

std::vector<float> makeSphereTriangleSoup(double radiusMm, int segments, int rings)
{
	segments = clampInt(segments, 8, 128);
	rings = clampInt(rings, 4, 64);
	const float r = static_cast<float>(radiusMm);
	std::vector<float> soup;
	for (int j = 0; j < rings; ++j)
	{
		const double phi0 = kPi * j / rings - kPi * 0.5;
		const double phi1 = kPi * (j + 1) / rings - kPi * 0.5;
		for (int i = 0; i < segments; ++i)
		{
			const double th0 = 2.0 * kPi * i / segments;
			const double th1 = 2.0 * kPi * (i + 1) / segments;
			auto sph = [&](double phi, double th) {
				const float cp = static_cast<float>(std::cos(phi));
				return std::array<float, 3>{
					r * cp * static_cast<float>(std::cos(th)),
					r * cp * static_cast<float>(std::sin(th)),
					r * static_cast<float>(std::sin(phi)) };
			};
			const auto p00 = sph(phi0, th0);
			const auto p10 = sph(phi0, th1);
			const auto p11 = sph(phi1, th1);
			const auto p01 = sph(phi1, th0);
			pushTri(soup, p00[0], p00[1], p00[2], p10[0], p10[1], p10[2], p11[0], p11[1], p11[2]);
			pushTri(soup, p00[0], p00[1], p00[2], p11[0], p11[1], p11[2], p01[0], p01[1], p01[2]);
		}
	}
	return soup;
}

std::vector<float> makePrimitiveTriangleSoup(const PrimitiveMeshParams& params, const PrimitiveMeshQuality& quality)
{
	PrimitiveMeshQuality q = quality;
	q.segments = clampInt(q.segments, 8, 128);
	q.rings = clampInt(q.rings, 4, 64);
	switch (params.kind)
	{
	case PrimitiveKind::Box:
		return makeBoxTriangleSoup(params.lengthMm, params.widthMm, params.heightMm);
	case PrimitiveKind::Cylinder:
		return makeCylinderTriangleSoup(params.radiusMm, params.heightMm, q.segments);
	case PrimitiveKind::Cone:
		return makeConeTriangleSoup(params.radiusMm, params.radiusTopMm, params.heightMm, q.segments);
	case PrimitiveKind::Sphere:
		return makeSphereTriangleSoup(params.radiusMm, q.segments, q.rings);
	}
	return {};
}

} // namespace BackendPrimitiveGeometry
