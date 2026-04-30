#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendVisualMath.h"

#include <cmath>

#include <osg/Matrixd>

namespace backendvisual_math {

osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg)
{
	const double ex = osg::DegreesToRadians(static_cast<double>(eulerDeg.x()));
	const double ey = osg::DegreesToRadians(static_cast<double>(eulerDeg.y()));
	const double ez = osg::DegreesToRadians(static_cast<double>(eulerDeg.z()));
	const double cx = std::cos(ex);
	const double sx = std::sin(ex);
	const double cy = std::cos(ey);
	const double sy = std::sin(ey);
	const double cz = std::cos(ez);
	const double sz = std::sin(ez);
	// Closed form for R = Rz(ez)*Ry(ey)*Rx(ex) (column vectors). This is the inverse of the
	// atan2/asin extraction used in quatToEulerDeg below; do not use Matrixd::rotate(axis)^n
	// composition here — OSG multiply order vs. this convention caused 180° Z errors.
	osg::Matrixd m;
	m(0, 0) = cy * cz;
	m(0, 1) = cz * sx * sy - cx * sz;
	m(0, 2) = sx * sz + cx * cz * sy;
	m(1, 0) = cy * sz;
	m(1, 1) = cx * cz + sx * sy * sz;
	m(1, 2) = cx * sy * sz - cz * sx;
	m(2, 0) = -sy;
	m(2, 1) = cy * sx;
	m(2, 2) = cx * cy;
	m(0, 3) = 0.0;
	m(1, 3) = 0.0;
	m(2, 3) = 0.0;
	m(3, 0) = 0.0;
	m(3, 1) = 0.0;
	m(3, 2) = 0.0;
	m(3, 3) = 1.0;
	return m.getRotate();
}

osg::Vec3f quatToEulerDeg(const osg::Quat& q)
{
	const osg::Matrixd m = osg::Matrixd::rotate(q);
	const double r00 = m(0, 0);
	const double r10 = m(1, 0);
	const double r20 = m(2, 0);
	const double r21 = m(2, 1);
	const double r22 = m(2, 2);

	double x, y, z;
	if (r20 < -0.999999)
	{
		y = osg::PI_2;
		x = std::atan2(m(0, 1), m(0, 2));
		z = 0.0;
	}
	else if (r20 > 0.999999)
	{
		y = -osg::PI_2;
		x = std::atan2(-m(0, 1), -m(0, 2));
		z = 0.0;
	}
	else
	{
		y = std::asin(-r20);
		x = std::atan2(r21, r22);
		z = std::atan2(r10, r00);
	}

	return osg::Vec3f(
		static_cast<float>(osg::RadiansToDegrees(x)),
		static_cast<float>(osg::RadiansToDegrees(y)),
		static_cast<float>(osg::RadiansToDegrees(z)));
}

} // namespace backendvisual_math
