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
	osg::Quat qx;
	qx.makeRotate(ex, osg::Vec3d(1.0, 0.0, 0.0));
	osg::Quat qy;
	qy.makeRotate(ey, osg::Vec3d(0.0, 1.0, 0.0));
	osg::Quat qz;
	qz.makeRotate(ez, osg::Vec3d(0.0, 0.0, 1.0));
	return qz * qy * qx;
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
