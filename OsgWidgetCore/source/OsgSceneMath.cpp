#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <cmath>

#include <osg/Matrixf>

osg::Quat OsgScene::eulerDegToQuat(const osg::Vec3f& eulerDeg)
{
	const float rx = osg::DegreesToRadians(eulerDeg.x());
	const float ry = osg::DegreesToRadians(eulerDeg.y());
	const float rz = osg::DegreesToRadians(eulerDeg.z());
	osg::Quat qx(rx, osg::Vec3(1.0f, 0.0f, 0.0f));
	osg::Quat qy(ry, osg::Vec3(0.0f, 1.0f, 0.0f));
	osg::Quat qz(rz, osg::Vec3(0.0f, 0.0f, 1.0f));
	return qz * qy * qx;
}

osg::Vec3f OsgScene::quatToEulerDeg(const osg::Quat& q)
{
	const osg::Matrixf m(q);
	float sy = std::sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0));
	bool singular = sy < 1e-6f;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	if (!singular)
	{
		x = std::atan2(m(2, 1), m(2, 2));
		y = std::atan2(-m(2, 0), sy);
		z = std::atan2(m(1, 0), m(0, 0));
	}
	else
	{
		x = std::atan2(-m(1, 2), m(1, 1));
		y = std::atan2(-m(2, 0), sy);
		z = 0.0f;
	}
	return osg::Vec3f(osg::RadiansToDegrees(x), osg::RadiansToDegrees(y), osg::RadiansToDegrees(z));
}
