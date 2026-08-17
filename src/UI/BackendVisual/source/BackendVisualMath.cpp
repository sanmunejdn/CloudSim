/// @file BackendVisualMath.cpp
/// @brief 后端视觉

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

#include <Adapters.h>

namespace backendvisual_math
{
osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg)
{
	return engine::eulerDegToQuat(static_cast<double>(eulerDeg.x()), static_cast<double>(eulerDeg.y()),
								  static_cast<double>(eulerDeg.z()));
}

osg::Vec3f quatToEulerDeg(const osg::Quat& q)
{
	return engine::quatToEulerDegVec3f(q);
}

} // namespace backendvisual_math
