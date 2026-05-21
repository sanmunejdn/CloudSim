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

#include "BackendVisualMath.h"

osg::Quat OsgScene::eulerDegToQuat(const osg::Vec3f& eulerDeg)
{
	return backendvisual_math::eulerDegToQuat(eulerDeg);
}

osg::Vec3f OsgScene::quatToEulerDeg(const osg::Quat& q)
{
	return backendvisual_math::quatToEulerDeg(q);
}
