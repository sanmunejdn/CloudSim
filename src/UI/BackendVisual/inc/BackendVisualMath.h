#pragma once

#include "backendvisual_global.h"

#include <osg/Quat>
#include <osg/Vec3f>

namespace backendvisual_math {

/// Same convention as OsgScene: intrinsic Tait–Bryan ZYX applied as qz * qy * qx on fixed axes.
BACKENDVISUAL_EXPORT osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg);
BACKENDVISUAL_EXPORT osg::Vec3f quatToEulerDeg(const osg::Quat& q);

} // namespace backendvisual_math
