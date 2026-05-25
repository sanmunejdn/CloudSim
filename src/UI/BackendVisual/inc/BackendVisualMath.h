#pragma once

#include "backendvisual_global.h"

#include <osg/Quat>
#include <osg/Vec3f>

namespace backendvisual_math {

/// 同 OsgScene：固定轴 Tait-Bryan ZYX，qz*qy*qx
BACKENDVISUAL_EXPORT osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg);
BACKENDVISUAL_EXPORT osg::Vec3f quatToEulerDeg(const osg::Quat& q);

} // namespace backendvisual_math
