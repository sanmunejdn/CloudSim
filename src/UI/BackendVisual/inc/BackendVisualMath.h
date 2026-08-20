#ifndef BACKENDVISUAL_BACKENDVISUALMATH_H
#define BACKENDVISUAL_BACKENDVISUALMATH_H

/// @file BackendVisualMath.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 同 OsgScene：固定轴 Tait-Bryan ZYX，qz*qy*qx

#include "backendvisual_global.h"

#include <osg/Quat>
#include <osg/Vec3f>

namespace backendvisual_math
{
/// 同 OsgScene：固定轴 Tait-Bryan ZYX，qz*qy*qx
BACKENDVISUAL_EXPORT osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg);
BACKENDVISUAL_EXPORT osg::Vec3f quatToEulerDeg(const osg::Quat& q);

} // namespace backendvisual_math

#endif // BACKENDVISUAL_BACKENDVISUALMATH_H
