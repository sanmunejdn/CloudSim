#ifndef KINEMATICCORE_MAT4OPS_H
#define KINEMATICCORE_MAT4OPS_H

/// @file Mat4Ops.h
/// @brief 4×4 列主序齐次变换

#include "kinematic_core_global.h"

namespace kinematic_core
{
KINEMATIC_CORE_API void mat4IdentityColumnMajor(double out[16]);
KINEMATIC_CORE_API void mat4CopyColumnMajor16(const double in[16], double out[16]);
KINEMATIC_CORE_API void mat4MulColumnMajor16(const double a[16], const double b[16], double out[16]);

} // namespace kinematic_core

#endif // KINEMATICCORE_MAT4OPS_H
