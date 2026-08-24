#ifndef KINEMATICCORE_JOINTMOTIONEVAL_H
#define KINEMATICCORE_JOINTMOTIONEVAL_H

/// @file JointMotionEval.h
/// @brief 1-DOF 运动副 q → 4×4 增量变换

#include "JointMotion1D.h"
#include "kinematic_core_global.h"

namespace kinematic_core
{
KINEMATIC_CORE_API void makeTranslateColumnMajor(double tx, double ty, double tz, double out[16]);

/// 绕 axis 过 origin 旋转；列主序 + T(-o)*R*T(+o)
KINEMATIC_CORE_API void makeRotateAboutAxisColumnMajor(double ox, double oy, double oz, double ax, double ay, double az,
													   double angleRad, double out[16]);

KINEMATIC_CORE_API void evaluateJointMotion1D(const JointMotion1D& motion, double q, double outColumnMajor[16]);

} // namespace kinematic_core

#endif // KINEMATICCORE_JOINTMOTIONEVAL_H
