#ifndef ROBOTSCENE_RAWTRAJECTORYMATH_H
#define ROBOTSCENE_RAWTRAJECTORYMATH_H

/// @file RawTrajectoryMath.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RawTrajectoryMath 接口

#include "RawTrajectory.h"

namespace RobotInstruction
{
constexpr double rawTrajectoryPi();

Vec3 normalizeVec(const Vec3& v);
Vec3 crossVec(const Vec3& a, const Vec3& b);
Vec3 eulerFromFrame(const Vec3& zAxis, const Vec3& xHint);
void resampleTrajectory(RawTrajectory& traj, double stepMm);

} // namespace RobotInstruction

#endif // ROBOTSCENE_RAWTRAJECTORYMATH_H
