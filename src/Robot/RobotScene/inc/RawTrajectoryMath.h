#pragma once

#include "RawTrajectory.h"

namespace RobotInstruction
{

constexpr double rawTrajectoryPi();

Vec3 normalizeVec(const Vec3& v);
Vec3 crossVec(const Vec3& a, const Vec3& b);
Vec3 eulerFromFrame(const Vec3& zAxis, const Vec3& xHint);
void resampleTrajectory(RawTrajectory& traj, double stepMm);

} // namespace RobotInstruction
