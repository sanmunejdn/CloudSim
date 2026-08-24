#ifndef KINEMATICCORE_JOINTMOTION1D_H
#define KINEMATICCORE_JOINTMOTION1D_H

/// @file JointMotion1D.h
/// @brief 统一 1-DOF 运动副参数

#include "kinematic_core_global.h"

#include <string>

namespace kinematic_core
{
enum class JointMotionType : int
{
	Translate = 0,
	Revolute = 1
};

struct KINEMATIC_CORE_API JointMotion1D
{
	bool enabled = true;
	std::string name;
	JointMotionType motionType = JointMotionType::Translate;
	double lower = 0.0;
	double upper = 1000.0;
	bool hasLimit = false;
	double home = 0.0;
	double axis[3]{1.0, 0.0, 0.0};
	double originMm[3]{0.0, 0.0, 0.0};
	/// q 乘数：URDF prismatic 输入为米时需设为 1000
	double qScale = 1.0;
};

} // namespace kinematic_core

#endif // KINEMATICCORE_JOINTMOTION1D_H
