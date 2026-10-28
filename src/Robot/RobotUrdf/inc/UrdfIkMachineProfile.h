#ifndef ROBOTURDF_URDFIKMACHINEPROFILE_H
#define ROBOTURDF_URDFIKMACHINEPROFILE_H

/// @file UrdfIkMachineProfile.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 按 URDF 包名/旁路 JSON 解析肘腕下标与球形腕强制项

#include "robot_urdf_global.h"

#include <QString>
#include <string>

namespace UrdfRobotLoader
{
enum class ROBOT_URDF_API SphericalWristForce
{
	Auto = 0,
	Yes,
	No
};

/// 白名单解析族；None 强制走数值，防非球形腕误开解析
enum class ROBOT_URDF_API AnalyticalIkFamily
{
	Auto = 0,
	SphericalWrist6R,
	None
};

struct ROBOT_URDF_API UrdfIkMachineProfile
{
	QString packageKey;
	int elbowJointIndex = -1;
	int wristJointIndex = -1;
	SphericalWristForce forceSphericalWrist = SphericalWristForce::Auto;
	AnalyticalIkFamily analyticalFamily = AnalyticalIkFamily::Auto;
	/// 空=不建议；drag|preview|run 仅导入缺省，不覆盖指令已写 profile
	std::string preferredIkProfile;
};

/// 缓存 → 旁路 ik_profile.json → 内置表 → 空 profile（下标 -1）
ROBOT_URDF_API UrdfIkMachineProfile resolveIkMachineProfile(const QString& urdfPath);

/// 与 clearEligibilityCache 一并在 URDF 热加载时调用
ROBOT_URDF_API void clearIkMachineProfileCache();

/// …/<pkg>/urdf/*.urdf → pkg；否则文件主名
ROBOT_URDF_API QString ikMachinePackageKeyFromUrdfPath(const QString& urdfPath);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFIKMACHINEPROFILE_H
