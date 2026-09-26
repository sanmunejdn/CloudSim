#ifndef ROBOTURDF_IKSEEDEXPAND_H
#define ROBOTURDF_IKSEEDEXPAND_H

/// @file IkSeedExpand.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 位姿 IK 多种子扩展与最短折圈选解

#include "robot_urdf_global.h"

#include <vector>

namespace UrdfRobotLoader
{
/// 末轴偏置 + J5 翻转 + 肘腕镜像；extraSeeds 追加（当前角/上一终点）
ROBOT_URDF_API std::vector<std::vector<double>> expandPoseIkSeeds(const std::vector<double>& primarySeed,
																  const std::vector<std::vector<double>>* extraSeeds =
																	  nullptr);

/// 各关节相对 ref 折到 (-π,π] 后的 L2
ROBOT_URDF_API double wrappedJointDistance(const std::vector<double>& q, const std::vector<double>& ref);

/// 候选中折圈位移最小者；空则返回空
ROBOT_URDF_API std::vector<double> selectNearestWrappedSolution(const std::vector<std::vector<double>>& candidates,
																const std::vector<double>& seedRef);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_IKSEEDEXPAND_H
