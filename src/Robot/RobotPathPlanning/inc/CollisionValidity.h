#ifndef ROBOTPATHPLANNING_COLLISIONVALIDITY_H
#define ROBOTPATHPLANNING_COLLISIONVALIDITY_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "RobotPathPlanning.h"

#include <QVector>

namespace robot_path
{
namespace detail
{

struct JointLimits
{
	std::vector<double> lowerRad;
	std::vector<double> upperRad;
};

bool loadJointLimits(const QString& urdfPath, JointLimits& out, std::string* errMsg);

bool isWithinLimits(const std::vector<double>& q, const JointLimits& lim);

/// 失败原因（限位 / 碰撞摘要）；有效时返回空
std::string describeStateInvalid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& q);

/// 更新连杆位姿并检测碰撞；world 须已含 scene 几何
bool isStateValid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& q);

bool isSegmentValid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& a,
					const std::vector<double>& b, double longestValidSegmentRad);

} // namespace detail
} // namespace robot_path

#endif
