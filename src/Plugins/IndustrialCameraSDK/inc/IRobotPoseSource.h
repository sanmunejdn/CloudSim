#ifndef INDUSTRIALCAMERASDK_IROBOTPOSESOURCE_H
#define INDUSTRIALCAMERASDK_IROBOTPOSESOURCE_H

/// @file IRobotPoseSource.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 手眼标定用机器人末端位姿源

#include "CameraTypes.h"

#include <memory>
#include <string>

namespace industrial_camera
{

class INDUSTRIAL_CAMERA_SDK_EXPORT IRobotPoseSource
{
public:
	virtual ~IRobotPoseSource() = default;
	virtual bool getCurrentPose(Pose6d& out) = 0;
	virtual std::string lastError() const = 0;
};

INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<IRobotPoseSource> createManualPoseSource(const Pose6d& initial = {});
INDUSTRIAL_CAMERA_SDK_EXPORT void setManualPose(IRobotPoseSource* src, const Pose6d& pose);

struct RealRobotPoseConfig
{
	std::string host = "127.0.0.1";
	int port = 19600;
	int timeoutMs = 2000;
};

INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<IRobotPoseSource> createRealRobotPoseSource(const RealRobotPoseConfig& cfg);

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_IROBOTPOSESOURCE_H
