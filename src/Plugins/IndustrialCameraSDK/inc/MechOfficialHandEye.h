#ifndef INDUSTRIALCAMERASDK_MECHOFFICIALHANDEYE_H
#define INDUSTRIALCAMERASDK_MECHOFFICIALHANDEYE_H

/// @file MechOfficialHandEye.h
/// @brief 梅卡官方手眼会话（采集期累加，求解时出候选）

#include "HandEyeTypes.h"
#include "ICamera.h"

#include <memory>
#include <string>

namespace industrial_camera
{

class INDUSTRIAL_CAMERA_SDK_EXPORT MechOfficialHandEyeSession
{
public:
	MechOfficialHandEyeSession();
	~MechOfficialHandEyeSession();

	bool begin(ICamera* camera, HandEyeMountMode mode, std::string* err);
	/// 写入当前机器人位姿并触发官方拍检；需已连接 MechEyeCamera
	bool addPoseAndDetect(const Pose6d& robotPoseMmDeg, std::string* err);
	int sampleCount() const { return sampleCount_; }
	HandEyeMethodScore calculate(std::string* err);
	void reset();

private:
	void* impl_ = nullptr;
	int sampleCount_ = 0;
	HandEyeMountMode mode_ = HandEyeMountMode::EyeInHand;
};

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_MECHOFFICIALHANDEYE_H
