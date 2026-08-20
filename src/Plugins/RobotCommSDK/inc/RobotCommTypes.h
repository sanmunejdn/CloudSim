#ifndef ROBOTCOMMSDK_ROBOTCOMMTYPES_H
#define ROBOTCOMMSDK_ROBOTCOMMTYPES_H

/// @file RobotCommTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人通讯 DTO（与 Bridge JSON / 仿真空间契约对齐）

#include "robot_comm_sdk_global.h"

#include <cstdint>
#include <string>
#include <vector>

enum class RobotCommBrand
{
	Abb = 0,
	Fanuc = 1,
	Kuka = 2
};

struct ROBOTCOMM_SDK_EXPORT RobotPose6d
{
	double positionMm[3]{0, 0, 0};
	double eulerDeg[3]{0, 0, 0}; // ZYX
};

struct ROBOTCOMM_SDK_EXPORT RobotCommBridgeEndpoint
{
	std::string host = "127.0.0.1";
	uint16_t port = 19610;
	int timeoutMs = 3000;
};

struct ROBOTCOMM_SDK_EXPORT RobotCommConnectConfig
{
	RobotCommBrand brand = RobotCommBrand::Fanuc;
	std::string robotHost = "127.0.0.1";
	uint16_t robotPort = 0; // 0=品牌默认
	std::string user;
	std::string password;
	std::string mechUnit = "ROB_1"; // ABB
	std::string fanucPoseAddr = "D751";
	std::string fanucJointAddr = "D777";
	int fanucPoseLen = 6;
	int fanucJointLen = 6;
	std::string kukaJointVar = "$AXIS_ACT";
	std::string kukaPoseVar = "$POS_ACT";
};

struct ROBOTCOMM_SDK_EXPORT RobotFeedback
{
	std::vector<double> jointRad;
	RobotPose6d toolPoseInBase;
	bool hasJoints = false;
	bool hasPose = false;
	std::string controllerState;
	int64_t timestampMs = 0;
};

#endif // ROBOTCOMMSDK_ROBOTCOMMTYPES_H
