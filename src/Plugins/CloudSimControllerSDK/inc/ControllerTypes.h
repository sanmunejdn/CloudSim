#ifndef CLOUDSIMCONTROLLERSDK_CONTROLLERTYPES_H
#define CLOUDSIMCONTROLLERSDK_CONTROLLERTYPES_H

/// @file ControllerTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器客户端 DTO（与 CONSENSUS protocolVer=1 对齐）

#include "cloudsim_controller_sdk_global.h"

#include <cstdint>
#include <string>
#include <vector>

struct CLOUDSIM_CONTROLLER_SDK_EXPORT ControllerEndpoint
{
	std::string host = "127.0.0.1";
	uint16_t port = CLOUDSIM_CONTROLLER_DEFAULT_PORT;
	int timeoutMs = 5000;
};

struct CLOUDSIM_CONTROLLER_SDK_EXPORT ControllerHelloAck
{
	int protocolVer = 1;
	int simDtMs = 16;
	int jointCount = 0;
	std::vector<std::string> jointNames;
	std::vector<double> jointLowerRad;
	std::vector<double> jointUpperRad;
	bool supportsStepPose = false;
};

struct CLOUDSIM_CONTROLLER_SDK_EXPORT ControllerStepReply
{
	int simTimeMs = 0;
	std::vector<double> actualJointRad;
	/// 与 actualJointRad 同源；后续 Distance/IMU 往此扩展
	std::vector<double> sensorJointPosition;
	bool quit = false;
	std::string quitReason;
};

#endif // CLOUDSIMCONTROLLERSDK_CONTROLLERTYPES_H
