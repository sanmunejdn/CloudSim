#ifndef ROBOTCOMMSDK_IROBOTMOTIONCLIENT_H
#define ROBOTCOMMSDK_IROBOTMOTIONCLIENT_H

/// @file IRobotMotionClient.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 经 RobotCommBridge 访问真实机器人反馈（不直连厂商协议）

#include "robot_comm_sdk_global.h"

#include "RobotCommTypes.h"

#include <memory>
#include <string>

class ROBOTCOMM_SDK_EXPORT IRobotMotionClient
{
public:
	virtual ~IRobotMotionClient() = default;

	virtual bool connectBridge(const RobotCommBridgeEndpoint& endpoint) = 0;
	virtual void disconnectBridge() = 0;
	virtual bool isBridgeConnected() const = 0;

	virtual bool connectRobot(const RobotCommConnectConfig& config) = 0;
	virtual bool disconnectRobot() = 0;
	virtual bool isRobotConnected() const = 0;

	virtual bool ping() = 0;
	virtual bool getFeedback(RobotFeedback& out) = 0;

	virtual std::string lastError() const = 0;
};

ROBOTCOMM_SDK_EXPORT std::unique_ptr<IRobotMotionClient> createRobotMotionClient();

#endif // ROBOTCOMMSDK_IROBOTMOTIONCLIENT_H
