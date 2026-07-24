/// @file RobotCommSdkExports.cpp
/// @brief RobotCommSDK 工厂导出

#include "IRobotMotionClient.h"
#include "RobotMotionClientImpl.h"

std::unique_ptr<IRobotMotionClient> createRobotMotionClient()
{
	return std::make_unique<RobotMotionClientImpl>();
}
