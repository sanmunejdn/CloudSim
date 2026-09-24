#ifndef CLOUDSIMCONTROLLERSDK_ICONTROLLERCLIENT_H
#define CLOUDSIMCONTROLLERSDK_ICONTROLLERCLIENT_H

/// @file IControllerClient.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器 TCP 客户端（连 Host Manager）

#include "cloudsim_controller_sdk_global.h"

#include "ControllerTypes.h"

#include <memory>
#include <string>
#include <vector>

class CLOUDSIM_CONTROLLER_SDK_EXPORT IControllerClient
{
public:
	virtual ~IControllerClient() = default;

	virtual bool connectHost(const ControllerEndpoint& endpoint) = 0;
	virtual void disconnectHost() = 0;
	virtual bool isConnected() const = 0;

	virtual bool hello(int robotInstanceIndex, ControllerHelloAck& outAck) = 0;
	virtual bool step(int dtMs, const std::vector<double>& targetJointRad, ControllerStepReply& outReply) = 0;
	virtual bool goodbye() = 0;

	virtual std::string lastError() const = 0;
};

CLOUDSIM_CONTROLLER_SDK_EXPORT std::unique_ptr<IControllerClient> createControllerClient();

#endif // CLOUDSIMCONTROLLERSDK_ICONTROLLERCLIENT_H
