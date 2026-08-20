#ifndef PLCCOMMSDK_IPLCCOMMCLIENT_H
#define PLCCOMMSDK_IPLCCOMMCLIENT_H

/// @file IPlcCommClient.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IPlcCommClient 接口

#include "plc_comm_sdk_global.h"

#include "PlcCommTypes.h"

#include <memory>
#include <string>

class PLCCOMM_SDK_EXPORT IPlcCommClient
{
public:
	virtual ~IPlcCommClient() = default;

	virtual bool connect(const PlcConnectionConfig& config) = 0;
	virtual void disconnect() = 0;
	virtual bool isConnected() const = 0;

	virtual int addTag(const PlcTagSpec& spec) = 0;
	virtual void removeTag(int handle) = 0;

	/// 同步读；仅允许在专用工作线程调用
	virtual bool readTag(int handle, PlcTagValue& out) = 0;
	/// 同步写；仅允许在专用工作线程调用
	virtual bool writeTag(int handle, const PlcTagValue& value) = 0;

	virtual std::string lastError() const = 0;
};

PLCCOMM_SDK_EXPORT std::unique_ptr<IPlcCommClient> createPlcCommClient();

#endif // PLCCOMMSDK_IPLCCOMMCLIENT_H
