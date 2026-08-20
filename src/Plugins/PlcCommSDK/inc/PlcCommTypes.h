#ifndef PLCCOMMSDK_PLCCOMMTYPES_H
#define PLCCOMMSDK_PLCCOMMTYPES_H

/// @file PlcCommTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PlcCommTypes 接口

#include "plc_comm_sdk_global.h"

#include <cstdint>
#include <string>
#include <vector>

enum class PlcProtocol
{
	AbEip,
	ModbusTcp
};

struct PLCCOMM_SDK_EXPORT PlcConnectionConfig
{
	PlcProtocol protocol = PlcProtocol::AbEip;
	std::string gateway;
	uint16_t port = 502;
	std::string path;
	std::string cpu;
	/// 创建/读写等待（毫秒），libplctag 阻塞上限
	int timeoutMs = 10000;
};

struct PLCCOMM_SDK_EXPORT PlcTagSpec
{
	std::string name;
	int elemCount = 1;
	int elemSize = 4;
};

struct PLCCOMM_SDK_EXPORT PlcTagValue
{
	std::vector<uint8_t> data;
};

#endif // PLCCOMMSDK_PLCCOMMTYPES_H
