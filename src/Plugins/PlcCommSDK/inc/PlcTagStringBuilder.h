#ifndef PLCCOMMSDK_PLCTAGSTRINGBUILDER_H
#define PLCCOMMSDK_PLCTAGSTRINGBUILDER_H

/// @file PlcTagStringBuilder.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 生成 libplctag 属性串；失败时 outError 非空

#include "PlcCommTypes.h"

#include <string>

/// 生成 libplctag 属性串；失败时 outError 非空
bool buildPlcTagAttributeString(const PlcConnectionConfig& connection, const PlcTagSpec& tag,
								std::string* outAttributeString, std::string* outError);

#endif // PLCCOMMSDK_PLCTAGSTRINGBUILDER_H
