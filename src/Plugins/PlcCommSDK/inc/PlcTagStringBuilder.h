#ifndef PLCCOMMSDK_PLCTAGSTRINGBUILDER_H
#define PLCCOMMSDK_PLCTAGSTRINGBUILDER_H

/// @file PlcTagStringBuilder.h
/// @brief 生成 libplctag 属性串；失败时 outError 非空

#include "PlcCommTypes.h"

#include <string>

/// 生成 libplctag 属性串；失败时 outError 非空
bool buildPlcTagAttributeString(const PlcConnectionConfig& connection, const PlcTagSpec& tag,
								std::string* outAttributeString, std::string* outError);

#endif // PLCCOMMSDK_PLCTAGSTRINGBUILDER_H
