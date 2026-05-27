#pragma once

#include "PlcCommTypes.h"

#include <string>

/// 生成 libplctag 属性串；失败时 outError 非空
bool buildPlcTagAttributeString(
    const PlcConnectionConfig& connection,
    const PlcTagSpec& tag,
    std::string* outAttributeString,
    std::string* outError);
