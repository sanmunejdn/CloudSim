#ifndef DATA_GEOMETRY_BASE64_H
#define DATA_GEOMETRY_BASE64_H

/// @file geometry_base64.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何缓冲 Base64（RFC 4648），工程内嵌 float 块

#include "data_global.h"

#include <cstddef>
#include <string>
#include <vector>

/// 几何缓冲 Base64（RFC 4648），工程内嵌 float 块

DATA_EXPORT std::string geometryBase64Encode(const void* bytes, std::size_t byteCount);
DATA_EXPORT bool geometryBase64Decode(const std::string& base64, std::vector<unsigned char>& outBytes);

inline std::string geometryBase64EncodeFloats(const std::vector<float>& values)
{
	if (values.empty())
	{
		return std::string();
	}
	return geometryBase64Encode(values.data(), values.size() * sizeof(float));
}

DATA_EXPORT bool geometryBase64DecodeFloats(const std::string& base64, std::vector<float>& outFloats);

#endif // DATA_GEOMETRY_BASE64_H
