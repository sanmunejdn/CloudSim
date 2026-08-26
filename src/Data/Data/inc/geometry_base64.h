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
	// 工程约定「float32_le」：主机原生 float 字节序；当前仅支持 little-endian
	static_assert(sizeof(float) == 4, "float must be 32-bit");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
	static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "geometry Base64 float32_le requires little-endian host");
#elif defined(_MSC_VER) || defined(__i386__) || defined(__x86_64__) || defined(_M_X64) || defined(_M_IX86)
	// MSVC / x86 默认 little-endian
#else
#error "geometry Base64 float32_le: unsupported endianness"
#endif
	if (values.empty())
	{
		return std::string();
	}
	return geometryBase64Encode(values.data(), values.size() * sizeof(float));
}

DATA_EXPORT bool geometryBase64DecodeFloats(const std::string& base64, std::vector<float>& outFloats);

#endif // DATA_GEOMETRY_BASE64_H
