#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "data_global.h"

// 几何缓冲 Base64（RFC 4648）：对二进制块编码解码，供工程内嵌 float 缓冲等使用。

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
