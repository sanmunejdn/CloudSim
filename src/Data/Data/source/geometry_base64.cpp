/// @file geometry_base64.cpp
/// @brief geometry_base64 实现

#include "geometry_base64.h"

#include <cstring>

namespace
{
constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline int decodeChar(unsigned char c)
{
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c >= 'a' && c <= 'z')
		return c - 'a' + 26;
	if (c >= '0' && c <= '9')
		return c - '0' + 52;
	if (c == '+')
		return 62;
	if (c == '/')
		return 63;
	return -1;
}

} // namespace

std::string geometryBase64Encode(const void* bytes, std::size_t byteCount)
{
	if (!bytes || byteCount == 0U)
	{
		return std::string();
	}
	const auto* data = static_cast<const unsigned char*>(bytes);
	std::string out;
	out.reserve(((byteCount + 2U) / 3U) * 4U);
	for (std::size_t i = 0; i < byteCount; i += 3U)
	{
		const unsigned int b0 = data[i];
		const unsigned int b1 = (i + 1U < byteCount) ? data[i + 1U] : 0U;
		const unsigned int b2 = (i + 2U < byteCount) ? data[i + 2U] : 0U;
		const unsigned int triple = (b0 << 16) | (b1 << 8) | b2;
		out.push_back(kTable[(triple >> 18) & 63U]);
		out.push_back(kTable[(triple >> 12) & 63U]);
		if (i + 1U < byteCount)
		{
			out.push_back(kTable[(triple >> 6) & 63U]);
		}
		else
		{
			out.push_back('=');
		}
		if (i + 2U < byteCount)
		{
			out.push_back(kTable[triple & 63U]);
		}
		else
		{
			out.push_back('=');
		}
	}
	return out;
}

bool geometryBase64Decode(const std::string& base64, std::vector<unsigned char>& outBytes)
{
	outBytes.clear();
	if (base64.empty() || (base64.size() % 4U) != 0U)
	{
		return false;
	}
	const std::size_t inLen = base64.size();
	for (std::size_t i = 0; i < inLen; i += 4U)
	{
		const unsigned char c0 = static_cast<unsigned char>(base64[i]);
		const unsigned char c1 = static_cast<unsigned char>(base64[i + 1U]);
		const unsigned char c2 = static_cast<unsigned char>(base64[i + 2U]);
		const unsigned char c3 = static_cast<unsigned char>(base64[i + 3U]);
		const int a = decodeChar(c0);
		const int b = decodeChar(c1);
		if (a < 0 || b < 0)
		{
			outBytes.clear();
			return false;
		}
		int v2 = 0;
		int v3 = 0;
		if (c2 == '=')
		{
			v2 = -1;
			v3 = -1;
		}
		else
		{
			v2 = decodeChar(c2);
			if (v2 < 0)
			{
				outBytes.clear();
				return false;
			}
		}
		if (c3 == '=')
		{
			v3 = -1;
		}
		else if (v2 >= 0)
		{
			v3 = decodeChar(c3);
			if (v3 < 0)
			{
				outBytes.clear();
				return false;
			}
		}
		const unsigned int triple = (static_cast<unsigned int>(a) << 18) | (static_cast<unsigned int>(b) << 12) |
									(v2 >= 0 ? (static_cast<unsigned int>(v2) << 6) : 0U) |
									(v3 >= 0 ? static_cast<unsigned int>(v3) : 0U);
		outBytes.push_back(static_cast<unsigned char>((triple >> 16) & 255U));
		if (v2 >= 0)
		{
			outBytes.push_back(static_cast<unsigned char>((triple >> 8) & 255U));
		}
		if (v3 >= 0)
		{
			outBytes.push_back(static_cast<unsigned char>(triple & 255U));
		}
	}
	return true;
}

bool geometryBase64DecodeFloats(const std::string& base64, std::vector<float>& outFloats)
{
	outFloats.clear();
	std::vector<unsigned char> raw;
	if (!geometryBase64Decode(base64, raw) || raw.empty())
	{
		return false;
	}
	if ((raw.size() % sizeof(float)) != 0U)
	{
		return false;
	}
	outFloats.resize(raw.size() / sizeof(float));
	std::memcpy(outFloats.data(), raw.data(), raw.size());
	return true;
}
