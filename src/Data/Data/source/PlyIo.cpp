/// @file PlyIo.cpp
/// @brief Ply读写

#include "pch.h"

#include "PlyIo.h"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace
{
void setErr(std::string* errMsg, const char* text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
}

void stripUtf8Bom(std::string& line)
{
	if (line.size() >= 3U && static_cast<unsigned char>(line[0]) == 0xEFU &&
		static_cast<unsigned char>(line[1]) == 0xBBU && static_cast<unsigned char>(line[2]) == 0xBFU)
	{
		line.erase(0, 3);
	}
}

bool isFaceLikeElement(const std::string& elementName)
{
	return elementName == "face" || elementName == "polygon" || elementName == "triangle";
}

} // namespace

bool scanPlyHeader(std::istream& input, PlyHeaderInfo& out, std::string* errMsg)
{
	out = PlyHeaderInfo{};
	std::string line;
	int lineNumber = 0;
	bool inVertex = false;
	int vertexPropIndex = 0;

	while (std::getline(input, line))
	{
		if (!line.empty() && line.back() == '\r')
		{
			line.pop_back();
		}
		++lineNumber;

		if (lineNumber == 1)
		{
			stripUtf8Bom(line);
			if (line != "ply")
			{
				setErr(errMsg, "Not a PLY file (missing 'ply' signature).");
				return false;
			}
			continue;
		}
		if (line.rfind("format ", 0) == 0)
		{
			// 容忍 header 前置 comment：不再要求 format 恰在第 2 行
			out.cgalFormatOnLine2 = true;
			out.isAscii = (line.find("ascii") != std::string::npos);
		}
		if (line == "end_header")
		{
			out.valid = true;
			break;
		}
		if (line.rfind("element ", 0) == 0)
		{
			std::istringstream ls(line);
			std::string keyword;
			std::string elementName;
			std::size_t elementCount = 0;
			if (ls >> keyword >> elementName)
			{
				ls >> elementCount;
			}
			if (elementName == "vertex")
			{
				out.vertexCount = elementCount;
				inVertex = true;
				vertexPropIndex = 0;
			}
			else if (isFaceLikeElement(elementName))
			{
				out.hasFaceElement = true;
				out.faceCount = elementCount;
				inVertex = false;
			}
			else
			{
				inVertex = false;
			}
		}
		else if (inVertex && line.rfind("property ", 0) == 0)
		{
			std::istringstream ls(line);
			std::string keyword;
			std::string type;
			std::string name;
			ls >> keyword >> type;
			if (type == "list")
			{
				out.vertexHasListProperty = true;
				std::string st;
				std::string lt;
				std::string nm;
				if (!(ls >> st >> lt >> nm))
				{
					continue;
				}
				name = nm;
			}
			else if (!(ls >> name))
			{
				continue;
			}
			auto mapRgb = [&](const std::string& n, int idx)
			{
				if (n == "red" || n == "diffuse_red")
				{
					out.ir = idx;
				}
				else if (n == "green" || n == "diffuse_green")
				{
					out.ig = idx;
				}
				else if (n == "blue" || n == "diffuse_blue")
				{
					out.ib = idx;
				}
			};
			if (name == "x")
			{
				out.ix = vertexPropIndex;
			}
			else if (name == "y")
			{
				out.iy = vertexPropIndex;
			}
			else if (name == "z")
			{
				out.iz = vertexPropIndex;
			}
			mapRgb(name, vertexPropIndex);
			++vertexPropIndex;
		}
	}

	if (!out.valid)
	{
		setErr(errMsg, "PLY header missing end_header.");
		return false;
	}
	out.hasUcharRgb = (out.ir >= 0 && out.ig >= 0 && out.ib >= 0);
	return true;
}

bool scanPlyHeader(const std::string& path, PlyHeaderInfo& out, std::string* errMsg)
{
	std::ifstream s{std::filesystem::path{path}, std::ios::binary};
	if (!s)
	{
		setErr(errMsg, "Cannot open PLY file.");
		return false;
	}
	return scanPlyHeader(s, out, errMsg);
}

bool plyFileHasTriangleFaces(const std::string& path)
{
	PlyHeaderInfo info;
	if (!scanPlyHeader(path, info, nullptr))
	{
		return false;
	}
	return info.valid && info.hasFaceElement && info.faceCount > 0;
}
