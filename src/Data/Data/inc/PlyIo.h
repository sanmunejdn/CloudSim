#ifndef DATA_PLYIO_H
#define DATA_PLYIO_H

/// @file PlyIo.h
/// @brief PLY 头扫描（点云/网格分流共用）

#include "data_global.h"

#include <iosfwd>
#include <string>

/// PLY 头扫描（点云/网格分流共用）
struct DATA_EXPORT PlyHeaderInfo
{
	bool valid = false;
	bool cgalFormatOnLine2 = false;
	bool isAscii = true;
	int vertexCount = 0;
	int faceCount = 0;
	bool hasFaceElement = false;
	bool hasUcharRgb = false;
	bool vertexHasListProperty = false;
	int ix = -1;
	int iy = -1;
	int iz = -1;
	int ir = -1;
	int ig = -1;
	int ib = -1;
};

DATA_EXPORT bool scanPlyHeader(std::istream& input, PlyHeaderInfo& out, std::string* errMsg = nullptr);
/// path：本地窄字节路径（与 QFile::encodeName / loadFromFile 一致，勿用 u8path）
DATA_EXPORT bool scanPlyHeader(const std::string& path, PlyHeaderInfo& out, std::string* errMsg = nullptr);
DATA_EXPORT bool plyFileHasTriangleFaces(const std::string& path);

#endif // DATA_PLYIO_H
