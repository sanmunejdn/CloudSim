#ifndef DATA_MESHBOOLEAN_H
#define DATA_MESHBOOLEAN_H

/// @file MeshBoolean.h
/// @brief 两三角 soup（世界坐标 mm，9 float/三角）布尔运算；输出新 soup

#include "data_global.h"

#include <string>
#include <vector>

enum class MeshBooleanOp
{
	Difference,
	Union,
	Intersection
};

namespace MeshBoolean
{
/// 两三角 soup（世界坐标 mm，9 float/三角）布尔运算；输出新 soup
DATA_EXPORT bool compute(const std::vector<float>& targetSoup, const std::vector<float>& toolSoup, MeshBooleanOp op,
						 std::vector<float>& outSoup, std::string* errMsg = nullptr);

/// 开发/CI 自检：box-cylinder 差集
DATA_EXPORT bool runSelfTest(std::string* errMsg = nullptr);
} // namespace MeshBoolean

#endif // DATA_MESHBOOLEAN_H
