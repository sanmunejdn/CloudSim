#ifndef GEOMETRYSERVICES_MESHBOOLEAN_H
#define GEOMETRYSERVICES_MESHBOOLEAN_H

/// @file MeshBoolean.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 两三角 soup（世界坐标 mm，9 float/三角）布尔运算；输出新 soup

#include "geometry_services_global.h"

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
GEOMETRY_SERVICES_EXPORT bool compute(const std::vector<float>& targetSoup, const std::vector<float>& toolSoup, MeshBooleanOp op,
						 std::vector<float>& outSoup, std::string* errMsg = nullptr);

/// 开发/CI 自检：box-cylinder 差集
GEOMETRY_SERVICES_EXPORT bool runSelfTest(std::string* errMsg = nullptr);
} // namespace MeshBoolean

#endif // GEOMETRYSERVICES_MESHBOOLEAN_H
