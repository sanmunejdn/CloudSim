#ifndef GEOMETRYSERVICES_SELFTEST_H
#define GEOMETRYSERVICES_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief GeometryServices 对外自检（含转发 PointCloudAlgorithm）

#include "geometry_services_global.h"

#include <string>
#include <vector>

namespace GeometryServicesSelfTest
{
GEOMETRY_SERVICES_EXPORT bool runPointCloudSelfTest(std::vector<std::string>& failures);
}

#endif // GEOMETRYSERVICES_SELFTEST_H
