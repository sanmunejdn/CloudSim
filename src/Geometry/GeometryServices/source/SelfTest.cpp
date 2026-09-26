/// @file SelfTest.cpp
/// @brief 转发 PointCloudAlgorithm 自检（供 SelfTestRunner 经 DLL 调用）

#include "pch.h"

#include "SelfTest.h"

#include "point_cloud_algorithm_global.h"

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);
}

namespace GeometryServicesSelfTest
{
bool runPointCloudSelfTest(std::vector<std::string>& failures)
{
	return pclalgo::runSelfTest(failures);
}
}
