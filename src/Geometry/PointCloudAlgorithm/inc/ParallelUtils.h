#ifndef POINTCLOUDALGORITHM_PARALLELUTILS_H
#define POINTCLOUDALGORITHM_PARALLELUTILS_H

/// @file ParallelUtils.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief ParallelUtils 接口

#include "point_cloud_algorithm_global.h"

namespace pclalgo
{
class POINT_CLOUD_ALGORITHM_API ParallelUtils
{
public:
	static bool isParallelEnabled();
	static void setParallelEnabled(bool enabled);

	static int getThreadCount();
	static bool isTbbAvailable();

private:
	static bool s_enabled;
	static int s_threadCount;
};

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_PARALLELUTILS_H
