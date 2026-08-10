/// @file OccSehCall.cpp
/// @brief MSVC SEH 包装：捕获 OCCT 内 AV（Standard_Failure 捕不到）

#include "detail/OccSehCall.h"

#if defined(_MSC_VER)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace geoalgo
{
namespace detail
{
int sehCall(int (*fn)(void*), void* arg)
{
#if defined(_MSC_VER)
	// 本函数不得有带析构的 C++ 局部对象，否则 C2712
	__try
	{
		return fn ? fn(arg) : 0;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		return 0;
	}
#else
	return fn ? fn(arg) : 0;
#endif
}

} // namespace detail
} // namespace geoalgo
