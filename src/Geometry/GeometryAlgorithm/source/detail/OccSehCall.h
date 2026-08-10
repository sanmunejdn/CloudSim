#pragma once

namespace geoalgo
{
namespace detail
{
/// 在 SEH 中调用 fn(arg)；成功返回 fn 返回值，访问冲突等返回 0
int sehCall(int (*fn)(void*), void* arg);

} // namespace detail
} // namespace geoalgo
