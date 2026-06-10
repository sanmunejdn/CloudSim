#include "ParallelUtils.h"

#ifdef CGAL_LINKED_WITH_TBB
#include <tbb/parallel_for.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#endif

namespace pclalgo
{

bool ParallelUtils::s_enabled = true;
int ParallelUtils::s_threadCount = 0;

bool ParallelUtils::isParallelEnabled()
{
    return s_enabled;
}

void ParallelUtils::setParallelEnabled(bool enabled)
{
    s_enabled = enabled;
}

int ParallelUtils::getThreadCount()
{
    if (!s_enabled)
    {
        return 1;
    }
    
#ifdef CGAL_LINKED_WITH_TBB
    if (s_threadCount <= 0)
    {
        s_threadCount = tbb::info::default_concurrency();
    }
    return s_threadCount;
#else
    return 1;
#endif
}

bool ParallelUtils::isTbbAvailable()
{
#ifdef CGAL_LINKED_WITH_TBB
    return true;
#else
    return false;
#endif
}

} // namespace pclalgo