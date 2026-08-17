/// @file CloudSimHostExport.cpp
/// @brief Host 导出入口

#include "CloudSimCoreVersion.h"
#include "CloudSimHost.h"
#include "IRenderView.h"

// API 版本校验
extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory*
cloudsimCreateRenderViewFactory(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::host::createHostRenderViewFactory().release();
}
