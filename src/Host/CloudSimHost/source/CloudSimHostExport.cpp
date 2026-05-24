#include "CloudSimCoreVersion.h"
#include "CloudSimHost.h"
#include "IRenderView.h"

// API 版本不一致则 nullptr，防跨 DLL 契约漂移
extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::host::createHostRenderViewFactory().release();
}
