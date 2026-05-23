#include "CloudSimCoreVersion.h"
#include "CloudSimHost.h"
#include "CloudSimCoreVersion.h"
#include "IRenderView.h"

extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::host::createHostRenderViewFactory().release();
}
