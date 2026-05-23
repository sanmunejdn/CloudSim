#include "CloudSimCoreVersion.h"
#include "IRenderView.h"
#include "NullCoreServices.h"
#include "osgwidgetcore_global.h"

extern "C" OSGWIDGETCORE_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::core::makeNullRenderViewFactory().release();
}
