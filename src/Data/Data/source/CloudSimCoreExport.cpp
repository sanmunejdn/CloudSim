#include "CloudSimCoreVersion.h"
#include "IDataService.h"
#include "NullCoreServices.h"
#include "data_global.h"

extern "C" DATA_EXPORT cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::core::makeNullDataService().release();
}
