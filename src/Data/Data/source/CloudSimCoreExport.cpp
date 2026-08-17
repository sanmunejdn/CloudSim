/// @file CloudSimCoreExport.cpp
/// @brief CloudSimCore导出

#include "data_global.h"

#include "CloudSimCoreVersion.h"
#include "IDataService.h"
#include "NullCoreServices.h"

extern "C" DATA_EXPORT cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion)
{
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return cloudsim::core::makeNullDataService().release();
}
