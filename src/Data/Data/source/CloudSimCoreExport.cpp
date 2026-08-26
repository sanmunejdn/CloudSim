/// @file CloudSimCoreExport.cpp
/// @brief CloudSimCore导出

#include "data_global.h"

#include "BackendManagerDataService.h"
#include "CloudSimCoreVersion.h"
#include "IDataService.h"

extern "C" DATA_EXPORT cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion)
{
	// apiVersion 为 ABI 契约，严格相等（无兼容窗）
	if (apiVersion != cloudsimCoreApiVersion())
		return nullptr;
	return makeBackendManagerDataService().release();
}

extern "C" DATA_EXPORT void cloudsimDestroyDataService(cloudsim::core::IDataService* svc)
{
	// P2/P3: 配套释放入口，跨 DLL delete 仅在共享 CRT（/MD）时安全
	delete svc;
}
