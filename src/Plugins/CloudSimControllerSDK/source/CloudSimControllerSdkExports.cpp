/// @file CloudSimControllerSdkExports.cpp
/// @brief CloudSimControllerSDK 工厂导出

#include "IControllerClient.h"
#include "ControllerClientImpl.h"

std::unique_ptr<IControllerClient> createControllerClient()
{
	return std::make_unique<ControllerClientImpl>();
}
