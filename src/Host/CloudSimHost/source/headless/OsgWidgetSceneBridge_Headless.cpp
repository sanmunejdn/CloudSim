/// @file OsgWidgetSceneBridge_Headless.cpp
/// @brief Headless：IBackendSceneBridge 空实现，不依赖真实 OsgWidget

#include "OsgWidgetSceneBridge.h"

#include "BackendDataBase.h"

void OsgWidgetSceneBridge::setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
																const std::array<double, 16>& columnMajor4x4)
{
	(void)backendId;
	(void)columnMajor4x4;
}

bool OsgWidgetSceneBridge::getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
																std::array<double, 16>& outColumnMajor4x4) const
{
	(void)backendId;
	(void)outColumnMajor4x4;
	return false;
}

void OsgWidgetSceneBridge::setBackendObjectVisible(const std::string& backendId, bool visible)
{
	(void)backendId;
	(void)visible;
}

void OsgWidgetSceneBridge::removeBackendObjectVisual(const std::string& backendId)
{
	(void)backendId;
}

bool OsgWidgetSceneBridge::hasBackendObjectBranch(const std::string& backendId) const
{
	(void)backendId;
	return false;
}

bool OsgWidgetSceneBridge::tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
													  double& outCz) const
{
	(void)backendId;
	(void)outCx;
	(void)outCy;
	(void)outCz;
	return false;
}

void OsgWidgetSceneBridge::syncOuterPatFromBackend(const BackendDataBase& data) { (void)data; }

void OsgWidgetSceneBridge::setBackendParent(const std::string& backendId, const std::string& parentBackendId)
{
	(void)backendId;
	(void)parentBackendId;
}
