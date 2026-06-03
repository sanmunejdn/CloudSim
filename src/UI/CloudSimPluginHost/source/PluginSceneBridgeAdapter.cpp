#include "PluginSceneBridgeAdapter.h"

#include "BackendSceneDocumentFacade.h"
#include "DocumentHost.h"
#include "IBackendSceneBridge.h"

PluginSceneBridgeAdapter::PluginSceneBridgeAdapter(cloudsim::host::DocumentHost* host)
	: m_host(host)
{
}

IBackendSceneBridge* PluginSceneBridgeAdapter::bridge() const
{
	if (!m_host)
	{
		return nullptr;
	}
	return &m_host->sceneFacade().bridge();
}

bool PluginSceneBridgeAdapter::setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
	const std::array<double, 16>& columnMajor4x4)
{
	IBackendSceneBridge* b = bridge();
	if (!b)
	{
		return false;
	}
	b->setBackendRootWorldMatrixColumnMajor(backendId, columnMajor4x4);
	return true;
}

bool PluginSceneBridgeAdapter::getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
	std::array<double, 16>& outColumnMajor4x4) const
{
	const IBackendSceneBridge* b = bridge();
	if (!b)
	{
		return false;
	}
	return b->getBackendRootWorldMatrixColumnMajor(backendId, outColumnMajor4x4);
}

void PluginSceneBridgeAdapter::setBackendObjectVisible(const std::string& backendId, bool visible)
{
	if (!m_host)
	{
		return;
	}
	BackendSceneEntity ent = m_host->sceneFacade().entity(backendId);
	if (!ent.valid())
	{
		return;
	}
	ent.setVisible(visible);
}

void PluginSceneBridgeAdapter::removeBackendObjectVisual(const std::string& backendId)
{
	IBackendSceneBridge* b = bridge();
	if (!b)
	{
		return;
	}
	b->removeBackendObjectVisual(backendId);
}

bool PluginSceneBridgeAdapter::hasBackendObjectBranch(const std::string& backendId) const
{
	const IBackendSceneBridge* b = bridge();
	if (!b)
	{
		return false;
	}
	return b->hasBackendObjectBranch(backendId);
}
