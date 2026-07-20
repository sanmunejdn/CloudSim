/// @file PluginDelegatedBackend.cpp
/// @brief PluginDelegatedBackend 实现

#include "PluginDelegatedBackend.h"

#include <json.hpp>

PluginDelegatedBackend::PluginDelegatedBackend(std::shared_ptr<IPluginBackendObject> delegate)
	: m_delegate(std::move(delegate))
{
	if (m_delegate)
	{
		setId(m_delegate->id());
		setName(m_delegate->name());
	}
}

std::string PluginDelegatedBackend::className() const
{
	return m_delegate ? m_delegate->className() : std::string();
}

bool PluginDelegatedBackend::hasGeometry() const
{
	return false;
}

BackendBoundingBox PluginDelegatedBackend::geometryBounds() const
{
	return BackendBoundingBox{};
}

std::size_t PluginDelegatedBackend::geometryElementCount() const
{
	return 0U;
}

void PluginDelegatedBackend::clearGeometry() {}

nlohmann::json PluginDelegatedBackend::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	(void)mgr;
	if (!m_delegate)
	{
		return nlohmann::json::array();
	}
	try
	{
		const nlohmann::json parsed = nlohmann::json::parse(m_delegate->propertyRowsJson());
		if (parsed.is_array())
		{
			return parsed;
		}
	}
	catch (...)
	{
	}
	return nlohmann::json::array();
}

bool PluginDelegatedBackend::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
												 const BackendDataManager* mgr)
{
	(void)mgr;
	if (!m_delegate)
	{
		if (errMsg)
		{
			*errMsg = "null plugin backend delegate";
		}
		return false;
	}
	if (!m_delegate->applyPropertyChange(key, value))
	{
		if (errMsg)
		{
			*errMsg = "plugin applyPropertyChange failed";
		}
		return false;
	}
	return true;
}
