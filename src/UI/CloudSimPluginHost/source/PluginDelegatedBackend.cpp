/// @file PluginDelegatedBackend.cpp
/// @brief PluginDelegatedBackend 实现

#include "PluginDelegatedBackend.h"

#include "PluginPropertyBindingRegistry.h"
#include "RunLogger.h"

#include <json.hpp>

PluginDelegatedBackend::PluginDelegatedBackend(std::shared_ptr<IPluginBackendObject> delegate,
											   PluginDelegatedBackendOptions options)
	: m_delegate(std::move(delegate)), m_options(options)
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

bool PluginDelegatedBackend::hasPoseProperty() const
{
	return m_options.supportsTransform;
}

bool PluginDelegatedBackend::hasRotationProperty() const
{
	return m_options.supportsTransform;
}

nlohmann::json PluginDelegatedBackend::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	if (m_options.usePropertyBindings)
	{
		nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
		plugin_property_binding_registry::Entry entry;
		if (plugin_property_binding_registry::tryGet(className(), entry) && m_delegate)
		{
			for (const PluginPropertyBindingEntry& b : entry.bindings)
			{
				if (!b.key || !b.formatValue)
				{
					continue;
				}
				nlohmann::json row;
				row["key"] = b.key;
				row["labelEn"] = b.label ? b.label : b.key;
				row["editable"] = b.editable;
				row["value"] = b.formatValue(m_delegate.get());
				rows.push_back(std::move(row));
			}
		}
		return rows;
	}

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
		RunLogger::warn("plugin backend: propertyRowsJson parse failed; returning empty property rows");
	}
	return nlohmann::json::array();
}

bool PluginDelegatedBackend::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
												 const BackendDataManager* mgr)
{
	if (m_options.usePropertyBindings)
	{
		std::string baseErr;
		if (BackendDataBase::applyPropertyChange(key, value, &baseErr, mgr))
		{
			if (errMsg)
			{
				*errMsg = baseErr;
			}
			return true;
		}
		plugin_property_binding_registry::Entry entry;
		if (plugin_property_binding_registry::tryGet(className(), entry) && m_delegate)
		{
			for (const PluginPropertyBindingEntry& b : entry.bindings)
			{
				if (!b.key || key != b.key)
				{
					continue;
				}
				if (!b.editable || !b.applyValue)
				{
					if (errMsg)
					{
						*errMsg = "Property is read-only for this object type.";
					}
					return false;
				}
				return b.applyValue(m_delegate.get(), value.c_str(), errMsg);
			}
		}
		if (errMsg)
		{
			*errMsg = baseErr.empty() ? "Unknown property key." : baseErr;
		}
		return false;
	}

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
