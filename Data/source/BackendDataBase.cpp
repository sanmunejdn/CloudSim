#include "BackendDataBase.h"

#include "BackendPropertyRow.h"

#include <atomic>

namespace
{
std::atomic<unsigned long long> g_backendDataIdCounter{ 1ULL };
}

BackendDataBase::BackendDataBase()
	: m_id(generateId())
	, m_name("UnnamedData")
{
}

const std::string& BackendDataBase::id() const
{
	return m_id;
}

void BackendDataBase::setId(const std::string& id)
{
	if (!id.empty())
	{
		m_id = id;
	}
}

const std::string& BackendDataBase::name() const
{
	return m_name;
}

void BackendDataBase::setName(const std::string& name)
{
	if (!name.empty())
	{
		m_name = name;
	}
}

std::string BackendDataBase::generateId()
{
	const auto id = g_backendDataIdCounter.fetch_add(1ULL);
	return "backend_data_" + std::to_string(id);
}

nlohmann::json BackendDataBase::snapshotPropertyRows() const
{
	nlohmann::json rows = nlohmann::json::array();
	backend_property_json::appendRow(rows, "core.id", "ID", false, m_id);
	backend_property_json::appendRow(rows, "core.name", "Name", false, m_name);
	backend_property_json::appendRow(rows, "core.class", "Class", false, className());
	return rows;
}

bool BackendDataBase::applyPropertyChange(const std::string& /*key*/, const std::string& /*value*/, std::string* errMsg)
{
	if (errMsg)
	{
		*errMsg = "Property is read-only for this object type.";
	}
	return false;
}

