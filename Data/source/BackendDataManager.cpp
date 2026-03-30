#include "BackendDataManager.h"
#include "BackendDataBase.h"

BackendDataManager& BackendDataManager::instance()
{
	static BackendDataManager manager;
	return manager;
}

bool BackendDataManager::registerData(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || data->id().empty())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	const auto result = m_records.emplace(data->id(), data);
	return result.second;
}

bool BackendDataManager::unregisterData(const std::string& id)
{
	if (id.empty())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	return m_records.erase(id) > 0;
}

bool BackendDataManager::contains(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_records.find(id) != m_records.end();
}

std::shared_ptr<BackendDataBase> BackendDataManager::getData(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_records.find(id);
	if (it == m_records.end())
	{
		return nullptr;
	}

	return it->second;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::listData() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> records;
	records.reserve(m_records.size());
	for (const auto& item : m_records)
	{
		records.push_back(item.second);
	}
	return records;
}

void BackendDataManager::clear()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_records.clear();
}

