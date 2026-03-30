#pragma once

#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "data_global.h"

class BackendDataBase;

/// 后端数据注册表（单例）：按 id 管理共享的 BackendDataBase，线程安全增删查。
class DATA_EXPORT BackendDataManager
{
public:
	BackendDataManager() = default;
	static BackendDataManager& instance();

	bool registerData(const std::shared_ptr<BackendDataBase>& data);
	/// Removes the shared object for \a id from the registry (reference counts drop; last owner may destroy it).
	bool unregisterData(const std::string& id);
	bool contains(const std::string& id) const;
	std::shared_ptr<BackendDataBase> getData(const std::string& id) const;
	std::vector<std::shared_ptr<BackendDataBase>> listData() const;
	void clear();

private:
	BackendDataManager(const BackendDataManager&) = delete;
	BackendDataManager& operator=(const BackendDataManager&) = delete;

private:
	mutable std::mutex m_mutex;
	std::unordered_map<std::string, std::shared_ptr<BackendDataBase>> m_records;
};

