#pragma once

#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
	std::vector<std::shared_ptr<BackendDataBase>> findByName(const std::string& name) const;
	std::vector<std::shared_ptr<BackendDataBase>> findByClass(const std::string& className) const;
	std::vector<std::shared_ptr<BackendDataBase>> findByComponent(const std::string& componentType) const;

	bool attachChild(const std::string& parentId, const std::string& childId);
	bool detachChild(const std::string& parentId, const std::string& childId);
	bool detachAllParents(const std::string& childId);
	std::vector<std::string> parentsOf(const std::string& id) const;
	std::vector<std::string> childrenOf(const std::string& id) const;
	std::vector<std::string> ancestorsOf(const std::string& id) const;
	std::vector<std::string> descendantsOf(const std::string& id) const;
	std::vector<std::string> rootIds() const;
	std::vector<std::string> topoOrder() const;
	std::vector<std::pair<std::string, std::string>> listEdges() const;
	bool wouldCreateCycle(const std::string& parentId, const std::string& childId) const;
	bool validateGraph(std::string* errMsg = nullptr) const;
	void clear();

private:
	BackendDataManager(const BackendDataManager&) = delete;
	BackendDataManager& operator=(const BackendDataManager&) = delete;

private:
	mutable std::mutex m_mutex;
	std::unordered_map<std::string, std::shared_ptr<BackendDataBase>> m_records;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_childrenByParent;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_parentsByChild;
};

