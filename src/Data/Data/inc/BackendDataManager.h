#ifndef DATA_BACKENDDATAMANAGER_H
#define DATA_BACKENDDATAMANAGER_H

/// @file BackendDataManager.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端数据注册表（单例）：按 id 管理共享的 BackendDataBase；读写锁保护索引与层级图，多读并发友好

#include "data_global.h"

#include "BackendDataBase.h"
#include "BackendFollowMath.h"
#include "BackendHierarchyChange.h"

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct BackendSnapshot
{
	std::string id;
	std::string name;
	std::string className;
	BackendVec3 pose{};
	BackendVec3 rotation{};
	BackendColor color{};
	BackendMat4 worldMatrix = BackendMat4::identity();
	std::vector<std::string> parents;
	std::vector<std::string> children;
};

struct BackendBaselineMetrics
{
	std::size_t objectCount = 0U;
	std::size_t edgeCount = 0U;
	double listDataMs = 0.0;
	double descendantsMs = 0.0;
	double followLookupMs = 0.0;
	double snapshotMs = 0.0;
};

/// 后端数据注册表（单例）：按 id 管理共享的 BackendDataBase；读写锁保护索引与层级图，多读并发友好
class DATA_EXPORT BackendDataManager
{
public:
	BackendDataManager() = default;
	static BackendDataManager& instance();

	bool registerData(const std::shared_ptr<BackendDataBase>& data);
	/// 按 id 移除注册对象（引用归零可销毁）
	bool unregisterData(const std::string& id);
	bool contains(const std::string& id) const;
	std::shared_ptr<BackendDataBase> getData(const std::string& id) const;
	std::vector<std::shared_ptr<BackendDataBase>> listData() const;
	std::vector<std::shared_ptr<BackendDataBase>> findByName(const std::string& name) const;
	std::vector<std::shared_ptr<BackendDataBase>> findByClass(const std::string& className) const;
	std::vector<std::shared_ptr<BackendDataBase>> findByComponent(const std::string& componentType) const;

	bool attachChild(const std::string& parentId, const std::string& childId);
	bool setParent(const std::string& childId, const std::string& parentId);
	bool detachChild(const std::string& parentId, const std::string& childId);
	bool detachAllParents(const std::string& childId);
	std::vector<std::string> parentsOf(const std::string& id) const;
	/// 显式主父；无记录时回退字母序首个并 warn
	std::string primaryParentOf(const std::string& childId) const;
	std::vector<std::string> childrenOf(const std::string& id) const;
	std::vector<std::string> ancestorsOf(const std::string& id) const;
	std::vector<std::string> descendantsOf(const std::string& id) const;
	std::vector<std::string> subtreeIds(const std::string& rootId) const;
	std::vector<std::string> rootIds() const;
	std::vector<std::string> topoOrder() const;
	std::vector<std::pair<std::string, std::string>> listEdges() const;
	bool wouldCreateCycle(const std::string& parentId, const std::string& childId) const;
	bool validateGraph(std::string* errMsg = nullptr) const;
	std::vector<BackendSnapshot> takeSnapshot() const;
	BackendBaselineMetrics collectBaselineMetrics(const std::string& sampleRootId = std::string()) const;
	void clear();

	/// 观察者于锁外回调（注册/移除仍持写锁）
	void addHierarchyObserver(void* key, BackendHierarchyObserver observer);
	void removeHierarchyObserver(void* key);

private:
	BackendDataManager(const BackendDataManager&) = delete;
	BackendDataManager& operator=(const BackendDataManager&) = delete;

	static void dispatchHierarchyEvents(
		const std::vector<std::pair<void*, BackendHierarchyObserver>>& observers,
		const std::vector<BackendHierarchyChangeEvent>& events);

	mutable std::shared_mutex m_mutex;
	std::unordered_map<std::string, std::shared_ptr<BackendDataBase>> m_records;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_childrenByParent;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_parentsByChild;
	std::unordered_map<std::string, std::string> m_primaryParentByChild;
	mutable std::unordered_map<std::string, std::vector<std::string>> m_subtreeCache;
	std::vector<std::pair<void*, BackendHierarchyObserver>> m_hierarchyObservers;
};

#endif // DATA_BACKENDDATAMANAGER_H
