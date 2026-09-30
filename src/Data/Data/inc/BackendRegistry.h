#ifndef DATA_BACKENDREGISTRY_H
#define DATA_BACKENDREGISTRY_H

/// @file BackendRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端类型元数据：工厂、显示名、属性编辑器

#include "BackendDataBase.h"
#include "data_global.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

/// 后端类型元数据：工厂、显示名、属性编辑器

struct BackendMeta

{
	std::string className;

	std::string displayName;

	std::function<std::shared_ptr<BackendDataBase>()> factory;
};

/// className → 创建与属性面板工厂

class DATA_EXPORT BackendRegistry

{
public:
	// 公开构造供 ServiceRegistry 持有；全局唯一性由组合根 + setProcessInstance 保证
	BackendRegistry() = default;

	/// 宿主注册 ServiceRegistry 时写入，使 Data 层 instance() 与宿主同一对象
	static void setProcessInstance(BackendRegistry* registry)
	{
		processInstanceSlot() = registry;
	}

	/// 业务路径须先 setProcessInstance（组合根）
	static BackendRegistry& instance();

	void registerType(const BackendMeta& meta)

	{
		if (meta.className.empty() || !meta.factory)

		{
			return;
		}

		std::lock_guard<std::mutex> lock(m_mutex);

		m_types[meta.className] = meta;
	}

	std::shared_ptr<BackendDataBase> create(const std::string& className) const

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		const auto it = m_types.find(className);

		if (it == m_types.end() || !it->second.factory)

		{
			return nullptr;
		}

		return it->second.factory();
	}

	const BackendMeta* meta(const std::string& className) const

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		const auto it = m_types.find(className);

		if (it == m_types.end())

		{
			return nullptr;
		}

		return &it->second;
	}

	std::vector<std::string> registeredClassNames() const

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		std::vector<std::string> keys;

		keys.reserve(m_types.size());

		for (const auto& kv : m_types)

		{
			keys.push_back(kv.first);
		}

		return keys;
	}

private:
	/// 定义在 .cpp，保证跨 DLL 同一 override 槽
	static BackendRegistry*& processInstanceSlot();

	mutable std::mutex m_mutex;

	std::unordered_map<std::string, BackendMeta> m_types;
};

#endif // DATA_BACKENDREGISTRY_H
