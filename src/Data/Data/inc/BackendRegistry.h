#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "BackendDataBase.h"

/// 后端类型元数据：工厂、显示名、属性编辑器
struct BackendMeta
{
	std::string className;
	std::string displayName;
	std::function<std::shared_ptr<BackendDataBase>()> factory;
	std::function<void*(BackendDataBase*)> propertyEditorFactory;
	bool supportsTransform = true;
	bool supportsVisibility = true;
};

/// className → 创建与属性面板工厂
class BackendRegistry
{
public:
	static BackendRegistry& instance()
	{
		static BackendRegistry registry;
		return registry;
	}

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
	BackendRegistry() = default;

	mutable std::mutex m_mutex;
	std::unordered_map<std::string, BackendMeta> m_types;
};
