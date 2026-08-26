#ifndef DATA_BACKENDCOMPONENTCODECREGISTRY_H
#define DATA_BACKENDCOMPONENTCODECREGISTRY_H

/// @file BackendComponentCodecRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 组件 JSON 编解码注册（工程保存/加载）

#include "BackendComponent.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <json.hpp>

#ifdef DATA_BUILD_DLL
// Data.dll 内部：直接链 RunLogger
#include "RunLogger.h"
#else
// 宿主工程无 RunLogger include 目录时走空操作兜底
namespace BackendComponentCodecRegistryDetail
{
inline void defaultWarn(const std::string&) {}
} // namespace BackendComponentCodecRegistryDetail
#endif

/// 组件 JSON 编解码注册（工程保存/加载）
class BackendComponentCodecRegistry
{
public:
	using Writer = std::function<bool(const BackendComponentPtr&, nlohmann::json&)>;
	using Reader = std::function<BackendComponentPtr(const nlohmann::json&)>;
	using WarningHook = std::function<void(const std::string&)>;
	using ComponentFactory = std::function<BackendComponentPtr()>;
	using LegacyReader = std::function<BackendComponentPtr(const nlohmann::json&)>;

	static BackendComponentCodecRegistry& instance()
	{
		static BackendComponentCodecRegistry registry;
		return registry;
	}

	void registerCodec(const std::string& type, Writer writer, Reader reader)
	{
		if (type.empty() || !writer || !reader)
		{
			return;
		}
		bool replaced = false;
		WarningHook warningHook;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			replaced = (m_codecs.find(type) != m_codecs.end());
			m_codecs[type] = Codec{std::move(writer), std::move(reader)};
			warningHook = m_warningHook;
		}
		if (replaced && warningHook)
		{
			warningHook(std::string("[BackendComponentCodecRegistry] codec replaced: ") + type);
		}
	}

	void setWarningHook(WarningHook hook)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_warningHook = std::move(hook);
	}

	nlohmann::json encodeComponent(const BackendComponentPtr& component) const
	{
		if (!component)
		{
			return nlohmann::json();
		}
		const std::string type = component->componentType();
		Codec codec{};
		bool found = false;
		WarningHook warningHook;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			const auto it = m_codecs.find(type);
			found = (it != m_codecs.end());
			if (found)
			{
				codec = it->second;
			}
			warningHook = m_warningHook;
		}
		if (!found)
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] no encoder for component type: ") + type);
			}
			return nlohmann::json();
		}
		nlohmann::json data = nlohmann::json::object();
		if (!codec.writer(component, data))
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] encoder failed for component type: ") + type);
			}
			return nlohmann::json();
		}
		return nlohmann::json{{"type", type}, {"data", std::move(data)}};
	}

	BackendComponentPtr decodeComponent(const nlohmann::json& entry) const
	{
		if (!entry.is_object())
		{
			return nullptr;
		}
		const std::string type = entry.value("type", std::string());
		if (type.empty())
		{
			return nullptr;
		}
		Codec codec{};
		bool found = false;
		WarningHook warningHook;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			const auto it = m_codecs.find(type);
			found = (it != m_codecs.end());
			if (found)
			{
				codec = it->second;
			}
			warningHook = m_warningHook;
		}
		if (!found)
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] no decoder for component type: ") + type);
			}
			return nullptr;
		}
		BackendComponentPtr decoded;
		try
		{
			decoded = codec.reader(entry.value("data", nlohmann::json::object()));
		}
		catch (const nlohmann::json::exception& ex)
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] json type error in component \"") + type +
							"\": " + ex.what());
			}
			return nullptr;
		}
		if (!decoded)
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] decoder failed for component type: ") + type);
			}
		}
		return decoded;
	}

	void registerPropertyPrefix(const std::string& prefix, const std::string& componentType, ComponentFactory factory)
	{
		if (prefix.empty() || componentType.empty() || !factory)
		{
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		m_propertyPrefixes.emplace_back(PropertyPrefix{prefix, componentType, std::move(factory)});
	}

	void registerLegacyObjectKey(const std::string& jsonKey, const std::string& componentType, LegacyReader reader)
	{
		if (jsonKey.empty() || componentType.empty() || !reader)
		{
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		m_legacyReaders[jsonKey] = LegacyEntry{componentType, std::move(reader)};
	}

	void registerDefaultPropertyRows(const std::string& componentType, ComponentFactory factory)
	{
		if (componentType.empty() || !factory)
		{
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		m_defaultPropertyRowTypes.push_back(DefaultPropertyRows{componentType, std::move(factory)});
	}

	BackendComponentPtr createForPropertyPrefix(const std::string& propertyKey) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (const PropertyPrefix& item : m_propertyPrefixes)
		{
			if (propertyKey.size() >= item.prefix.size() &&
				propertyKey.compare(0, item.prefix.size(), item.prefix) == 0)
			{
				return item.factory();
			}
		}
		return nullptr;
	}

	std::string componentTypeForPropertyPrefix(const std::string& propertyKey) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (const PropertyPrefix& item : m_propertyPrefixes)
		{
			if (propertyKey.size() >= item.prefix.size() &&
				propertyKey.compare(0, item.prefix.size(), item.prefix) == 0)
			{
				return item.componentType;
			}
		}
		return {};
	}

	BackendComponentPtr decodeLegacyObject(const std::string& jsonKey, const nlohmann::json& in) const
	{
		LegacyReader reader;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			const auto it = m_legacyReaders.find(jsonKey);
			if (it == m_legacyReaders.end())
			{
				return nullptr;
			}
			reader = it->second.reader;
		}
		return reader(in);
	}

	std::vector<std::string> legacyComponentTypes() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<std::string> types;
		types.reserve(m_legacyReaders.size());
		for (const auto& kv : m_legacyReaders)
		{
			types.push_back(kv.second.componentType);
		}
		return types;
	}

	struct DefaultPropertyRows
	{
		std::string componentType;
		ComponentFactory factory;
	};

	std::vector<DefaultPropertyRows> defaultPropertyRowFactories() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_defaultPropertyRowTypes;
	}

	void loadLegacyComponentsFromJson(const nlohmann::json& in, std::vector<BackendComponentPtr>& out) const
	{
		std::unordered_map<std::string, LegacyEntry> readers;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			readers = m_legacyReaders;
		}
		for (const auto& kv : readers)
		{
			if (!in.contains(kv.first) || !in[kv.first].is_object())
			{
				continue;
			}
			BackendComponentPtr decoded;
			try
			{
				decoded = kv.second.reader(in[kv.first]);
			}
			catch (const nlohmann::json::exception& ex)
			{
#ifdef DATA_BUILD_DLL
				RunLogger::warn(std::string("[BackendComponentCodecRegistry] json type error in legacy key \"") +
								kv.first + "\": " + ex.what());
#else
				BackendComponentCodecRegistryDetail::defaultWarn(ex.what());
#endif
				continue;
			}
			if (decoded)
			{
				out.push_back(std::move(decoded));
			}
		}
	}

private:
	struct LegacyEntry
	{
		std::string componentType;
		LegacyReader reader;
	};

	struct PropertyPrefix
	{
		std::string prefix;
		std::string componentType;
		ComponentFactory factory;
	};

	struct Codec
	{
		Writer writer;
		Reader reader;
	};

	mutable std::mutex m_mutex;
	std::unordered_map<std::string, Codec> m_codecs;
	std::vector<PropertyPrefix> m_propertyPrefixes;
	std::unordered_map<std::string, LegacyEntry> m_legacyReaders;
	std::vector<DefaultPropertyRows> m_defaultPropertyRowTypes;
	WarningHook m_warningHook;
};

#endif // DATA_BACKENDCOMPONENTCODECREGISTRY_H
