#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <json.hpp>

#include "BackendComponent.h"

class BackendComponentCodecRegistry
{
public:
	using Writer = std::function<bool(const BackendComponentPtr&, nlohmann::json&)>;
	using Reader = std::function<BackendComponentPtr(const nlohmann::json&)>;
	using WarningHook = std::function<void(const std::string&)>;

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
			m_codecs[type] = Codec{ std::move(writer), std::move(reader) };
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
		return nlohmann::json{ { "type", type }, { "data", std::move(data) } };
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
		BackendComponentPtr decoded = codec.reader(entry.value("data", nlohmann::json::object()));
		if (!decoded)
		{
			if (warningHook)
			{
				warningHook(std::string("[BackendComponentCodecRegistry] decoder failed for component type: ") + type);
			}
		}
		return decoded;
	}

private:
	struct Codec
	{
		Writer writer;
		Reader reader;
	};

	mutable std::mutex m_mutex;
	std::unordered_map<std::string, Codec> m_codecs;
	WarningHook m_warningHook;
};

