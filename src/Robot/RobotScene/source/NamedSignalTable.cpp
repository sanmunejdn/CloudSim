/// @file NamedSignalTable.cpp
/// @brief NamedSignalTable 实现

#include "NamedSignalTable.h"

#include <atomic>
#include <cctype>

namespace RobotIo
{
namespace
{
std::string toLower(std::string s)
{
	for (char& c : s)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return s;
}
} // namespace

std::string NamedSignalTable::kindToString(const SignalKind k)
{
	switch (k)
	{
	case SignalKind::DO:
		return "DO";
	case SignalKind::AI:
		return "AI";
	case SignalKind::AO:
		return "AO";
	case SignalKind::DI:
	default:
		return "DI";
	}
}

bool NamedSignalTable::kindFromString(const std::string& s, SignalKind& out)
{
	const std::string k = toLower(s);
	if (k == "di")
	{
		out = SignalKind::DI;
		return true;
	}
	if (k == "do")
	{
		out = SignalKind::DO;
		return true;
	}
	if (k == "ai")
	{
		out = SignalKind::AI;
		return true;
	}
	if (k == "ao")
	{
		out = SignalKind::AO;
		return true;
	}
	return false;
}

std::string NamedSignalTable::makeSignalId()
{
	static std::atomic<unsigned long long> sCounter{1ULL};
	return std::string("SIG_") + std::to_string(sCounter.fetch_add(1ULL));
}

void NamedSignalTable::clear()
{
	m_entries.clear();
}

void NamedSignalTable::setEntries(std::vector<SignalDef> defs)
{
	m_entries = std::move(defs);
}

const SignalDef* NamedSignalTable::findById(const std::string& id) const
{
	if (id.empty())
	{
		return nullptr;
	}
	for (const SignalDef& s : m_entries)
	{
		if (s.id == id)
		{
			return &s;
		}
	}
	return nullptr;
}

const SignalDef* NamedSignalTable::findByName(const std::string& name) const
{
	if (name.empty())
	{
		return nullptr;
	}
	for (const SignalDef& s : m_entries)
	{
		if (s.name == name)
		{
			return &s;
		}
	}
	return nullptr;
}

const SignalDef* NamedSignalTable::findByPort(const SignalKind kind, const int port) const
{
	for (const SignalDef& s : m_entries)
	{
		if (s.kind == kind && s.port == port)
		{
			return &s;
		}
	}
	return nullptr;
}

int NamedSignalTable::resolvePort(const std::string& signalName, const int fallbackPort) const
{
	if (const SignalDef* s = findByName(signalName))
	{
		return s->port;
	}
	return fallbackPort;
}

nlohmann::json NamedSignalTable::toJson() const
{
	nlohmann::json arr = nlohmann::json::array();
	for (const SignalDef& s : m_entries)
	{
		nlohmann::json j;
		j["id"] = s.id;
		j["name"] = s.name;
		j["kind"] = kindToString(s.kind);
		j["port"] = s.port;
		j["defaultBool"] = s.defaultBool;
		j["defaultAnalog"] = s.defaultAnalog;
		j["description"] = s.description;
		j["simForceable"] = s.simForceable;
		arr.push_back(std::move(j));
	}
	nlohmann::json root;
	root["signals"] = std::move(arr);
	return root;
}

bool NamedSignalTable::fromJson(const nlohmann::json& j, std::string* errMsg)
{
	m_entries.clear();
	if (j.is_null() || (j.is_object() && j.empty()))
	{
		return true;
	}
	const nlohmann::json* arrPtr = nullptr;
	if (j.is_array())
	{
		arrPtr = &j;
	}
	else if (j.is_object() && j.contains("signals") && j["signals"].is_array())
	{
		arrPtr = &j["signals"];
	}
	else
	{
		if (errMsg)
		{
			*errMsg = "ioSignals: expected object with signals[] or array";
		}
		return false;
	}

	for (const auto& item : *arrPtr)
	{
		if (!item.is_object())
		{
			continue;
		}
		SignalDef s;
		s.id = item.value("id", std::string());
		if (s.id.empty())
		{
			s.id = makeSignalId();
		}
		s.name = item.value("name", std::string());
		SignalKind kind = SignalKind::DI;
		if (!kindFromString(item.value("kind", std::string("DI")), kind))
		{
			kind = SignalKind::DI;
		}
		s.kind = kind;
		s.port = item.value("port", 0);
		s.defaultBool = item.value("defaultBool", false);
		s.defaultAnalog = item.value("defaultAnalog", 0.0);
		s.description = item.value("description", std::string());
		s.simForceable = item.value("simForceable", kind == SignalKind::DI);
		m_entries.push_back(std::move(s));
	}
	return true;
}

} // namespace RobotIo
