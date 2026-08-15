#ifndef ROBOTSCENE_NAMEDSIGNALTABLE_H
#define ROBOTSCENE_NAMEDSIGNALTABLE_H

/// @file NamedSignalTable.h
/// @brief 命名 IO 信号定义表（Owner 自持；与 IRobotIoSink 端口解耦）

#include "robot_scene_global.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace RobotIo
{
enum class ROBOT_SCENE_API SignalKind
{
	DI = 0,
	DO,
	AI,
	AO
};

struct ROBOT_SCENE_API SignalDef
{
	std::string id;
	std::string name;
	SignalKind kind = SignalKind::DI;
	int port = 0;
	bool defaultBool = false;
	double defaultAnalog = 0.0;
	std::string description;
	/// 仿真面板可强制翻转（典型用于 DI）
	bool simForceable = true;
};

class ROBOT_SCENE_API NamedSignalTable
{
public:
	const std::vector<SignalDef>& entries() const { return m_entries; }
	std::vector<SignalDef>& entriesMut() { return m_entries; }

	void clear();
	void setEntries(std::vector<SignalDef> defs);

	const SignalDef* findById(const std::string& id) const;
	const SignalDef* findByName(const std::string& name) const;
	const SignalDef* findByPort(SignalKind kind, int port) const;

	/// 按名解析端口；无名或未命中则返回 fallbackPort
	int resolvePort(const std::string& signalName, int fallbackPort) const;

	nlohmann::json toJson() const;
	bool fromJson(const nlohmann::json& j, std::string* errMsg = nullptr);

	static std::string kindToString(SignalKind k);
	static bool kindFromString(const std::string& s, SignalKind& out);
	static std::string makeSignalId();

private:
	std::vector<SignalDef> m_entries;
};

} // namespace RobotIo

#endif // ROBOTSCENE_NAMEDSIGNALTABLE_H
