/// @file DeviceProgramStore.cpp
/// @brief 设备程序存储

#include "DeviceProgramStore.h"

#include "RobotInstructionFactory.h"

#include <QJsonArray>

std::vector<std::shared_ptr<RobotInstruction::Base>> DeviceProgramStore::s_empty;

void DeviceProgramStore::clear()
{
	m_programs.clear();
}

std::vector<std::shared_ptr<RobotInstruction::Base>>& DeviceProgramStore::stepsMut(const QString& deviceBackendId)
{
	return m_programs[deviceBackendId];
}

const std::vector<std::shared_ptr<RobotInstruction::Base>>& DeviceProgramStore::steps(const QString& deviceBackendId) const
{
	const auto it = m_programs.constFind(deviceBackendId);
	return it == m_programs.cend() ? s_empty : it.value();
}

bool DeviceProgramStore::hasProgram(const QString& deviceBackendId) const
{
	const auto it = m_programs.constFind(deviceBackendId);
	return it != m_programs.cend() && !it.value().empty();
}

QStringList DeviceProgramStore::deviceIds() const
{
	return m_programs.keys();
}

QJsonObject DeviceProgramStore::toProjectJson() const
{
	QJsonObject root;
	QJsonObject devices;
	for (auto it = m_programs.constBegin(); it != m_programs.constEnd(); ++it)
	{
		nlohmann::json arr = nlohmann::json::array();
		for (const auto& step : it.value())
		{
			if (step)
			{
				arr.push_back(RobotInstruction::toJson(*step));
			}
		}
		devices.insert(it.key(), QString::fromStdString(arr.dump()));
	}
	root.insert(QStringLiteral("devicePrograms"), devices);
	return root;
}

bool DeviceProgramStore::fromProjectJson(const QJsonObject& root, QString* err)
{
	m_programs.clear();
	const QJsonObject devices = root.value(QStringLiteral("devicePrograms")).toObject();
	for (auto it = devices.begin(); it != devices.end(); ++it)
	{
		const QString dump = it.value().toString();
		if (dump.isEmpty())
		{
			continue;
		}
		try
		{
			const nlohmann::json arr = nlohmann::json::parse(dump.toStdString());
			std::string parseErr;
			auto list = RobotInstruction::createListFromJson(arr, &parseErr);
			if (!parseErr.empty() && err)
			{
				*err = QString::fromStdString(parseErr);
				return false;
			}
			m_programs.insert(it.key(), std::move(list));
		}
		catch (const std::exception& ex)
		{
			if (err)
			{
				*err = QString::fromUtf8(ex.what());
			}
			return false;
		}
	}
	return true;
}
