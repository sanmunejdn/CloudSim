/// @file RobotInstructionPropertyDto.cpp
/// @brief 指令属性 DTO

#include "RobotInstructionPropertyDto.h"

#include <json.hpp>

namespace cloudsim::host
{
QVector<core::PropertyRowDto> propertyRowsFromInstructionSnapshotJson(const nlohmann::json& rows)
{
	QVector<core::PropertyRowDto> out;
	if (!rows.is_array())
	{
		return out;
	}
	out.reserve(static_cast<int>(rows.size()));
	for (const nlohmann::json& row : rows)
	{
		if (!row.is_object())
		{
			continue;
		}
		core::PropertyRowDto dto;
		dto.key = QString::fromStdString(row.value("key", std::string()));
		dto.labelEn = QString::fromStdString(row.value("label", std::string()));
		dto.editable = row.value("editable", true);
		if (row.contains("value") && !row["value"].is_null())
		{
			if (row["value"].is_string())
			{
				dto.value = QString::fromStdString(row["value"].get<std::string>());
			}
			else if (row["value"].is_number())
			{
				dto.value = QString::number(row["value"].get<double>());
			}
			else if (row["value"].is_boolean())
			{
				dto.value = row["value"].get<bool>() ? QStringLiteral("true") : QStringLiteral("false");
			}
		}
		if (!dto.key.isEmpty())
		{
			out.push_back(dto);
		}
	}
	return out;
}

core::FeasibleMotionAxisOptionsDto
feasibleAxisOptionsFromEngine(const std::vector<std::string>& preset, const std::vector<std::string>& elbow,
							  const std::vector<std::string>& wrist, const std::vector<std::string>& arm,
							  const std::vector<std::string>& turnJ1, const std::vector<std::string>& turnJ4,
							  const std::vector<std::string>& turnJ6)
{
	auto toList = [](const std::vector<std::string>& tokens)
	{
		QStringList list;
		list.reserve(static_cast<int>(tokens.size()));
		for (const std::string& t : tokens)
		{
			list.push_back(QString::fromStdString(t));
		}
		return list;
	};
	core::FeasibleMotionAxisOptionsDto dto;
	dto.presetTokens = toList(preset);
	dto.elbowTokens = toList(elbow);
	dto.wristTokens = toList(wrist);
	dto.armTokens = toList(arm);
	dto.turnJ1Tokens = toList(turnJ1);
	dto.turnJ4Tokens = toList(turnJ4);
	dto.turnJ6Tokens = toList(turnJ6);
	return dto;
}

} // namespace cloudsim::host
