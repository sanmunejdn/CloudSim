#ifndef ROBOTSCENE_DEVICEPROGRAMSTORE_H
#define ROBOTSCENE_DEVICEPROGRAMSTORE_H

/// @file DeviceProgramStore.h
/// @brief 自定义设备程序侧车（按 backendId）

#include "robot_scene_global.h"

#include "RobotInstructionModel.h"

#include <QHash>
#include <QJsonObject>
#include <QString>
#include <memory>
#include <vector>

class ROBOT_SCENE_API DeviceProgramStore
{
public:
	void clear();
	std::vector<std::shared_ptr<RobotInstruction::Base>>& stepsMut(const QString& deviceBackendId);
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps(const QString& deviceBackendId) const;
	bool hasProgram(const QString& deviceBackendId) const;
	QStringList deviceIds() const;

	QJsonObject toProjectJson() const;
	bool fromProjectJson(const QJsonObject& root, QString* err = nullptr);

private:
	QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>> m_programs;
	static std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
};

#endif // ROBOTSCENE_DEVICEPROGRAMSTORE_H
