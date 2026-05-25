#pragma once

#include "RobotInstructionModel.h"
#include "robotwidget_global.h"

#include <QHash>
#include <QString>
#include <QStringList>

#include <memory>
#include <vector>

/// 按机器人后端 id 存储指令程序
class ROBOTWIDGET_EXPORT RobotProgramStore
{
public:
	void clear();

	QStringList robotBackendIds() const;
	QStringList robotLabels() const;

	int activeInstanceIndex() const { return m_activeInstanceIndex; }
	QString activeRobotBackendId() const;

	void setActiveInstanceIndex(int index);
	void setActiveRobotBackendId(const QString& sceneBackendId);

	void setRobotInstances(const QStringList& labels, const QStringList& backendIds);

	std::vector<std::shared_ptr<RobotInstruction::Base>>& programFor(const QString& sceneBackendId);
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& programFor(const QString& sceneBackendId) const;

	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeProgram();
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& activeProgram() const;

	void setProgramFor(const QString& sceneBackendId, std::vector<std::shared_ptr<RobotInstruction::Base>> program);

	const QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>>& allPrograms() const
	{
		return m_programs;
	}

private:
	void ensureActiveProgram();

	QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>> m_programs;
	QStringList m_labels;
	QStringList m_backendIds;
	int m_activeInstanceIndex = 0;
};
