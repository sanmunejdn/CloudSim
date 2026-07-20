#ifndef ROBOTWIDGET_ROBOTPROGRAMSTORE_H
#define ROBOTWIDGET_ROBOTPROGRAMSTORE_H

/// @file RobotProgramStore.h
/// @brief 按机器人后端 id 存储多程序目录

#include "robotwidget_global.h"

#include "RobotInstructionModel.h"
#include "RobotProgramCatalog.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <memory>
#include <vector>

/// 按机器人后端 id 存储多程序目录
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

	RobotInstruction::RobotProgramCatalog& catalogFor(const QString& sceneBackendId);
	const RobotInstruction::RobotProgramCatalog& catalogFor(const QString& sceneBackendId) const;

	RobotInstruction::RobotProgramCatalog& activeCatalog();
	const RobotInstruction::RobotProgramCatalog& activeCatalog() const;

	std::string activeProgramIdUtf8() const;
	void setActiveProgramIdUtf8(const std::string& programId);

	std::vector<std::shared_ptr<RobotInstruction::Base>>& programFor(const QString& sceneBackendId);
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& programFor(const QString& sceneBackendId) const;

	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeProgram();
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& activeProgram() const;

	void setProgramFor(const QString& sceneBackendId, std::vector<std::shared_ptr<RobotInstruction::Base>> program);

	const QHash<QString, RobotInstruction::RobotProgramCatalog>& allCatalogs() const { return m_catalogs; }

	/// 兼容旧接口
	const QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>>& allPrograms() const;

private:
	void ensureActiveCatalog();

	mutable QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>> m_legacyProgramsView;
	QHash<QString, RobotInstruction::RobotProgramCatalog> m_catalogs;
	QStringList m_labels;
	QStringList m_backendIds;
	int m_activeInstanceIndex = 0;
};

#endif // ROBOTWIDGET_ROBOTPROGRAMSTORE_H
