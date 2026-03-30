#pragma once

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <osg/Matrixd>

#include "widget_global.h"

#include "IRobotSimulationDocument.h"

class QTabWidget;
class OsgWidget;
#include "BackendDataManager.h"

/// 单个文档标签页：包含一个 OsgWidget 与对应的后端数据管理器，表示一个独立编辑单元。
class WIDGET_EXPORT DocumentPage : public QWidget, public IRobotSimulationDocument
{
	Q_OBJECT

public:
	explicit DocumentPage(QTabWidget* parentTabs);
	~DocumentPage() override = default;

	OsgWidget* osgWidget() const { return m_osgWidget; }
	BackendDataManager& backend() { return m_backend; }

	QMap<QString, QString>& backendSourcePath() { return m_backendSourcePath; }
	QMap<QString, QString>& backendSourceType() { return m_backendSourceType; }
	QMap<QString, QString>& backendParentId() { return m_backendParentId; }

	/// Unregisters \a rootBackendId and all descendants in the parent map from \ref backend(),
	/// and removes their entries from backendParentId, backendSourcePath, and backendSourceType.
	/// Returns the list of removed ids (parent before children in BFS order).
	QStringList removeBackendSubtree(const QString& rootBackendId);

	void setProjectFilePath(const QString& path) { m_projectFilePath = path; }
	const QString& projectFilePath() const { return m_projectFilePath; }

	/// Last URDF import: link names → backend ids, URDF path, revolute joint order, and per-joint limits (rad).
	void setRobotSimulationContext(const QString& importParentBackendId,
		const QString& urdfAbsolutePath,
		const QHash<QString, QString>& linkNameToBackendId,
		const QStringList& revoluteJointNames,
		const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad);
	/// FK mesh matrix at q=0 and outer PAT world matrix at bind (after import). Keys: link name / backend id.
	void setRobotKinematicsBind(const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
		const QHash<QString, osg::Matrixd>& outerWorldByBackendId);
	void clearRobotSimulationContext();
	void clearRobotSimulationIfContains(const QString& removedBackendId);
	bool hasRobotSimulationContext() const override;
	bool hasRobotKinematicsBind() const override;
	QString robotImportParentId() const { return m_robotImportParentId; }
	QStringList robotLinkBackendIds() const;
	const QString& robotUrdfAbsolutePath() const override { return m_robotUrdfAbsolutePath; }
	const QStringList& robotRevoluteJointNames() const override { return m_robotRevoluteJointNames; }
	const QHash<QString, QString>& robotLinkNameToBackendId() const override { return m_robotLinkNameToBackendId; }
	const QVector<double>& robotJointLowerRad() const { return m_robotJointLowerRad; }
	const QVector<double>& robotJointUpperRad() const { return m_robotJointUpperRad; }
	const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const override { return m_robotFkMeshWorldT0; }
	const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const override { return m_robotOuterWorldAtBind; }

private:
	QMap<QString, QString> m_backendSourcePath;
	QMap<QString, QString> m_backendSourceType;
	QMap<QString, QString> m_backendParentId;
	BackendDataManager m_backend;
	OsgWidget* m_osgWidget = nullptr;
	QString m_projectFilePath;
	QString m_robotImportParentId;
	QString m_robotUrdfAbsolutePath;
	QStringList m_robotRevoluteJointNames;
	QHash<QString, QString> m_robotLinkNameToBackendId;
	QVector<double> m_robotJointLowerRad;
	QVector<double> m_robotJointUpperRad;
	QHash<QString, osg::Matrixd> m_robotFkMeshWorldT0;
	QHash<QString, osg::Matrixd> m_robotOuterWorldAtBind;
};
