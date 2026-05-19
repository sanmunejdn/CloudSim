#pragma once

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Group>
#include <osg/ref_ptr>

#include "widget_global.h"

#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotProgramStore.h"

#include <string>
#include <unordered_set>

class QTabWidget;
class OsgWidget;

#include "BackendDataManager.h"
#include "BackendFollowReverseIndex.h"
#include "BackendHierarchyModel.h"
#include "BackendSceneDocumentFacade.h"
#include "OsgWidgetSceneBridge.h"

/// 单个文档标签页：包含一个 OsgWidget 与对应的后端数据管理器，表示一个独立编辑单元。
/// 【中文】支持动态层级法：存储层级化机器人场景图的关节 MatrixTransform 节点。
class WIDGET_EXPORT DocumentPage : public QWidget, public IRobotSimulationDocument
{
	Q_OBJECT

public:
	explicit DocumentPage(QTabWidget* parentTabs);
	~DocumentPage() override = default;

	OsgWidget* osgWidget() const { return m_osgWidget; }
	BackendDataManager& backend() { return m_backend; }
	RobotProgramStore& robotProgramStore() { return m_robotProgramStore; }
	const RobotProgramStore& robotProgramStore() const { return m_robotProgramStore; }
	BackendHierarchyModel& hierarchyModel() { return m_hierarchyModel; }
	const BackendHierarchyModel& hierarchyModel() const { return m_hierarchyModel; }

	/// Backend data + OSG scene operations for this tab (single entry for visibility/transform/remove, etc.).
	BackendSceneDocumentFacade sceneFacade()
	{
		return BackendSceneDocumentFacade(m_backend, m_sceneBridge, m_followReverseIndex, m_osgWidget);
	}
	/// Invalidate cached follow-target → followers map (call after follow bindings or subtree removal).
	void invalidateFollowReverseIndex() { m_followReverseIndex.invalidate(); }

	/// Follow-attachment graph: mark \a seedBackendId, all transitive followers, and backend children for a follow solve pass.
	void markFollowAttachmentDirtyFromBackendMove(const BackendDataManager& mgr, const std::string& seedBackendId);
	std::unordered_set<std::string>& followDirtyBackendIds() { return m_followDirtyBackendIds; }
	const std::unordered_set<std::string>& followDirtyBackendIds() const { return m_followDirtyBackendIds; }
	void clearFollowDirtyBackendIds() { m_followDirtyBackendIds.clear(); }
	/// Next \ref MainWindow::runBackendFollowSolveAndSync runs a full follow pass (e.g. robot tick, project load).
	void requestFollowSolveForced() { m_followSolveForced = true; }
	bool takeFollowSolveForced();
	bool followSolveForcedPending() const { return m_followSolveForced; }

	QMap<QString, QString>& backendSourcePath() { return m_backendSourcePath; }
	QMap<QString, QString>& backendSourceType() { return m_backendSourceType; }
	/// Legacy compatibility mirror for single-parent UI/IO paths. Prefer BackendDataManager DAG APIs.
	QMap<QString, QString>& backendParentId() { return m_backendParentId; }

	/// Unregisters \a rootBackendId and all descendants in the parent map from \ref backend(),
	/// and removes their entries from backendParentId, backendSourcePath, and backendSourceType.
	/// Returns the list of removed ids (parent before children in BFS order).
	QStringList removeBackendSubtree(const QString& rootBackendId);

	void setProjectFilePath(const QString& path) { m_projectFilePath = path; }
	const QString& projectFilePath() const { return m_projectFilePath; }

	/// 【中文】设置层级化机器人仿真上下文（新动态层级法），会清空已有机器人并只保留本台。
	/// Note: jointTransforms 使用 URDF 原始关节名；内部会加 backendId 前缀避免多机冲突。
	void setHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath,
		const QStringList& revoluteJointNames,
		const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransforms,
		const QString& robotBackendId,
		osg::Group* robotAssembly);

	/// 【中文】追加一台层级化机器人（不清除已有实例）；关节键为 \a jointTransformsPrefixedKeys（已含前缀）。
	/// 机器人网格已由 OsgWidget::addHierarchicalRobotScene 挂入场景；此处只登记运动学元数据，不持有 RobotAssembly ref_ptr。
	/// @param robotSceneBackendId 用于 \ref robotSceneBackendId()（如 TCP 世界根）；可与关节前缀根不同（每连杆后端模式下传末端连杆或基座 backend id）。
	/// @param jointPrefixRootOverride 若非空，关节展平键前缀为 \a jointPrefixRootOverride + "::"；否则为 \a robotSceneBackendId + "::"。
	void appendHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath,
		const QStringList& revoluteJointNamesUnprefixed,
		const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys,
		const QString& robotSceneBackendId,
		const QString& jointPrefixRootOverride = QString());

	/// 【中文】动态层级法：获取关节的 MatrixTransform 节点，用于直接设置关节角度。
	osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const override;

	int robotKinematicInstanceCount() const override;

	QString robotSceneBackendIdForInstance(int instanceIndex) const;
	/// per-link 模式下 sceneBackendId 可能无 OSG 节点；返回用于坐标系 overlay 世界矩阵的连杆 backend id。
	QString robotFrameWorldReferenceBackendId(int instanceIndex) const;
	QString robotDisplayLabelForInstance(int instanceIndex) const;
	QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const;
	void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad, QVector<double>& upperRad) const;
	int robotJointOffsetInAggregatedVector(int instanceIndex) const;
	int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const;

	QString robotUrdfAbsolutePathForInstance(int instanceIndex) const override;
	int robotRevoluteJointCountForInstance(int instanceIndex) const override;
	QString robotJointKeyPrefixForInstance(int instanceIndex) const override;
	bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const override;
	bool robotPerLinkKinematicsForInstance(int instanceIndex, RobotPerLinkKinematicsSlice& out) const override;

	/// 【中文】获取机器人场景的后端 ID（用于移除场景）。
	QString robotSceneBackendId() const { return m_robotSceneBackendId; }

	/// 【中文】兼容接口：机器人根节点由 OsgWidget 场景图持有；此处不再缓存 ref_ptr，避免与 OSG 双重引用。
	osg::Group* robotSceneRoot() const { return nullptr; }

	void clearRobotSimulationContext();
	void clearRobotSimulationIfContains(const QString& removedBackendId);
	bool hasRobotSimulationContext() const override;
	bool hasRobotKinematicsBind() const override;
	const QString& robotUrdfAbsolutePath() const override { return m_robotUrdfAbsolutePath; }
	const QStringList& robotRevoluteJointNames() const override { return m_robotRevoluteJointNames; }

	/// 【中文】每连杆模式：link 名 → mesh 后端 id；传统层级模式可为空。
	const QHash<QString, QString>& robotLinkNameToBackendId() const override;
	const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const override;
	const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const override;
	bool robotUrdfMeshVerticesInLinkFrame() const override;

	/// 【中文】传统接口（返回空，新架构不使用）。
	QString robotImportParentId() const;
	QStringList robotLinkBackendIds() const;
	/// Strip trailing "::" from the first instance joint key prefix (used when saving robotKinematics).
	QString robotJointPrefixRoot() const;
	const QVector<double>& robotJointLowerRad() const { return m_robotJointLowerRad; }
	const QVector<double>& robotJointUpperRad() const { return m_robotJointUpperRad; }

	BackendDataManager* robotBackendManagerForKinematics() override { return &m_backend; }

	void notifyRobotKinematicsAppliedToScene() override;

	/// Per-link URDF kinematics: link name -> mesh backend id, FK mesh world at bind, outer PAT world at bind (keys = backend id).
	void setRobotPerLinkKinematicsBinding(const QString& importKey,
		const QHash<QString, QString>& linkNameToBackendId,
		const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
		const QHash<QString, osg::Matrixd>& outerWorldAtBindByBackendId,
		bool meshVerticesInLinkFrame = false);

	const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const;
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex);
	const RobotCoordinate::RobotUserFrame* robotActiveUserFrameForInstance(int instanceIndex) const;

private:
	struct HierarchicalRobotInstance
	{
		QString urdfAbsolutePath;
		QString sceneBackendId;
		QString jointKeyPrefix;
		QStringList revoluteJointNamesUnprefixed;
		QVector<double> jointLowerRad;
		QVector<double> jointUpperRad;
		QHash<QString, osg::MatrixTransform*> jointTransformsByPrefixedKey;
		bool perLinkBackends = false;
		QString perLinkImportKey;
		QHash<QString, QString> linkNameToBackendId;
		QHash<QString, osg::Matrixd> fkMeshWorldT0;
		QHash<QString, osg::Matrixd> outerWorldAtBindByBackendId;
		bool meshVerticesInLinkFrame = false;
		RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
	};

	void rebuildHierarchicalRobotAggregates();
	void rebuildPerLinkLegacyAggregates();

	QMap<QString, QString> m_backendSourcePath;
	QMap<QString, QString> m_backendSourceType;
	QMap<QString, QString> m_backendParentId;
	BackendDataManager m_backend;
	BackendHierarchyModel m_hierarchyModel{m_backend};
	OsgWidgetSceneBridge m_sceneBridge;
	BackendFollowReverseIndex m_followReverseIndex;
	OsgWidget* m_osgWidget = nullptr;
	QString m_projectFilePath;

	// 【中文】传统烘焙法的遗留成员（为向后兼容保留，但新架构不使用）。
	QString m_robotImportParentId;
	QHash<QString, QString> m_robotLinkNameToBackendId;
	QHash<QString, osg::Matrixd> m_robotFkMeshWorldT0;
	QHash<QString, osg::Matrixd> m_robotOuterWorldAtBind;
	bool m_robotUrdfMeshVerticesInLinkFrame = false;

	// 【中文】动态层级法：可有多台机器人实例。
	QVector<HierarchicalRobotInstance> m_hierarchicalRobots;

	// 【中文】由 m_hierarchicalRobots 汇总，供关节列表 / FK / 旧接口读取。
	QString m_robotUrdfAbsolutePath;
	QStringList m_robotRevoluteJointNames;
	QVector<double> m_robotJointLowerRad;
	QVector<double> m_robotJointUpperRad;
	/// 【中文】展平键（带实例前缀）-> MatrixTransform；指针由 OsgWidget 中 PAT→RobotAssembly 场景树持有。
	QHash<QString, osg::MatrixTransform*> m_robotJointTransforms;
	/// 【中文】兼容：第一台机器人场景的后端 ID。
	QString m_robotSceneBackendId;

	RobotProgramStore m_robotProgramStore;

	std::unordered_set<std::string> m_followDirtyBackendIds;
	bool m_followSolveForced = false;
};
