#pragma once

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include "widget_global.h"

#include "DocumentHost.h"
#include "IRobotUrdfImportContext.h"
#include "IPerLinkRobotStateAccessor.h"

namespace cloudsim::core {
class EventHub;
}
#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"

#include <string>
#include <unordered_set>

class QTabWidget;
class MeshBackendData;
class BackendSceneDocumentFacade;
class IRobotBackendPoseSink;

namespace osg {
class Group;
class MatrixTransform;
template<class T>
class ref_ptr;
}

/// 单文档页：宿主层 DocumentHost + 机器人仿真元数据（IRobotSimulationDocument）
class WIDGET_EXPORT DocumentPage : public cloudsim::host::DocumentHost,
								  public IRobotSimulationDocument,
								  public cloudsim::host::IRobotUrdfImportContext,
								  public cloudsim::host::IPerLinkKinematicsHost,
								  public cloudsim::host::IPerLinkRobotStateAccessor
{
	Q_OBJECT

public:
	explicit DocumentPage(QTabWidget* parentTabs, cloudsim::core::EventHub& events);
	~DocumentPage() override = default;

	void invalidateFollowReverseIndex() { followReverseIndex().invalidate(); }

	void markFollowAttachmentDirtyFromBackendMove(const QString& seedBackendId);

	/// 可见性委托（避免 MainWindowSelectionService 直接 include BackendSceneDocumentFacade.h）
	void setBackendVisible(const QString& backendId, bool visible);
	void setBackendsVisible(const QStringList& backendIds, bool visible);

	void setHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath,
		const QStringList& revoluteJointNames,
		const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransforms,
		const QString& robotBackendId,
		osg::Group* robotAssembly);

	void appendHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath,
		const QStringList& revoluteJointNamesUnprefixed,
		const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys,
		const QString& robotSceneBackendId,
		const QString& jointPrefixRootOverride = QString()) override;

	osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const override;

	int robotKinematicInstanceCount() const override;
	int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const override;

	QString robotSceneBackendIdForInstance(int instanceIndex) const;
	QString robotFrameWorldReferenceBackendId(int instanceIndex) const;
	QString robotDisplayLabelForInstance(int instanceIndex) const;
	QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const;
	void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad, QVector<double>& upperRad) const;
	int robotJointOffsetInAggregatedVector(int instanceIndex) const;

	QString robotUrdfAbsolutePathForInstance(int instanceIndex) const override;
	int robotRevoluteJointCountForInstance(int instanceIndex) const override;
	QString robotJointKeyPrefixForInstance(int instanceIndex) const override;
	bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const override;
	bool robotPerLinkKinematicsForInstance(int instanceIndex, RobotPerLinkKinematicsSlice& out) const override;
	bool robotPerLinkKinematicsDtoForInstance(int instanceIndex, cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const override;

	QString robotSceneBackendId() const { return m_robotSceneBackendId; }

	osg::Group* robotSceneRoot() const { return nullptr; }

	void clearRobotSimulationContext();
	void clearRobotSimulationIfContains(const QString& removedBackendId);
	bool hasRobotSimulationContext() const override;
	bool hasRobotKinematicsBind() const override;
	const QString& robotUrdfAbsolutePath() const override { return m_robotUrdfAbsolutePath; }
	const QStringList& robotRevoluteJointNames() const override { return m_robotRevoluteJointNames; }

	const QHash<QString, QString>& robotLinkNameToBackendId() const override;
	const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const override;
	const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const override;

	/// DTO 版本（Widget 优先调用，避免 osg 依赖）
	QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0Dto() const override;
	QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBindDto() const override;
	bool robotUrdfMeshVerticesInLinkFrame() const override;

	QString robotImportParentId() const;
	QStringList robotLinkBackendIds() const;
	/// 树/OSG 选择：per-link 连杆 id 归并到机器人 scene 根 id
	QString selectionRootBackendId(const QString& backendId) const;
	/// per-link 机器人 gizmo 挂在根连杆 mesh（scene 根无 OSG 分支）
	QString robotGizmoAnchorBackendId(const QString& backendId) const;
	/// 按当前场景位姿反解 bind 表 M0（M = M0·inv(T0)·Tq·P，勿把含 P 的世界矩阵直接写入 M0）
	void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) override;
	/// per-link 机器人对象 gizmo：由锚点连杆世界位姿反解 basePlacement 并 FK 全连杆
	bool applyPerLinkRobotFkFromGizmoAnchor(
		int instanceIndex,
		const QString& anchorLinkBackendId,
		const QVector<double>& jointAnglesRad) override;
	QString robotJointPrefixRoot() const;
	const QVector<double>& robotJointLowerRad() const { return m_robotJointLowerRad; }
	const QVector<double>& robotJointUpperRad() const { return m_robotJointUpperRad; }

	BackendDataManager* robotBackendManagerForKinematics() override { return &DocumentHost::backend(); }
	BackendDataManager& backend() override { return DocumentHost::backend(); }

	void notifyRobotKinematicsAppliedToScene() override;

	BackendDataManager& urdfImportBackend() override { return DocumentHost::backend(); }
	IRobotSimulationDocument* urdfImportRobotSimulationDocument() override { return this; }
	IRobotBackendPoseSink* urdfImportScenePoseSink() override;
	bool urdfImportLoadLinkMeshIntoScene(const MeshBackendData& mesh, QString* errorMessage = nullptr) override
	{
		return loadUrdfLinkMeshIntoScene(mesh, errorMessage);
	}
	void urdfImportSetBackendParent(const std::string& childBackendId, const std::string& parentBackendId) override
	{
		syncSceneBackendParent(childBackendId, parentBackendId);
	}
	void urdfImportClearStagingGeometry() override { clearStagingGeometry(); }
	void urdfImportFocusCameraOnBackend(const std::string& backendId) override { focusSceneCameraOnBackend(backendId); }
	QMap<QString, QString>& urdfImportBackendSourcePath() override { return DocumentHost::backendSourcePath(); }

	/// IPerLinkRobotStateAccessor
	cloudsim::host::PerLinkRobotStateSnapshot extractPerLinkStateSnapshot(int instanceIndex) const override;
	void applyPerLinkFkResult(const cloudsim::host::PerLinkRobotFkResult& result) override;
	QMap<QString, QString>& urdfImportBackendSourceType() override { return DocumentHost::backendSourceType(); }
	QMap<QString, QString>& urdfImportBackendParentId() override { return DocumentHost::backendParentId(); }

	void setRobotPerLinkKinematicsBinding(const QString& importKey,
		const QHash<QString, QString>& linkNameToBackendId,
		const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
		const QHash<QString, osg::Matrixd>& outerWorldAtBindByBackendId,
		bool meshVerticesInLinkFrame = false) override;

	int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot = nullptr) const;

	void setRobotBasePlacementWorldForInstance(int instanceIndex, const osg::Matrixd& placementWorld) override;

	void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId, const osg::Matrixd& world);

	const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const;
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) override;
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
		osg::Matrixd basePlacementWorld;
		bool meshVerticesInLinkFrame = false;
		RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
	};

	void rebuildHierarchicalRobotAggregates();
	void rebuildPerLinkLegacyAggregates();

	QString m_robotImportParentId;
	QHash<QString, QString> m_robotLinkNameToBackendId;
	QHash<QString, osg::Matrixd> m_robotFkMeshWorldT0;
	QHash<QString, osg::Matrixd> m_robotOuterWorldAtBind;
	bool m_robotUrdfMeshVerticesInLinkFrame = false;

	QVector<HierarchicalRobotInstance> m_hierarchicalRobots;

	QString m_robotUrdfAbsolutePath;
	QStringList m_robotRevoluteJointNames;
	QVector<double> m_robotJointLowerRad;
	QVector<double> m_robotJointUpperRad;
	QHash<QString, osg::MatrixTransform*> m_robotJointTransforms;
	QString m_robotSceneBackendId;
};
