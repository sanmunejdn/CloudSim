#ifndef WIDGET_DOCUMENTPAGE_H
#define WIDGET_DOCUMENTPAGE_H

/// @file DocumentPage.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 单文档页：宿主层 DocumentHost + 机器人仿真元数据（IRobotSimulationDocument）

#include "widget_global.h"

#include "DocumentHost.h"
#include "IDataService.h"
#include "IPerLinkRobotStateAccessor.h"
#include "IRobotUrdfImportContext.h"

#include <QHash>
#include <QJsonObject>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

namespace cloudsim::core
{
class EventHub;
enum class SelectionSource;
}
#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotCollisionSettings.h"

#include <string>
#include <unordered_set>
#include <vector>

#include <osg/MatrixTransform>

class QTabWidget;

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

	/// 同步视口工具栏深/浅主题
	void setViewportToolBarDarkTheme(bool dark);
	void setViewportToolBarUseChinese(bool useChinese);
	void syncViewportSidePanelToggleState(bool leftVisible, bool rightVisible);
	void setViewportObjectSelectionChecked(bool checked);

	void markFollowAttachmentDirtyFromBackendMove(const QString& seedBackendId);

	/// 可见性委托（避免 MainWindowSelectionService 直接 include BackendSceneDocumentFacade.h）
	void setBackendVisible(const QString& backendId, bool visible);
	void setBackendsVisible(const QStringList& backendIds, bool visible);

	void setHierarchicalRobotSimulationContext(const QString& urdfAbsolutePath, const QStringList& revoluteJointNames,
											   const QVector<double>& jointLowerRad,
											   const QVector<double>& jointUpperRad,
											   const QHash<QString, osg::MatrixTransform*>& jointTransforms,
											   const QString& robotBackendId, osg::Group* robotAssembly);

	void appendHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath, const QStringList& revoluteJointNamesUnprefixed,
		const QVector<double>& jointLowerRad, const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys, const QString& robotSceneBackendId,
		const QString& jointPrefixRootOverride = QString()) override;

	osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const;
	bool hasRobotJointLocalMatrix(const QString& jointName) const override;
	bool robotJointWorldMatrix(const QString& jointName, cloudsim::core::Mat4& outWorld) const override;
	bool applyRobotJointLocalMatrix(const QString& jointName, const cloudsim::core::Mat4& localColumnMajor) override;

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
	bool robotPerLinkKinematicsForInstance(int instanceIndex,
										   cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const override;

	QString robotSceneBackendId() const { return m_robotSceneBackendId; }

	osg::Group* robotSceneRoot() const { return nullptr; }

	void clearRobotSimulationContext();
	/// 打开工程覆盖当前页前：Backend + OSG + 跟随脏集 + 碰撞默认，避免旧场景节点残留
	void clearContentForProjectOpen();
	void clearRobotSimulationIfContains(const QString& removedBackendId);
	bool hasRobotSimulationContext() const override;
	bool hasRobotKinematicsBind() const override;
	const QString& robotUrdfAbsolutePath() const override { return m_robotUrdfAbsolutePath; }
	const QStringList& robotRevoluteJointNames() const override { return m_robotRevoluteJointNames; }

	const QHash<QString, QString>& robotLinkNameToBackendId() const override;
	QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0() const override;
	QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBind() const override;
	bool robotUrdfMeshVerticesInLinkFrame() const override;

	QString robotImportParentId() const;
	QStringList robotLinkBackendIds() const;
	/// 树选：per-link 连杆归并到机器人 scene 根；STEP/装配空壳父不在此处理
	QString selectionRootBackendId(const QString& backendId) const;
	/// 对象选择目标：机器人归并 + 视口拾取默认选装配根，树已选同装配子件时保留子件
	QString resolveObjectSelectionBackendId(const QString& backendId, cloudsim::core::SelectionSource source,
											const QString& currentSelectedId) const;
	/// per-link 机器人 gizmo 挂在根连杆 mesh（scene 根无 OSG 分支）
	QString robotGizmoAnchorBackendId(const QString& backendId) const;
	/// 按当前场景位姿反解 bind 表 M0（M = M0·inv(T0)·Tq·P，勿把含 P 的世界矩阵直接写入 M0）
	void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) override;
	/// per-link 机器人对象 gizmo：由锚点连杆世界位姿反解 basePlacement 并 FK 全连杆
	bool applyPerLinkRobotFkFromGizmoAnchor(int instanceIndex, const QString& anchorLinkBackendId,
											const QVector<double>& jointAnglesRad) override;
	QString robotJointPrefixRoot() const;
	const QVector<double>& robotJointLowerRad() const { return m_robotJointLowerRad; }
	const QVector<double>& robotJointUpperRad() const { return m_robotJointUpperRad; }

	BackendDataManager* robotBackendManagerForKinematics() override { return &DocumentHost::backend(); }
	/// 存量白名单：运动学 / URDF 导入 / mesh 几何仍需 BackendDataManager；新 UI 走 DocumentHost::data()
	BackendDataManager& backend() override { return DocumentHost::backend(); }
	/// 供 IRobotDocumentHost 适配层转发（DocumentPage 本身不继承 IRobotDocumentHost）
	cloudsim::core::IDataService& documentData() { return DocumentHost::data(); }
	const cloudsim::core::IDataService& documentData() const
	{
		return const_cast<DocumentPage*>(this)->DocumentHost::data();
	}
	using DocumentHost::findObject;
	using DocumentHost::listObjects;

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

	void setRobotPerLinkKinematicsBinding(const QString& importKey, const QHash<QString, QString>& linkNameToBackendId,
										  const QHash<QString, cloudsim::core::Mat4>& fkMeshWorldT0,
										  const QHash<QString, cloudsim::core::Mat4>& outerWorldAtBindByBackendId,
										  bool meshVerticesInLinkFrame = false) override;

	int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot = nullptr) const;

	void setRobotBasePlacementWorldForInstance(int instanceIndex,
											   const cloudsim::core::Mat4& placementWorld) override;
	cloudsim::core::Mat4 robotBasePlacementWorldForInstance(int instanceIndex) const;
	void setRobotExternalAxisQMm(int instanceIndex, double qMm);
	double robotExternalAxisQMm(int instanceIndex) const;
	void setRobotExternalAxisQ(int instanceIndex, const std::vector<double>& qValues);
	std::vector<double> robotExternalAxisQ(int instanceIndex) const;
	/// 工件外轴零位 W0；首次绑定时从场景根矩阵捕获
	cloudsim::core::Mat4 workpieceExternalBasePlacement(int instanceIndex, const QString& backendId) const;
	void setWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
										   const cloudsim::core::Mat4& w0);
	void ensureWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
											  const cloudsim::core::Mat4& currentWorld);
	/// 工作架相对 W0 的固定偏置；workingFrameId 空或等于 backend 时为单位阵
	cloudsim::core::Mat4 workpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId) const;
	void ensureWorkpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId,
										   const QString& workingFrameId, const cloudsim::core::Mat4& workingWorld);

	void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId,
										   const cloudsim::core::Mat4& world);

	const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const;
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) override;
	const RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) const;
	RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) override;
	RobotCollision::Settings& robotCollisionSettings() { return m_robotCollisionSettings; }
	const RobotCollision::Settings& robotCollisionSettings() const { return m_robotCollisionSettings; }

	/// 切 Tab 时暂存/恢复；打开/保存工程与侧车 ioSignalNetwork 对齐
	void setIoSignalNetworkCache(const QJsonObject& root) { m_ioSignalNetworkCache = root; }
	const QJsonObject& ioSignalNetworkCache() const { return m_ioSignalNetworkCache; }

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
		QHash<QString, cloudsim::core::Mat4> fkMeshWorldT0;
		QHash<QString, cloudsim::core::Mat4> outerWorldAtBindByBackendId;
		/// 默认单位阵：Mat4{} 全零会让 FK/跟随目标坍缩到原点
		cloudsim::core::Mat4 basePlacementWorld = cloudsim::core::PlanContextDto::identityMat4();
		/// 运行时外轴量（与 externalAxes.axes 下标对齐）；兼容旧单标量语义见 robotExternalAxisQMm
		std::vector<double> externalAxisQ;
		/// 兼容旧字段读写；与 externalAxisQ 首个 RobotBase 轴同步
		double externalAxisQMm = 0.0;
		/// 工件外轴零位 W0（backendId → 世界根）
		QHash<QString, cloudsim::core::Mat4> workpieceBasePlacementWorld;
		/// 工作架相对 W0 的局部偏置（key=boundBackendId）
		QHash<QString, cloudsim::core::Mat4> workpieceWorkingFrameOffsetByBackend;
		bool meshVerticesInLinkFrame = false;
		RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
		RobotExternal::RobotExternalAxisConfigSet externalAxes;
	};

	void rebuildHierarchicalRobotAggregates();
	void rebuildPerLinkLegacyAggregates();

	QString m_robotImportParentId;
	QHash<QString, QString> m_robotLinkNameToBackendId;
	QHash<QString, cloudsim::core::Mat4> m_robotFkMeshWorldT0;
	QHash<QString, cloudsim::core::Mat4> m_robotOuterWorldAtBind;
	bool m_robotUrdfMeshVerticesInLinkFrame = false;

	QVector<HierarchicalRobotInstance> m_hierarchicalRobots;

	RobotCollision::Settings m_robotCollisionSettings;
	QJsonObject m_ioSignalNetworkCache;

	QString m_robotUrdfAbsolutePath;
	QStringList m_robotRevoluteJointNames;
	QVector<double> m_robotJointLowerRad;
	QVector<double> m_robotJointUpperRad;
	QHash<QString, osg::MatrixTransform*> m_robotJointTransforms;
	QString m_robotSceneBackendId;
};

#endif // WIDGET_DOCUMENTPAGE_H
