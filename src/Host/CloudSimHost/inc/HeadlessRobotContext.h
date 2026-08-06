#ifndef CLOUDSIMHOST_HEADLESSROBOTCONTEXT_H
#define CLOUDSIMHOST_HEADLESSROBOTCONTEXT_H

/// @file HeadlessRobotContext.h
/// @brief Web/Headless：无 DocumentPage/OSG 时的 URDF 导入与 per-link FK 上下文

#include "cloudsim_host_global.h"

#include "IRobotUrdfImportContext.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"

#include <QHash>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>
#include <memory>
#include <vector>

class BackendDataManager;

namespace cloudsim::host
{
class DocumentHost;
/// 仅写 BackendDataManager::worldMatrix，供 FK 无 OSG 时使用
class CLOUDSIM_HOST_EXPORT BackendDataPoseSink final : public IRobotBackendPoseSink
{
public:
	explicit BackendDataPoseSink(BackendDataManager& backend);

	bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const override;
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId,
											const cloudsim::core::Mat4& worldColumnMajor) override;

private:
	BackendDataManager& m_backend;
};

/// DocumentHost(headless) 持有；实现导入上下文 + 仿真文档视图
class CLOUDSIM_HOST_EXPORT HeadlessRobotContext final : public IRobotUrdfImportContext,
														public IRobotSimulationDocument
{
public:
	explicit HeadlessRobotContext(DocumentHost& host);
	~HeadlessRobotContext() override;

	struct InstanceInfo
	{
		QString sceneRootBackendId;
		QString label;
		QString urdfPath;
		int jointCount = 0;
	};

	void clearRobotSimulationContext();
	/// 删除后端对象时：若命中场景根或任一连杆，卸掉该机器人实例（对齐 DocumentPage）
	void clearRobotSimulationIfContains(const QString& removedBackendId);
	QVector<InstanceInfo> listInstances() const;
	bool jointMetaForSceneRoot(const QString& sceneRootBackendId, QStringList& outNames, QVector<double>& outLower,
							   QVector<double>& outUpper, QVector<double>& outAngles) const;
	void recordJointAnglesForSceneRoot(const QString& sceneRootBackendId, const QVector<double>& localAnglesRad);

	/// 任意连杆/场景根 → 实例；非机器人返回 -1
	int robotInstanceIndexForBackendId(const QString& backendId, bool* outIsSceneRoot = nullptr) const;
	QString robotGizmoAnchorBackendId(const QString& backendId) const;
	/// 末端法兰连杆 backendId（坐标系 flangeLinkName）；无则回退 gizmo 锚点
	QString robotFlangeBackendId(const QString& backendId) const;
	/// gizmo 拖到锚点世界位姿后：反解 P 并 FK 全连杆（列主序 Mat4，与 pose sink 一致）
	bool applyFkFromGizmoAnchorWorld(const QString& anchorBackendId, const cloudsim::core::Mat4& anchorWorldColumnMajor,
									 QString* outError = nullptr);
	/// worldMatrix 编码与 GET /api/objects 的 Three.js 列主序一致（16 元）
	bool applyFkFromGizmoAnchorThreeJsMatrix(const QString& anchorBackendId, const QVector<double>& threeJsColMajor16,
											 QString* outError = nullptr);
	/// 拖法兰：世界位姿 → 示教 IK → 更新关节（基座 P 不变）
	bool applyIkFromFlangeThreeJsMatrix(const QString& flangeBackendId, const QVector<double>& threeJsColMajor16,
										QVector<double>* outJointAnglesRad = nullptr, QString* outError = nullptr);

	struct TcpPoseCapture
	{
		double positionMm[3]{0.0, 0.0, 0.0};
		double eulerDeg[3]{0.0, 0.0, 0.0};
		QString jointRadCsv;
		QString flangeLinkName;
		/// 场景系 TCP（与 objects.worldMatrix 同 BackendMat4 布局），供网页罗盘贴合
		BackendMat4 worldMat = BackendMat4::identity();
	};
	/// 当前关节下基座系 TCP（工具原点）示教位姿
	bool captureTcpPose(const QString& sceneRootBackendId, TcpPoseCapture& out, QString* outError = nullptr) const;

	// —— IRobotUrdfImportContext ——
	BackendDataManager& urdfImportBackend() override;
	IRobotSimulationDocument* urdfImportRobotSimulationDocument() override;
	IRobotBackendPoseSink* urdfImportScenePoseSink() override;
	bool urdfImportLoadLinkMeshIntoScene(const MeshBackendData& mesh, QString* errorMessage = nullptr) override;
	void urdfImportSetBackendParent(const std::string& childBackendId, const std::string& parentBackendId) override;
	void urdfImportClearStagingGeometry() override;
	void urdfImportFocusCameraOnBackend(const std::string& backendId) override;
	QMap<QString, QString>& urdfImportBackendSourcePath() override;
	QMap<QString, QString>& urdfImportBackendSourceType() override;
	QMap<QString, QString>& urdfImportBackendParentId() override;
	void appendHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath, const QStringList& revoluteJointNamesUnprefixed,
		const QVector<double>& jointLowerRad, const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys, const QString& robotSceneBackendId,
		const QString& jointPrefixRootOverride = QString()) override;
	void setRobotPerLinkKinematicsBinding(const QString& importKey, const QHash<QString, QString>& linkNameToBackendId,
										  const QHash<QString, cloudsim::core::Mat4>& fkMeshWorldT0,
										  const QHash<QString, cloudsim::core::Mat4>& outerWorldAtBindByBackendId,
										  bool meshVerticesInLinkFrame = false) override;
	int robotKinematicInstanceCount() const override;
	int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const override;
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) override;
	RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) override;
	void setRobotBasePlacementWorldForInstance(int instanceIndex, const cloudsim::core::Mat4& placementWorld) override;

	// —— IRobotSimulationDocument ——
	bool hasRobotSimulationContext() const override;
	bool hasRobotKinematicsBind() const override;
	const QString& robotUrdfAbsolutePath() const override;
	const QStringList& robotRevoluteJointNames() const override;
	const QHash<QString, QString>& robotLinkNameToBackendId() const override;
	QString robotUrdfAbsolutePathForInstance(int instanceIndex) const override;
	int robotRevoluteJointCountForInstance(int instanceIndex) const override;
	QString robotJointKeyPrefixForInstance(int instanceIndex) const override;
	bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const override;
	bool robotPerLinkKinematicsForInstance(int instanceIndex,
										   cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const override;
	QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0() const override;
	QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBind() const override;
	bool robotUrdfMeshVerticesInLinkFrame() const override;
	BackendDataManager* robotBackendManagerForKinematics() override;
	void notifyRobotKinematicsAppliedToScene() override;

private:
	struct HierarchicalRobotInstance
	{
		QString urdfAbsolutePath;
		QString sceneBackendId;
		QString jointKeyPrefix;
		QStringList revoluteJointNamesUnprefixed;
		QVector<double> jointLowerRad;
		QVector<double> jointUpperRad;
		bool perLinkBackends = false;
		QString perLinkImportKey;
		QHash<QString, QString> linkNameToBackendId;
		QHash<QString, cloudsim::core::Mat4> fkMeshWorldT0;
		QHash<QString, cloudsim::core::Mat4> outerWorldAtBindByBackendId;
		cloudsim::core::Mat4 basePlacementWorld = cloudsim::core::PlanContextDto::identityMat4();
		std::vector<double> externalAxisQ;
		double externalAxisQMm = 0.0;
		bool meshVerticesInLinkFrame = false;
		RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
		RobotExternal::RobotExternalAxisConfigSet externalAxes;
		QVector<double> lastLocalJointAnglesRad;
	};

	void rebuildAggregates();

	DocumentHost& m_host;
	std::unique_ptr<BackendDataPoseSink> m_poseSink;
	QVector<HierarchicalRobotInstance> m_robots;
	QString m_robotUrdfAbsolutePath;
	QStringList m_robotRevoluteJointNames;
	QHash<QString, QString> m_robotLinkNameToBackendId;
	QHash<QString, cloudsim::core::Mat4> m_robotFkMeshWorldT0;
	QHash<QString, cloudsim::core::Mat4> m_robotOuterWorldAtBind;
	bool m_robotUrdfMeshVerticesInLinkFrame = false;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HEADLESSROBOTCONTEXT_H
