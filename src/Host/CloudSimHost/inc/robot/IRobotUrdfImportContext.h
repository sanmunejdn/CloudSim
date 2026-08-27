#ifndef CLOUDSIMHOST_IROBOTURDFIMPORTCONTEXT_H
#define CLOUDSIMHOST_IROBOTURDFIMPORTCONTEXT_H

/// @file IRobotUrdfImportContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief URDF 导入回调

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>

class BackendDataManager;
class IRobotBackendPoseSink;
class IRobotSimulationDocument;
class MeshBackendData;

namespace osg
{
class MatrixTransform;
} // namespace osg

namespace RobotCoordinate
{
struct RobotCoordinateFrameSet;
}

namespace RobotExternal
{
struct RobotExternalAxisConfigSet;
}

namespace cloudsim::host
{
/// URDF 导入回调
class CLOUDSIM_HOST_EXPORT IRobotUrdfImportContext
{
public:
	virtual ~IRobotUrdfImportContext() = default;

	virtual BackendDataManager& urdfImportBackend() = 0;
	virtual IRobotSimulationDocument* urdfImportRobotSimulationDocument() = 0;
	virtual IRobotBackendPoseSink* urdfImportScenePoseSink() = 0;

	/// 连杆网格进场景（无 OSG 时跳过，返回 true）
	virtual bool urdfImportLoadLinkMeshIntoScene(const MeshBackendData& mesh, QString* errorMessage = nullptr) = 0;
	virtual void urdfImportSetBackendParent(const std::string& childBackendId, const std::string& parentBackendId) = 0;
	virtual void urdfImportClearStagingGeometry() = 0;
	virtual void urdfImportFocusCameraOnBackend(const std::string& backendId) = 0;

	virtual QMap<QString, QString>& urdfImportBackendSourcePath() = 0;
	virtual QMap<QString, QString>& urdfImportBackendSourceType() = 0;
	virtual QMap<QString, QString>& urdfImportBackendParentId() = 0;

	virtual void appendHierarchicalRobotSimulationContext(
		const QString& urdfAbsolutePath, const QStringList& revoluteJointNamesUnprefixed,
		const QVector<double>& jointLowerRad, const QVector<double>& jointUpperRad,
		const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys, const QString& robotSceneBackendId,
		const QString& jointPrefixRootOverride = QString()) = 0;

	virtual void setRobotPerLinkKinematicsBinding(const QString& importKey,
												  const QHash<QString, QString>& linkNameToBackendId,
												  const QHash<QString, cloudsim::core::Mat4>& fkMeshWorldT0,
												  const QHash<QString, cloudsim::core::Mat4>& outerWorldAtBindByBackendId,
												  bool meshVerticesInLinkFrame = false) = 0;

	virtual int robotKinematicInstanceCount() const = 0;
	virtual int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const = 0;
	virtual RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) = 0;
	virtual RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) = 0;

	/// 设置机器人基座放置位姿 P（工程恢复时从 project.json 还原；列主序 Mat4）
	virtual void setRobotBasePlacementWorldForInstance(int instanceIndex,
													   const cloudsim::core::Mat4& placementWorld) = 0;

	/// 导入后按场景与关节角重算 per-link M0，使后续拖动 FK 与绑定一致
	virtual void urdfImportReconcilePerLinkBind(int instanceIndex, const QVector<double>& localJointRad)
	{
		(void)instanceIndex;
		(void)localJointRad;
	}
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IROBOTURDFIMPORTCONTEXT_H
