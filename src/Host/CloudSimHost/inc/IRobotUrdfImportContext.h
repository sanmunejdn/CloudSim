#pragma once

#include "cloudsim_host_global.h"

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>

class BackendDataManager;
class IRobotBackendPoseSink;
class IRobotSimulationDocument;
class MeshBackendData;

namespace osg {
class MatrixTransform;
class Matrixd;
}

namespace RobotCoordinate {
struct RobotCoordinateFrameSet;
}

namespace cloudsim::host {

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

	virtual void appendHierarchicalRobotSimulationContext(const QString& urdfAbsolutePath,
		const QStringList& revoluteJointNamesUnprefixed, const QVector<double>& jointLowerRad,
		const QVector<double>& jointUpperRad, const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys,
		const QString& robotSceneBackendId, const QString& jointPrefixRootOverride = QString()) = 0;

	virtual void setRobotPerLinkKinematicsBinding(const QString& importKey, const QHash<QString, QString>& linkNameToBackendId,
		const QHash<QString, osg::Matrixd>& fkMeshWorldT0, const QHash<QString, osg::Matrixd>& outerWorldAtBindByBackendId,
		bool meshVerticesInLinkFrame = false) = 0;

	virtual int robotKinematicInstanceCount() const = 0;
	virtual int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const = 0;
	virtual RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) = 0;

	/// 设置机器人基座放置位姿 P（工程恢复时从 project.json 还原）
	virtual void setRobotBasePlacementWorldForInstance(int instanceIndex, const osg::Matrixd& placementWorld) = 0;
};

} // namespace cloudsim::host
