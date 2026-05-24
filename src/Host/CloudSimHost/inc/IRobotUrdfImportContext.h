#pragma once

#include "cloudsim_host_global.h"

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>

class BackendDataManager;
class IRobotBackendPoseSink;
class IRobotSimulationDocument;
class OsgWidget;

namespace osg {
class MatrixTransform;
class Matrixd;
}

namespace RobotCoordinate {
struct RobotCoordinateFrameSet;
}

namespace cloudsim::host {

/// DocumentPage 实现：URDF/工程恢复写后端与仿真元数据（与 DocumentHost 类名区分，避免遮蔽）
class CLOUDSIM_HOST_EXPORT IRobotUrdfImportContext
{
public:
	virtual ~IRobotUrdfImportContext() = default;

	virtual BackendDataManager& urdfImportBackend() = 0;
	virtual OsgWidget* urdfImportOsgWidget() = 0;
	virtual IRobotSimulationDocument* urdfImportRobotSimulationDocument() = 0;
	virtual IRobotBackendPoseSink* urdfImportScenePoseSink() = 0;

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
};

} // namespace cloudsim::host
