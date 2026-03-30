#pragma once

#include "robot_scene_global.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <osg/Matrixd>

/// Read-only robot simulation state exposed by a document (implemented by \ref DocumentPage in Widget).
class ROBOT_SCENE_API IRobotSimulationDocument
{
public:
	virtual ~IRobotSimulationDocument() = default;

	virtual bool hasRobotSimulationContext() const = 0;
	virtual bool hasRobotKinematicsBind() const = 0;
	virtual const QString& robotUrdfAbsolutePath() const = 0;
	virtual const QStringList& robotRevoluteJointNames() const = 0;
	virtual const QHash<QString, QString>& robotLinkNameToBackendId() const = 0;
	virtual const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const = 0;
	virtual const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const = 0;
};
