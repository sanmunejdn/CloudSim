#pragma once

#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotProgramStore.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <osg/Matrixd>

class BackendDataManager;
class BackendDataBase;
class IRobotBackendPoseSink;

/// Document-level robot state and mutation (implemented by Widget \ref DocumentPage wrapper).
class ROBOTWIDGET_EXPORT IRobotDocumentHost : public IRobotSimulationDocument
{
public:
	~IRobotDocumentHost() override = default;

	virtual RobotProgramStore& robotProgramStore() = 0;
	virtual const RobotProgramStore& robotProgramStore() const = 0;

	virtual IRobotBackendPoseSink* poseSink() = 0;
	virtual BackendDataManager& backend() = 0;

	virtual QString robotSceneBackendIdForInstance(int instanceIndex) const = 0;
	virtual QString robotFrameWorldReferenceBackendId(int instanceIndex) const = 0;
	virtual QString robotDisplayLabelForInstance(int instanceIndex) const = 0;
	virtual QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const = 0;
	virtual void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad, QVector<double>& upperRad) const = 0;
	virtual int robotJointOffsetInAggregatedVector(int instanceIndex) const = 0;
	virtual int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const = 0;
	virtual int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot = nullptr) const = 0;

	virtual RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) = 0;
	virtual const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const = 0;
	virtual const RobotCoordinate::RobotUserFrame* robotActiveUserFrameForInstance(int instanceIndex) const = 0;

	virtual void setRobotBasePlacementWorldForInstance(int instanceIndex, const osg::Matrixd& placementWorld) = 0;
	virtual void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId, const osg::Matrixd& world) = 0;
	virtual void notifyRobotKinematicsAppliedToScene() = 0;
	virtual void requestFollowSolveForced() = 0;
};
