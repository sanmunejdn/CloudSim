#pragma once

#include "CoreTypes.h"

namespace cloudsim::core {

enum class SelectionSource
{
	Tree,
	OsgPick,
	Programmatic
};

struct SelectionChangedEvent
{
	QString documentId;
	ObjectId primaryId;
	SelectionSource source = SelectionSource::Programmatic;
};

struct PoseCommittedEvent
{
	QString documentId;
	ObjectId objectId;
	PoseDto pose;
};

struct BackendObjectRegisteredEvent
{
	QString documentId;
	ObjectId objectId;
	QString className;
};

struct BackendObjectRemovedEvent
{
	QString documentId;
	ObjectId objectId;
};

struct ProjectLoadedEvent
{
	QString documentId;
	QString projectPath;
};

struct RobotKinematicsAppliedEvent
{
	QString documentId;
	ObjectId sceneRootBackendId;
	QVector<double> jointAnglesRad;
};

} // namespace cloudsim::core
