/// @file DocumentHostEvents.cpp
/// @brief 文档宿主事件协作

#include "DocumentHostEvents.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "CoreEvents.h"
#include "CoreTypes.h"
#include "DocumentHost.h"
#include "EventHub.h"

namespace cloudsim::host
{
using ::BackendDataBase;

void publishBackendObjectRegistered(DocumentHost& host, const QString& objectId, const QString& className)
{
	if (objectId.isEmpty())
	{
		return;
	}
	cloudsim::core::BackendObjectRegisteredEvent ev;
	ev.documentId = host.documentId();
	ev.objectId = objectId;
	ev.className = className;
	host.events().publish(ev);
}

void publishBackendObjectRemoved(DocumentHost& host, const QString& objectId)
{
	if (objectId.isEmpty())
	{
		return;
	}
	cloudsim::core::BackendObjectRemovedEvent ev;
	ev.documentId = host.documentId();
	ev.objectId = objectId;
	host.events().publish(ev);
}

void publishRobotKinematicsApplied(DocumentHost& host, const QString& sceneRootBackendId,
								   const QVector<double>& jointAnglesRad)
{
	host.noteRobotLocalJointAnglesForSceneRoot(sceneRootBackendId, jointAnglesRad);
	cloudsim::core::RobotKinematicsAppliedEvent ev;
	ev.documentId = host.documentId();
	ev.sceneRootBackendId = sceneRootBackendId;
	ev.jointAnglesRad = jointAnglesRad;
	host.events().publish(ev);
}

void publishProjectLoaded(DocumentHost& host, const QString& projectPath)
{
	cloudsim::core::ProjectLoadedEvent ev;
	ev.documentId = host.documentId();
	ev.projectPath = projectPath;
	host.events().publish(ev);
}

void publishSelectionChanged(DocumentHost& host, const QString& objectId, const cloudsim::core::SelectionSource source)
{
	cloudsim::core::SelectionChangedEvent ev;
	ev.documentId = host.documentId();
	ev.primaryId = objectId;
	ev.source = source;
	host.events().publish(ev);
}

void publishPoseCommitted(DocumentHost& host, const QString& objectId, const cloudsim::core::PoseDto& pose)
{
	if (objectId.isEmpty())
	{
		return;
	}
	cloudsim::core::PoseCommittedEvent ev;
	ev.documentId = host.documentId();
	ev.objectId = objectId;
	ev.pose = pose;
	host.events().publish(ev);
}

void publishPoseCommittedFromBackend(DocumentHost& host, const BackendDataBase& data)
{
	const BackendVec3 pos = data.pose();
	const BackendVec3 rot = data.rotation();
	cloudsim::core::PoseDto dto;
	dto.positionMm.x = pos.x;
	dto.positionMm.y = pos.y;
	dto.positionMm.z = pos.z;
	dto.eulerDeg.x = rot.x;
	dto.eulerDeg.y = rot.y;
	dto.eulerDeg.z = rot.z;
	publishPoseCommitted(host, QString::fromStdString(data.id()), dto);
}

void publishPoseCommittedFromBackendId(DocumentHost& host, const QString& objectId)
{
	const auto obj = host.backend().getData(objectId.toStdString());
	if (!obj)
	{
		return;
	}
	publishPoseCommittedFromBackend(host, *obj);
}

} // namespace cloudsim::host
