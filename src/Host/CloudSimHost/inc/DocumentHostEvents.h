#ifndef CLOUDSIMHOST_DOCUMENTHOSTEVENTS_H
#define CLOUDSIMHOST_DOCUMENTHOSTEVENTS_H

/// @file DocumentHostEvents.h
/// @brief Host 事件发布封装

#include "cloudsim_host_global.h"

#include "CoreEvents.h"

class BackendDataBase;

namespace cloudsim::host
{
class DocumentHost;

/// Host 事件发布封装
CLOUDSIM_HOST_EXPORT void publishBackendObjectRegistered(DocumentHost& host, const QString& objectId,
														 const QString& className);
CLOUDSIM_HOST_EXPORT void publishBackendObjectRemoved(DocumentHost& host, const QString& objectId);
CLOUDSIM_HOST_EXPORT void publishRobotKinematicsApplied(DocumentHost& host, const QString& sceneRootBackendId,
														const QVector<double>& jointAnglesRad);
CLOUDSIM_HOST_EXPORT void publishProjectLoaded(DocumentHost& host, const QString& projectPath);
CLOUDSIM_HOST_EXPORT void publishSelectionChanged(DocumentHost& host, const QString& objectId,
												  cloudsim::core::SelectionSource source);
CLOUDSIM_HOST_EXPORT void publishPoseCommitted(DocumentHost& host, const QString& objectId,
											   const cloudsim::core::PoseDto& pose);
/// 从 Backend 组装 PoseDto
CLOUDSIM_HOST_EXPORT void publishPoseCommittedFromBackend(DocumentHost& host, const BackendDataBase& data);
CLOUDSIM_HOST_EXPORT void publishPoseCommittedFromBackendId(DocumentHost& host, const QString& objectId);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_DOCUMENTHOSTEVENTS_H
