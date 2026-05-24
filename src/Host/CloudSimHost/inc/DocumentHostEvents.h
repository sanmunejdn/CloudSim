#pragma once

#include "CoreEvents.h"
#include "cloudsim_host_global.h"

class BackendDataBase;

namespace cloudsim::host {

class DocumentHost;

/// DocumentHost → EventHub 的薄封装，避免 UI 直接拼装 core 事件字段

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
/// gizmo 松手等位姿落盘场景，从 Backend 标量组装 PoseDto
CLOUDSIM_HOST_EXPORT void publishPoseCommittedFromBackend(DocumentHost& host, const BackendDataBase& data);

} // namespace cloudsim::host
