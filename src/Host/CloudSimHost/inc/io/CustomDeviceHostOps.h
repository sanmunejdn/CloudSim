#ifndef CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H
#define CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H

/// @file CustomDeviceHostOps.h
/// @brief Web/Headless：自定义设备列表、姿态绑定、applyQ、组装提交

#include "cloudsim_host_global.h"

#include <QJsonObject>
#include <QString>

namespace cloudsim::host
{
class DocumentHost;
class IoSignalNetwork;

CLOUDSIM_HOST_EXPORT QJsonObject listCustomDevicesJson(DocumentHost& host);
CLOUDSIM_HOST_EXPORT QJsonObject customDeviceDetailJson(DocumentHost& host, const QString& deviceId);
CLOUDSIM_HOST_EXPORT bool putCustomDeviceRuntimeFields(DocumentHost& host, const QString& deviceId,
													   const QJsonObject& body, QString* err);
CLOUDSIM_HOST_EXPORT bool applyCustomDeviceQ(DocumentHost& host, const QString& deviceId, const QJsonObject& body,
											 QString* err);
CLOUDSIM_HOST_EXPORT bool gotoCustomDevicePose(DocumentHost& host, const QString& deviceId, const QJsonObject& body,
											   QString* err);
/// DI 上升沿 → named pose（durationSec 走 Host 插值播放）
CLOUDSIM_HOST_EXPORT void processCustomDevicePoseRisingEdges(DocumentHost& host, IoSignalNetwork& network,
															 const QString& deviceOwnerId);
/// 按当前 DI 写入边沿基线（不触发姿态）；避免「首次变化已是高」被吞掉
CLOUDSIM_HOST_EXPORT void primeCustomDevicePoseEdgeMemory(IoSignalNetwork& network);
/// 清边沿记忆；若给 host 则同时停止姿态插值（对齐桌面 resetEdgeState）
CLOUDSIM_HOST_EXPORT void clearCustomDevicePoseEdgeMemory(DocumentHost* host = nullptr);
CLOUDSIM_HOST_EXPORT bool commitCustomDeviceAssembly(DocumentHost& host, const QJsonObject& body, QString* err,
													 QString* outDeviceId);
/// 确保工程内有自定义设备（可空 id 新建）
CLOUDSIM_HOST_EXPORT bool ensureCustomDevice(DocumentHost& host, const QJsonObject& body, QString* err,
											 QString* outDeviceId);
/// 将场景几何挂到设备下（与桌面 attachChildToCustomDevice 同语义）
CLOUDSIM_HOST_EXPORT bool attachCustomDeviceChildren(DocumentHost& host, const QString& deviceId,
													 const QJsonArray& childIds, QString* err);
/// 可组装几何候选：Mesh / Brep
CLOUDSIM_HOST_EXPORT QJsonObject listAssemblyGeometryCandidatesJson(DocumentHost& host);
CLOUDSIM_HOST_EXPORT bool exportCustomDeviceUrdfZip(DocumentHost& host, const QString& deviceId,
													const QString& packageParentDir, QString* err,
													QString* outPackageDir);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H
