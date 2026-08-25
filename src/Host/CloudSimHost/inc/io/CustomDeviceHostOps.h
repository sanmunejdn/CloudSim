#ifndef CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H
#define CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H

/// @file CustomDeviceHostOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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

CLOUDSIM_HOST_EXPORT QJsonObject listRobotsForMountJson(DocumentHost& host);
CLOUDSIM_HOST_EXPORT bool mountCustomDeviceToRobotFlange(DocumentHost& host, const QString& deviceId,
														 const QJsonObject& body, QString* err);
CLOUDSIM_HOST_EXPORT bool unmountCustomDeviceFromRobotFlange(DocumentHost& host, const QString& deviceId, QString* err);

/// 设备根位姿变更后：applyQ 传播连杆几何，并同步子树 OSG
CLOUDSIM_HOST_EXPORT void syncCustomDeviceKinematicsAfterRootPoseChange(DocumentHost& host,
																		const std::string& deviceBackendId);

/// Link/Joint 提交后：登记连杆几何为 FK 独占并卸层级 Follow（否则轴控 applyQ 会被 Follow 拉回）
CLOUDSIM_HOST_EXPORT void finalizeCustomDeviceLinkJointGraph(DocumentHost& host, const std::string& deviceBackendId);

/// applyQ 后：把连杆几何 worldMatrix 刷到 OSG（poseSink 只标脏，须显式 flush）
CLOUDSIM_HOST_EXPORT void flushCustomDeviceLinkGeometryVisual(DocumentHost& host, const std::string& deviceBackendId);

/// applyQ 后：旋转中心坐标系刷到 OSG
CLOUDSIM_HOST_EXPORT void flushCustomDeviceMotionCenterFrameVisual(DocumentHost& host,
																   const std::string& deviceBackendId);

/// 工程加载：为已存图的自定义设备补登记连杆 FK 独占
CLOUDSIM_HOST_EXPORT void registerAllCustomDeviceLinkGeometryOwnership(DocumentHost& host);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_CUSTOMDEVICEHOSTOPS_H
