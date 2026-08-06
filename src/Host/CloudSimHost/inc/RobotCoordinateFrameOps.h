#ifndef CLOUDSIMHOST_ROBOTCOORDINATEFRAMEOPS_H
#define CLOUDSIMHOST_ROBOTCOORDINATEFRAMEOPS_H

/// @file RobotCoordinateFrameOps.h
/// @brief Headless/Web：工具/用户系捕获、重置、叠加与程序工具上下文同步

#include "cloudsim_host_global.h"

#include "RobotCoordinateFrames.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <string>

class BackendDataManager;
class RobotProgramStore;

namespace cloudsim::host
{
class HeadlessRobotContext;
class DocumentHost;

struct FrameOverlayEntry
{
	QString id;
	QString name;
	bool active = false;
	double positionMm[3]{0.0, 0.0, 0.0};
	double eulerDeg[3]{0.0, 0.0, 0.0};
	/// 与 GET /api/objects 相同的 BackendMat4 布局，供 Three.js worldMatrix 转换
	BackendMat4 worldMat = BackendMat4::identity();
};

struct FrameOverlaySnapshot
{
	QVector<FrameOverlayEntry> tools;
	QVector<FrameOverlayEntry> users;
};

/// T_flange_tool = inv(T_base_flange) * T_base_tcp
CLOUDSIM_HOST_EXPORT bool captureToolFrameFromTcpPose(const QString& urdfPath, const QVector<double>& jointAnglesRad,
													  const QString& flangeLinkName, const BackendMat4& T_base_tcp,
													  RobotCoordinate::RobotCoordinateFrameSet& frames,
													  QString* outError = nullptr);

CLOUDSIM_HOST_EXPORT bool captureUserFrameFromTcpPose(double posXmm, double posYmm, double posZmm, double eulerXdeg,
													  double eulerYdeg, double eulerZdeg,
													  RobotCoordinate::RobotCoordinateFrameSet& frames,
													  QString* outError = nullptr);

CLOUDSIM_HOST_EXPORT void resetActiveToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames);

/// 可见工具/用户系世界位姿（sceneRoot world * base 系）
CLOUDSIM_HOST_EXPORT bool buildFrameOverlaySnapshot(HeadlessRobotContext& hrc, BackendDataManager& backend,
													const QString& sceneRootBackendId, FrameOverlaySnapshot& out,
													QString* outError = nullptr);

/// Active/几何变更时同步路点 tool context；DisplayOnly 跳过
CLOUDSIM_HOST_EXPORT void syncProgramToolContextAfterFrameChange(
	RobotProgramStore& store, const QString& sceneRootBackendId,
	const RobotCoordinate::RobotCoordinateFrameSet& oldFrames,
	const RobotCoordinate::RobotCoordinateFrameSet& newFrames);

/// 与桌面 robotKinematicsInstances 字段兼容
CLOUDSIM_HOST_EXPORT void mergeRobotKinematicsIntoProjectRoot(DocumentHost& host, QJsonObject& root);

CLOUDSIM_HOST_EXPORT QJsonObject coordinateFrameSetToQJson(const RobotCoordinate::RobotCoordinateFrameSet& frames);
CLOUDSIM_HOST_EXPORT bool coordinateFrameSetFromQJson(const QJsonObject& obj,
													  RobotCoordinate::RobotCoordinateFrameSet& out);

CLOUDSIM_HOST_EXPORT bool coordinateFrameSetPlanningEquals(const RobotCoordinate::RobotCoordinateFrameSet& a,
														   const RobotCoordinate::RobotCoordinateFrameSet& b);

/// 对齐桌面 onAddToolFrame / onAddUserFrame；返回新帧 id
CLOUDSIM_HOST_EXPORT std::string addToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames);
CLOUDSIM_HOST_EXPORT std::string addUserFrame(RobotCoordinate::RobotCoordinateFrameSet& frames);
CLOUDSIM_HOST_EXPORT std::string duplicateToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames,
													const std::string& sourceId);
CLOUDSIM_HOST_EXPORT std::string duplicateUserFrame(RobotCoordinate::RobotCoordinateFrameSet& frames,
													const std::string& sourceId);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTCOORDINATEFRAMEOPS_H
