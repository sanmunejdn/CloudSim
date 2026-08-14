#ifndef ROBOTWIDGET_IROBOTDOCUMENTHOST_H
#define ROBOTWIDGET_IROBOTDOCUMENTHOST_H

/// @file IRobotDocumentHost.h
/// @brief 文档级机器人状态与变更（DocumentPage 实现）

#include "robotwidget_global.h"

#include "BackendFollowMath.h"
#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotCollisionSettings.h"
#include "RobotProgramStore.h"

#include "CoreTypes.h"
#include "IDataService.h"

#include <memory>
#include <string>
#include <vector>

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <vector>

class BackendDataManager;
class BackendDataBase;
class IRobotBackendPoseSink;

/// 文档级机器人状态与变更（DocumentPage 实现）
class ROBOTWIDGET_EXPORT IRobotDocumentHost : public IRobotSimulationDocument
{
public:
	~IRobotDocumentHost() override = default;

	virtual RobotProgramStore& robotProgramStore() = 0;
	virtual const RobotProgramStore& robotProgramStore() const = 0;

	virtual IRobotBackendPoseSink* poseSink() = 0;
	virtual BackendDataManager& backend() = 0;
	/// 契约数据面（新代码优先；减少 backend()）
	virtual cloudsim::core::IDataService& documentData() = 0;
	virtual const cloudsim::core::IDataService& documentData() const = 0;
	/// 对象查询（替代 backend().getData / listData；运动学等仍可走 backend()）
	virtual std::shared_ptr<BackendDataBase> findObject(const std::string& id) const = 0;
	virtual std::vector<std::shared_ptr<BackendDataBase>> listObjects() const = 0;

	virtual QString robotSceneBackendIdForInstance(int instanceIndex) const = 0;
	virtual QString robotFrameWorldReferenceBackendId(int instanceIndex) const = 0;
	virtual QString robotDisplayLabelForInstance(int instanceIndex) const = 0;
	virtual QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const = 0;
	virtual void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad,
											 QVector<double>& upperRad) const = 0;
	virtual int robotJointOffsetInAggregatedVector(int instanceIndex) const = 0;
	virtual int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const = 0;
	virtual int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot = nullptr) const = 0;

	virtual RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) = 0;
	virtual const RobotCoordinate::RobotCoordinateFrameSet&
	robotCoordinateFramesForInstance(int instanceIndex) const = 0;
	virtual RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) = 0;
	virtual const RobotExternal::RobotExternalAxisConfigSet&
	robotExternalAxesForInstance(int instanceIndex) const = 0;
	virtual RobotCollision::Settings& robotCollisionSettings() = 0;
	virtual const RobotCollision::Settings& robotCollisionSettings() const = 0;
	virtual const RobotCoordinate::RobotUserFrame* robotActiveUserFrameForInstance(int instanceIndex) const = 0;

	virtual void setRobotBasePlacementWorldForInstance(int instanceIndex,
													   const cloudsim::core::Mat4& placementWorld) = 0;
	/// 存盘基座 P0（不含外轴）
	virtual cloudsim::core::Mat4 robotBasePlacementWorldForInstance(int instanceIndex) const = 0;
	/// 运行时外轴 q（首个 RobotBase 轴兼容）；P0 仍由 basePlacement 保存
	virtual void setRobotExternalAxisQMm(int instanceIndex, double qMm) = 0;
	virtual double robotExternalAxisQMm(int instanceIndex) const = 0;
	/// 与 config.axes 下标对齐的外轴向量
	virtual void setRobotExternalAxisQ(int instanceIndex, const std::vector<double>& qValues) = 0;
	virtual std::vector<double> robotExternalAxisQ(int instanceIndex) const = 0;
	virtual cloudsim::core::Mat4 workpieceExternalBasePlacement(int instanceIndex, const QString& backendId) const = 0;
	virtual void setWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
												   const cloudsim::core::Mat4& w0) = 0;
	virtual void ensureWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
													  const cloudsim::core::Mat4& currentWorld) = 0;
	virtual cloudsim::core::Mat4 workpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId) const = 0;
	virtual void ensureWorkpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId,
												   const QString& workingFrameId,
												   const cloudsim::core::Mat4& workingWorld) = 0;
	virtual void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId,
												   const cloudsim::core::Mat4& world) = 0;
	virtual void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) = 0;
	virtual void notifyRobotKinematicsAppliedToScene() = 0;
	virtual void requestFollowSolveForced() = 0;

	/// 应用关节角到指定机器人实例，更新聚合向量
	virtual bool applyJointAnglesRad(int instanceIndex, const QVector<double>& jointAnglesRad,
									 QVector<double>& aggregatedJointAnglesRad, QString* outError = nullptr) = 0;

	/// 从 TCP 位姿捕获工具坐标系（计算 T_flange_tool 并更新活动工具帧）
	virtual bool captureToolFrameFromTcp(int instanceIndex, const BackendMat4& T_base_tcp,
										 const QVector<double>& jointAnglesRad, const QString& flangeLinkName,
										 RobotCoordinate::RobotCoordinateFrameSet& frames,
										 QString* outError = nullptr) = 0;

	/// 从 TCP 位姿捕获用户坐标系（更新活动用户帧的 T_base_user）
	virtual bool captureUserFrameFromTcp(int instanceIndex, double posXmm, double posYmm, double posZmm,
										 double eulerXdeg, double eulerYdeg, double eulerZdeg,
										 RobotCoordinate::RobotCoordinateFrameSet& frames,
										 QString* outError = nullptr) = 0;

	/// 重置活动工具帧为法兰原点
	virtual void resetToolFrame(int instanceIndex, RobotCoordinate::RobotCoordinateFrameSet& frames) = 0;

	/// TCP 拖拽示教 IK 求解（核心 IK 逻辑，不含 UI 钳位/节流）
	struct TcpDragIkResult
	{
		bool ok = false;
		QVector<double> jointRad;
		bool hasExternalAxisQ = false;
		double externalAxisQ = 0.0;
		std::vector<double> externalAxisQs;
		QString error;
	};
	virtual TcpDragIkResult solveTcpDragTeachIk(int instanceIndex, double pxMm, double pyMm, double pzMm, double exDeg,
												double eyDeg, double ezDeg, const QVector<double>& seedJointRad,
												const QString& ikLinkName,
												const std::vector<double>& externalAxisQSeed = {},
												bool hasExternalAxisQSeed = false) = 0;

	/// 导出程序规划结果（不含 UI 文件对话框）
	struct ExportPlanResult
	{
		bool ok = false;
		std::vector<double> jointTargetsRad;
		std::string summary;
		std::string plannerName;
	};
	virtual bool planForExport(int instanceIndex,
							   const std::vector<std::shared_ptr<RobotInstruction::Base>>& instructions,
							   const QVector<double>& seedJointRad, const QString& urdfPath, const QString& tcpLinkName,
							   std::vector<ExportPlanResult>& outPlans, int& outFailedCount,
							   QString* outError = nullptr) = 0;
	virtual void setSuppressRobotFollowDirtyNotify(bool suppress) = 0;
	virtual void clearFollowDirtyBackendIds() = 0;
	virtual QString meshBackendStepSourcePath(const QString& backendId) const { return QString(); }
};

#endif // ROBOTWIDGET_IROBOTDOCUMENTHOST_H
