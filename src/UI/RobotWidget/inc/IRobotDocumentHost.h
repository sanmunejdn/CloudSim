#pragma once

#include "IRobotSimulationDocument.h"
#include "RobotCoordinateFrames.h"
#include "RobotProgramStore.h"
#include "BackendFollowMath.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <osg/Matrixd>

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

	/// 应用关节角到指定机器人实例，更新聚合向量
	virtual bool applyJointAnglesRad(int instanceIndex, const QVector<double>& jointAnglesRad,
		QVector<double>& aggregatedJointAnglesRad, QString* outError = nullptr) = 0;

	/// 从 TCP 位姿捕获工具坐标系（计算 T_flange_tool 并更新活动工具帧）
	virtual bool captureToolFrameFromTcp(int instanceIndex, const BackendMat4& T_base_tcp,
		const QVector<double>& jointAnglesRad, const QString& flangeLinkName,
		RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError = nullptr) = 0;

	/// 从 TCP 位姿捕获用户坐标系（更新活动用户帧的 T_base_user）
	virtual bool captureUserFrameFromTcp(int instanceIndex, double posXmm, double posYmm, double posZmm,
		double eulerXdeg, double eulerYdeg, double eulerZdeg,
		RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError = nullptr) = 0;

	/// 重置活动工具帧为法兰原点
	virtual void resetToolFrame(int instanceIndex, RobotCoordinate::RobotCoordinateFrameSet& frames) = 0;

	/// TCP 拖拽示教 IK 求解（核心 IK 逻辑，不含 UI 钳位/节流）
	struct TcpDragIkResult
	{
		bool ok = false;
		QVector<double> jointRad;
		QString error;
	};
	virtual TcpDragIkResult solveTcpDragTeachIk(int instanceIndex,
		double pxMm, double pyMm, double pzMm,
		double exDeg, double eyDeg, double ezDeg,
		const QVector<double>& seedJointRad,
		const QString& ikLinkName) = 0;

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
		const QVector<double>& seedJointRad,
		const QString& urdfPath,
		const QString& tcpLinkName,
		std::vector<ExportPlanResult>& outPlans,
		int& outFailedCount,
		QString* outError = nullptr) = 0;
	virtual void setSuppressRobotFollowDirtyNotify(bool suppress) = 0;
	virtual void clearFollowDirtyBackendIds() = 0;
	virtual QString meshBackendStepSourcePath(const QString& backendId) const { return QString(); }
};
