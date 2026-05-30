#pragma once

#include "IRobotMainWindowHost.h"
#include "RobotInstructionController.h"
#include "RobotProgramExecutor.h"
#include "SimulationLogIoSink.h"
#include "robotwidget_global.h"

#include <QElapsedTimer>
#include <QHash>
#include <QObject>
#include <QTimer>
#include <QVector>

#include <memory>

#include <RigidTransform.h>
#include <osg/Matrixd>

class BackendDataBase;

class RobotSimulationDockWidget;
class QtProperty;
class ProgramEditService;
class TrajectoryEditSession;
namespace RobotInstruction { class Base; struct FeasibleMotionAxisConfigurationOptions; }

/// 机器人仿真/示教/指令编排（自 MainWindow 迁出）
class ROBOTWIDGET_EXPORT RobotSimulationController : public QObject
{
	Q_OBJECT

public:
	explicit RobotSimulationController(QObject* parent = nullptr);

	void setHost(IRobotMainWindowHost* host);
	IRobotMainWindowHost* host() const { return m_host; }

	RobotSimulationDockWidget* simulationDock() const { return m_simulationDock; }
	ProgramEditService* programEditService() { return m_programEditService; }
	TrajectoryEditSession* trajectoryEditSession() { return m_trajectoryEditSession; }
	void createSimulationDock(QWidget* parentForTabs);

	void initializePlanners();
	void wireSimulationSignals();
	void attachPlaybackTimer(QTimer* externalTimer);

	RobotInstruction::Controller& instructionController() { return m_instructionController; }
	RobotProgramExecutor& programExecutor() { return m_programExecutor; }
	SimulationLogIoSink& simulationIoSink() { return m_simulationIoSink; }

	RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleMotionAxisConfigurationOptionsForInstruction(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		QVector<double>* outSeedJointRad = nullptr);
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& cachedFeasibleAxisConfigurationOptions() const
	{
		return m_cachedFeasibleAxisOptions;
	}
	void invalidateFeasibleAxisConfigurationCache();

	bool tryCaptureCurrentRobotTcpPose(
		RobotInstruction::Vec3& outPoseMm,
		RobotInstruction::Vec3& outEulerDeg,
		osg::Matrixd* outTcpLocalMat,
		osg::Matrixd* outTcpRenderWorldMat,
		QString* outTcpLinkName,
		QString* errMsg) const;

	void syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data);

public slots:
	void onUrdfImportRequested(const QString& urdfPath);
	void onSimulationStartTriggered();
	void onSimulationRunRequested();
	void onSimulationStopRequested();
	void onSimulationAddInstructionRequested(RobotInstruction::Type type);
	void onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void onRobotSimulationTick();
	void onSimulationExportRequested();
	void onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId);
	void onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad);
	void onSimulationTcpDragTeachModeChanged(bool enabled);
	void onTcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void onTcpDragTeachEnded();
	void onRobotCoordinateFramesChanged();
	void onCaptureToolFrameFromTcp();
	void onCaptureUserFrameFromTcp();
	void onResetToolFrame();

	void stopRobotSimulation();
	QVector<double> aggregatedJointAnglesRad() const { return m_aggregatedJointAnglesRad; }
	void restoreAggregatedJointStateAfterProjectLoad(const QVector<double>& allJointAnglesRad);
	void applyProgramStartPoseAfterProjectLoad();
	void refreshInstructionPoseAxes(bool computeReachability = true);
	void setRawTrajectoryPreviewActive(bool active);
	bool rawTrajectoryPreviewActive() const { return m_rawTrajectoryPreviewActive; }
	void refreshSimulationJointListFromCurrentDoc();
	void syncRobotFrameSettingsFromDocument(int instanceIndex);
	void refreshRobotCoordinateFrameOverlays(
		const std::shared_ptr<RobotInstruction::Base>& highlightInstruction = nullptr,
		const QVector<double>* jointAnglesRadLocal = nullptr);
	void refreshRobotCoordinateFrameOverlaysForPlayback();
	void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction);

private:
	void applyProgramStartPoseAfterProjectLoadImpl();
	void finishProgramStartPoseAfterProjectLoad(int instIdx, QVector<double> startJointQ);
	void captureMotionPreviewProgramStartJoints();
	QVector<double> motionPreviewProgramStartJointsLocal(int nj, int jointOffset) const;
	QVector<double> localJointAnglesForInstance(int instIdx) const;
	void logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad);
	QHash<QString, bool> computeMotionReachabilityForCurrentProgram();
	bool applyTcpDragTeachIkFromPose(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void syncTcpDragTeachAnchorFromCurrentJoints();
	void syncTcpDragExitJointState();

	bool planMotionOnHost(
		RobotInstruction::Base& instruction,
		const QVector<double>& seedJointRad,
		int instanceIndex,
		const QString& urdfPath,
		const QString& defaultTcpLinkName,
		const QString& sceneRootBackendId,
		RobotInstruction::PlanResult& plan,
		std::string* planErr) const;

	IRobotMainWindowHost* m_host = nullptr;
	RobotSimulationDockWidget* m_simulationDock = nullptr;
	ProgramEditService* m_programEditService = nullptr;
	TrajectoryEditSession* m_trajectoryEditSession = nullptr;
	QTimer* m_playbackTimer = nullptr;
	bool m_ownsPlaybackTimer = false;

	RobotInstruction::Controller m_instructionController;
	RobotProgramExecutor m_programExecutor;
	SimulationLogIoSink m_simulationIoSink;
	QVector<double> m_aggregatedJointAnglesRad;
	QVector<double> m_motionPreviewProgramStartJointRad;
	bool m_suppressMotionPreviewStartCapture = false;
	QString m_tcpDragTeachFlangeLink;
	QElapsedTimer m_tcpDragTeachIkTimer;
	bool m_tcpDragApplyingIk = false;
	QVector<double> m_tcpDragLastAppliedJointRad;
	engine::RigidTransform m_lastTcpDragTargetInBase;
	bool m_lastTcpDragTargetValid = false;
	bool m_skipInstructionPreviewOnce = false;
	bool m_rawTrajectoryPreviewActive = false;
	RobotInstruction::FeasibleMotionAxisConfigurationOptions m_cachedFeasibleAxisOptions;
	QString m_cachedFeasibleAxisInstructionId;
	QString m_cachedFeasibleAxisFingerprint;
	QVector<double> m_cachedFeasibleAxisSeedJointRad;
};
