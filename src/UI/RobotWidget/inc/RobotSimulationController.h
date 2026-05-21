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
namespace RobotInstruction { class Base; struct FeasibleMotionAxisConfigurationOptions; }

/// Robot simulation / teach / instruction orchestration (migrated from MainWindow).
class ROBOTWIDGET_EXPORT RobotSimulationController : public QObject
{
	Q_OBJECT

public:
	explicit RobotSimulationController(QObject* parent = nullptr);

	void setHost(IRobotMainWindowHost* host);
	IRobotMainWindowHost* host() const { return m_host; }

	RobotSimulationDockWidget* simulationDock() const { return m_simulationDock; }
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
	void refreshSimulationJointListFromCurrentDoc();
	void syncRobotFrameSettingsFromDocument(int instanceIndex);
	void refreshRobotCoordinateFrameOverlays(
		const std::shared_ptr<RobotInstruction::Base>& highlightInstruction = nullptr,
		const QVector<double>* jointAnglesRadLocal = nullptr);
	void refreshRobotCoordinateFrameOverlaysForPlayback();
	void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void refreshInstructionPoseAxes();

private:
	void captureMotionPreviewProgramStartJoints();
	QVector<double> motionPreviewProgramStartJointsLocal(int nj, int jointOffset) const;
	QVector<double> localJointAnglesForInstance(int instIdx) const;
	void logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad);
	QHash<QString, bool> computeMotionReachabilityForCurrentProgram();
	bool applyTcpDragTeachIkFromPose(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);

	IRobotMainWindowHost* m_host = nullptr;
	RobotSimulationDockWidget* m_simulationDock = nullptr;
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
	engine::RigidTransform m_lastTcpDragTargetInBase;
	bool m_lastTcpDragTargetValid = false;
	bool m_skipInstructionPreviewOnce = false;
	RobotInstruction::FeasibleMotionAxisConfigurationOptions m_cachedFeasibleAxisOptions;
	QString m_cachedFeasibleAxisInstructionId;
	QString m_cachedFeasibleAxisFingerprint;
	QVector<double> m_cachedFeasibleAxisSeedJointRad;
};
