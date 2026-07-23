#ifndef ROBOTWIDGET_ROBOTSIMULATIONCONTROLLER_H
#define ROBOTWIDGET_ROBOTSIMULATIONCONTROLLER_H

/// @file RobotSimulationController.h
/// @brief 指令选中时链式种子，供 feasible 与 preview 共用，避免重复 IK

#include "robotwidget_global.h"

#include "IRobotMainWindowHost.h"
#include "PlanResultCache.h"
#include "RobotInstructionController.h"
#include "RobotProgramExecutor.h"
#include "SimulationLogIoSink.h"

#include <QElapsedTimer>
#include <QHash>
#include <QObject>
#include <QSet>
#include <QTimer>
#include <QVector>
#include <memory>
#include <string>
#include <vector>

#include <RigidTransform.h>
#include <osg/Matrixd>

class BackendDataBase;

class RobotSimulationDockWidget;
class QtProperty;
class ProgramEditService;
class TrajectoryEditSession;
namespace RobotInstruction
{
class Base;
struct FeasibleMotionAxisConfigurationOptions;
struct RawTrajectory;
} // namespace RobotInstruction

/// 指令选中时链式种子，供 feasible 与 preview 共用，避免重复 IK
struct ROBOTWIDGET_EXPORT PrecomputedChainSeed
{
	QVector<double> jointRad;
	bool reliable = true;
	int motionIndex = -1;
};

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

	RobotInstruction::FeasibleMotionAxisConfigurationOptions
	feasibleMotionAxisConfigurationOptionsForInstruction(const std::shared_ptr<RobotInstruction::Base>& instruction,
														 QVector<double>* outSeedJointRad = nullptr,
														 const PrecomputedChainSeed* precomputedChainSeed = nullptr);
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& cachedFeasibleAxisConfigurationOptions() const
	{
		return m_cachedFeasibleAxisOptions;
	}
	void invalidateFeasibleAxisConfigurationCache();

	enum class FeasibleAxisProbePurpose
	{
		PropertyPanel,
		SelectionAutoSeed,
	};
	void scheduleDeferredFeasibleAxisProbe(const std::shared_ptr<RobotInstruction::Base>& instruction,
										   FeasibleAxisProbePurpose purpose = FeasibleAxisProbePurpose::PropertyPanel);

	std::shared_ptr<RobotInstruction::Base> findInstructionById(const QString& instructionId) const;

	bool tryCaptureCurrentRobotTcpPose(RobotInstruction::Vec3& outPoseMm, RobotInstruction::Vec3& outEulerDeg,
									   osg::Matrixd* outTcpLocalMat, osg::Matrixd* outTcpRenderWorldMat,
									   QString* outTcpLinkName, QString* errMsg) const;

	void syncRobotKinematicsAfterPoseEdit(const QString& backendId);
	void syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data);

public slots:
	void onUrdfImportRequested(const QString& urdfPath);
	void onSimulationStartTriggered();
	void onSimulationRunRequested();
	void onSimulationStopRequested();
	void onSimulationAddInstructionRequested(RobotInstruction::Type type);
	void onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void onInstructionGroupVisibilityChangeRequested(const std::string& groupId, bool visible);
	void onRobotSimulationTick();
	void onSimulationExportRequested();
	void onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId);
	void onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad);
	void onRobotAxisExternalValuesChanged(const QVector<double>& values);
	void onSimulationTcpDragTeachModeChanged(bool enabled);
	void onTcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void onTcpDragTeachEnded();
	void onRobotCoordinateFramesChanged();
	void onRobotExternalAxesChanged();
	void onCaptureToolFrameFromTcp();
	void onCaptureUserFrameFromTcp();
	void onResetToolFrame();
	void onSimulationDockTabChanged(int index);

	void stopRobotSimulation();
	QVector<double> aggregatedJointAnglesRad() const { return m_aggregatedJointAnglesRad; }
	void restoreAggregatedJointStateAfterProjectLoad(const QVector<double>& allJointAnglesRad);
	void applyProgramStartPoseAfterProjectLoad();
	void refreshInstructionPoseAxes(bool computeReachability = true);
	void refreshPathPlanRawOverlays();
	void refreshBoundPathPlanPreview(const RobotInstruction::RawTrajectory* preferRaw = nullptr);
	void clearBoundPathPlanPreview();
	void refreshPathPlanPreviewForActiveTab(const RobotInstruction::RawTrajectory* preferRaw = nullptr);
	bool isTrajectoryGenerationTabActive() const;
	bool shouldShowTrajectoryGenerationPreview() const;
	void setInstructionGroupVisible(const std::string& groupId, bool visible);
	bool isInstructionGroupVisible(const std::string& groupId) const;
	bool isInstructionVisibleIn3d(const std::string& instructionId) const;
	void setRawTrajectoryPreviewActive(bool active);
	bool rawTrajectoryPreviewActive() const { return m_rawTrajectoryPreviewActive; }
	void refreshSimulationJointListFromCurrentDoc();
	void syncRobotFrameSettingsFromDocument(int instanceIndex);
	void syncRobotExternalAxisSettingsFromDocument(int instanceIndex);
	void syncRobotAxisControlExternalAxes(int instanceIndex);
	void applyAxisControlExternalPose(int instanceIndex, const QVector<double>& values);
	/// 规划/示教结果驱动地轨；无外轴量则忽略
	/// progress01<1 时按段起点插值（播放用）；默认 1 直接落到目标
	void applyExternalAxisFromPlan(int instanceIndex, const RobotInstruction::PlanResult& plan,
								   const RobotInstruction::Base* instruction = nullptr, double progress01 = 1.0,
								   double segmentStartQMm = 0.0);
	void
	refreshRobotCoordinateFrameOverlays(const std::shared_ptr<RobotInstruction::Base>& highlightInstruction = nullptr,
										const QVector<double>* jointAnglesRadLocal = nullptr);
	void refreshRobotCoordinateFrameOverlaysForPlayback();
	void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction,
											 const PrecomputedChainSeed* precomputedChainSeed = nullptr);
	void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction);
	/// 路点 pose 已是世界系（CAD/Unified Apply）时，直接用位姿写 render.tcpWorldMat4，避免按当前机器人基座重算导致轴错位
	void syncInstructionRenderMatricesFromWorldPose(const std::shared_ptr<RobotInstruction::Base>& instruction);

	/// AI 轨迹特征：轨迹页工件 / 预览 / 提交
	bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath);
	bool showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* err = nullptr);
	void clearAiFeatureCandidatePreview();
	bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* summary, QString* err);

private:
	void applyProgramStartPoseAfterProjectLoadImpl();
	void finishProgramStartPoseAfterProjectLoad(int instIdx, QVector<double> startJointQ);
	void captureMotionPreviewProgramStartJoints();
	QVector<double> motionPreviewProgramStartJointsLocal(int nj, int jointOffset) const;
	QVector<double> localJointAnglesForInstance(int instIdx) const;
	bool buildChainSeedJointRadForInstruction(const std::shared_ptr<RobotInstruction::Base>& instruction,
											  QVector<double>& outChainSeed, int* outTargetMotionIndex = nullptr,
											  bool* outChainReliable = nullptr);
	void applyToolFrameChangeToProgram(const RobotCoordinate::RobotCoordinateFrameSet& oldFrames,
									   const RobotCoordinate::RobotCoordinateFrameSet& newFrames,
									   bool activeToolChanged, bool toolGeometryChanged);
	void refreshInstructionPoseAxesWithReachability(const QHash<QString, bool>& reachability);
	bool isPathPlanRawVisible(const std::string& pathPlanId) const;
	void scheduleAsyncMotionReachabilityRefresh();
	void logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad);
	QHash<QString, bool> computeMotionReachabilityForCurrentProgram();
	bool applyTcpDragTeachIkFromPose(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void syncTcpDragTeachAnchorFromCurrentJoints();
	void syncTcpDragExitJointState();

	bool planMotionOnHost(RobotInstruction::Base& instruction, const QVector<double>& seedJointRad, int instanceIndex,
						  const QString& urdfPath, const QString& defaultTcpLinkName, const QString& sceneRootBackendId,
						  RobotInstruction::PlanResult& plan, std::string* planErr) const;

	/// 预览与 Run 共用：示教 CSV → 示教种子 IK → 链式种子 IK → 程序起点 IK
	/// gateTaughtResidual：Run/可达性保持 FK 门控；点击预览可关以省掉双次 FK
	bool planMotionConsistentWithPreview(RobotInstruction::Base& instruction, const QVector<double>& chainSeedQ,
										 const QVector<double>& programStartQ, int instanceIndex,
										 const QString& urdfPath, const QString& defaultTcpLinkName,
										 const QString& sceneRootBackendId,
										 const RobotCoordinate::RobotCoordinateFrameSet& frames,
										 RobotInstruction::PlanResult& outPlan, std::string* planErr,
										 bool persistTaughtOnSuccess, bool gateTaughtResidual = true);

	void ensureInstructionControllerKinematics(IRobotDocumentHost* doc, int instanceIndex, const QString& urdfPath);
	void scheduleInstructionPoseAxesRefresh(bool computeReachability = false);

	IRobotMainWindowHost* m_host = nullptr;
	RobotSimulationDockWidget* m_simulationDock = nullptr;
	ProgramEditService* m_programEditService = nullptr;
	TrajectoryEditSession* m_trajectoryEditSession = nullptr;
	QTimer* m_playbackTimer = nullptr;
	bool m_ownsPlaybackTimer = false;

	RobotInstruction::Controller m_instructionController;
	QString m_cachedInstructionDhUrdfPath;
	quint64 m_poseAxesRefreshToken = 0;
	RobotProgramExecutor m_programExecutor;
	SimulationLogIoSink m_simulationIoSink;
	QVector<double> m_aggregatedJointAnglesRad;
	/// 轴控制已应用到基座的外轴量（与 UI 滑条对应，用于差分更新 P）
	QVector<double> m_axisControlExternalQApplied;
	double m_axisControlExternalAxis[3]{1.0, 0.0, 0.0};
	int m_axisControlExternalInstIdx = -1;
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

	bool m_arcTeachPending = false;
	RobotInstruction::Vec3 m_arcTeachViaPose{};
	RobotInstruction::Vec3 m_arcTeachViaEuler{};
	engine::RigidTransform m_arcTeachViaTransform{};
	std::string m_arcTeachViaJointCsv;
	QString m_arcTeachViaTcpLinkName;
	void cancelArcTeach();

	QSet<QString> m_hiddenInstructionGroupIds;
	RobotInstruction::FeasibleMotionAxisConfigurationOptions m_cachedFeasibleAxisOptions;
	QString m_cachedFeasibleAxisInstructionId;
	QString m_cachedFeasibleAxisFingerprint;
	QVector<double> m_cachedFeasibleAxisSeedJointRad;

	PlanResultCache m_planResultCache;
	QHash<QString, bool> m_motionReachabilityCache;
	quint64 m_reachabilityJobToken = 0;
	int m_reachabilityPendingJobs = 0;
	quint64 m_feasibleAxisJobToken = 0;
	/// 选中链式种子：前缀段末关节，避免每次从 0 同步 IK
	QString m_chainSeedRollFingerprint;
	QVector<QVector<double>> m_chainSeedEndJointsByIndex;
	int m_reachabilityNextBatchStart = 0;
	QVector<double> m_reachabilityBatchRollingQ;

	void invalidateChainSeedRollCache();
	void enqueueReachabilityBatch(int batchStart);

	QString computePlanFingerprint(const RobotInstruction::Base& instruction, const QVector<double>& seedJointRad,
								   const QString& urdfPath, const QString& tcpLinkName) const;

	struct LookaheadConfig
	{
		int maxAdvanceBlocks = 16;
		int maxConcurrentJobs = 4;
		bool enabled = true;
	};
	LookaheadConfig m_lookaheadConfig;
	int m_lookaheadPendingJobs = 0;
	std::vector<const RobotInstruction::Base*> m_currentRunMotions;
	std::string m_lastHighlightedInstructionId;
	size_t m_playbackMotionIndex = 0;
	/// 播放游标处段起点关节（规划 current 的种子）；禁止从 0 扫到 N
	QVector<double> m_playbackRollingSeedQ;
	QVector<double> m_playbackProgramStartQ;
	/// 当前运动段起点外轴位移（mm）；与 plan.externalAxisQ 插值驱动滑轨
	double m_playbackSegmentExternalAxisStartMm = 0.0;
	const RobotInstruction::Base* m_playbackExtInterpMotion = nullptr;
	/// 播放叠加高亮：避免每 tick 扫 instructionList
	std::shared_ptr<RobotInstruction::Base> m_playbackOverlayHighlight;
	QString m_overlayCachedUrdfPath;
	QString m_overlayCachedUrdfRootLink;

	void tickLookaheadPlanning();
	void ensurePlaybackPlansReady();
	bool syncPlanMotionAtIndex(size_t motionIndex);
	static bool isPlaybackUiInteractionBusy();
	bool trySeedJointRadForMotionIndex(size_t targetMotionIndex, const QVector<double>& programStartQ,
									   const QString& urdfPath, const QString& tcpLinkName, int jointCount,
									   QVector<double>& outSeedQ) const;
	void commitPlaybackPlan(const RobotInstruction::Base* motion, size_t motionIndex,
							RobotInstruction::PlanResult plan);
	static void stripPlanTrajectory(RobotInstruction::PlanResult& plan);
};

#endif // ROBOTWIDGET_ROBOTSIMULATIONCONTROLLER_H
