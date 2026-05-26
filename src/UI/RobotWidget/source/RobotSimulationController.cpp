#include "RobotSimulationController.h"
#include "RobotSimulationDockWidget.h"
#include "RobotSimulationMath.h"
#include "RobotInstructionPlanningHelpers.h"
#include "ProgramEditService.h"
#include "TrajectoryEditSession.h"
#include "TrajectoryEditPageWidget.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotMatrixOsgBridge.h"
#include "SimulationCommandWidget.h"
#include "RobotAxisControlWidget.h"
#include "RobotFrameSettingsWidget.h"
#include "RobotProgramExport.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotSceneKinematics.h"
#include "RobotTeachIk.h"
#include "UrdfRobotLoader.h"
#include "RunLogger.h"
#include "RobotOsgUiTypes.h"
#include "../../OsgWidgetCore/inc/ObjectGizmoFrame.h"
#include "../../OsgWidgetCore/inc/OsgScene.h"
#include <Adapters.h>
#include <ToolKinematics.h>
#include <BackendDataBase.h>
#include <BackendDataManager.h>
#include <QMessageBox>
#include <QFile>
#include <QFileDialog>
#include <QSet>
#include <QSignalBlocker>
#include <QSet>
#include <QSignalBlocker>
#include <algorithm>
#include <cmath>
#include <osg/Matrixd>
#include <sstream>
#include <string>

using namespace RobotSimulation;

namespace
{
constexpr double kTaughtReuseResidualMm = 1.0;
constexpr double kPi = 3.14159265358979323846;

int changedJointCount(const QVector<double>& a, const QVector<double>& b, const double eps = 1e-9)
{
	const int n = std::min(a.size(), b.size());
	int changed = 0;
	for (int i = 0; i < n; ++i)
	{
		if (std::abs(a[i] - b[i]) > eps)
		{
			++changed;
		}
	}
	return changed;
}

void wrapJointAnglesTowardSeed(QVector<double>& q, const QVector<double>& seed)
{
	if (q.size() != seed.size())
	{
		return;
	}
	for (int j = 0; j < q.size(); ++j)
	{
		double d = q[j] - seed[j];
		while (d > kPi)
		{
			q[j] -= 2.0 * kPi;
			d = q[j] - seed[j];
		}
		while (d < -kPi)
		{
			q[j] += 2.0 * kPi;
			d = q[j] - seed[j];
		}
	}
}

QVector<double> clampJointStepFromPrevious(
	const QVector<double>& target, const QVector<double>& previous, const double maxStepRad)
{
	QVector<double> out = target;
	if (previous.size() != target.size() || maxStepRad <= 0.0)
	{
		return out;
	}
	for (int j = 0; j < target.size(); ++j)
	{
		const double d = target[j] - previous[j];
		if (std::abs(d) > maxStepRad)
		{
			out[j] = previous[j] + std::copysign(maxStepRad, d);
		}
	}
	return out;
}

double maxJointDeltaRad(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return 0.0;
	}
	double m = 0.0;
	for (int j = 0; j < a.size(); ++j)
	{
		m = std::max(m, std::abs(a[j] - b[j]));
	}
	return m;
}

bool instructionTcpWorldMat4FromTaughtJoints(
	IRobotDocumentHost* doc,
	int instIdx,
	const RobotInstruction::Base& ins,
	const QVector<double>& taughtQ,
	osg::Matrixd& outTcpWorld);

double targetResidualMmForInstruction(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	const RobotInstruction::Base& ins)
{
	engine::RigidTransform target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(ins, target))
	{
		return -1.0;
	}
	BackendMat4 fkTargetMat = BackendMat4::identity();
	QString resolvedFlangeLink;
	if (!RobotSimulationMath::targetInBaseFromUrdfFlangeFk(
			urdfPath, jointQ, frames, fallbackFlangeLink, fkTargetMat, &ins, &resolvedFlangeLink))
	{
		return -1.0;
	}
	const engine::RigidTransform fkTarget = RobotCoordinate::rigidTransformFromBackendMat4(fkTargetMat);
	const Eigen::Vector3d a = target.translationMm();
	const Eigen::Vector3d b = fkTarget.translationMm();
	return (a - b).norm();
}

QVector<double> clampJointAnglesToInstanceLimits(
	IRobotDocumentHost* doc, const int instIdx, const QVector<double>& jointAnglesRad)
{
	QVector<double> out = jointAnglesRad;
	if (!doc || instIdx < 0)
	{
		return out;
	}
	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instIdx, lower, upper);
	const int n = std::min(out.size(), std::min(lower.size(), upper.size()));
	for (int j = 0; j < n; ++j)
	{
		out[j] = std::clamp(out[j], lower[j], upper[j]);
	}
	return out;
}
} // namespace

namespace InstructionPoseDiagState
{
void requestRefresh() {}
bool shouldLog(const std::string&) { return false; }
}

RobotSimulationController::RobotSimulationController(QObject* parent)
	: QObject(parent)
{
}

void RobotSimulationController::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
}

void RobotSimulationController::initializePlanners()
{
	m_instructionController.buildDefaultPlanners();
}

void RobotSimulationController::createSimulationDock(QWidget* parentForTabs)
{
	m_simulationDock = new RobotSimulationDockWidget(parentForTabs);
	m_programEditService = new ProgramEditService(this);
	m_trajectoryEditSession = new TrajectoryEditSession(this);
}

void RobotSimulationController::wireSimulationSignals()
{
	if (!m_host || !m_simulationDock)
	{
		return;
	}
	auto* cmd = m_simulationDock->commandPage();
	auto* axis = m_simulationDock->axisPage();
	auto* frame = m_simulationDock->framePage();
	auto* traj = m_simulationDock->trajectoryEditPage();
	if (m_host && m_host->document())
	{
		m_programEditService->bindStore(&m_host->document()->robotProgramStore());
	}
	m_trajectoryEditSession->bindEditService(m_programEditService);
	m_trajectoryEditSession->bindSimulationController(this);
	if (traj)
	{
		traj->bindEditService(m_programEditService);
		traj->bindSession(m_trajectoryEditSession);
		traj->bindCommandPage(cmd);
	}
	if (cmd)
	{
		cmd->setProgramEditService(m_programEditService);
	}
	connect(cmd, &SimulationCommandWidget::runRequested, this, &RobotSimulationController::onSimulationRunRequested);
	connect(cmd, &SimulationCommandWidget::stopRequested, this, &RobotSimulationController::onSimulationStopRequested);
	connect(cmd, &SimulationCommandWidget::exportProgramRequested, this, &RobotSimulationController::onSimulationExportRequested);
	connect(cmd, &SimulationCommandWidget::addInstructionRequested, this, &RobotSimulationController::onSimulationAddInstructionRequested);
	connect(cmd, &SimulationCommandWidget::instructionSelectionChanged, this,
		&RobotSimulationController::onSimulationInstructionSelectionChanged);
	connect(cmd, &SimulationCommandWidget::robotSelectionChanged, this, &RobotSimulationController::onSimulationRobotSelectionChanged);
	connect(cmd, &SimulationCommandWidget::tcpDragTeachModeChanged, this, &RobotSimulationController::onSimulationTcpDragTeachModeChanged);
	connect(axis, &RobotAxisControlWidget::allJointAnglesChanged, this, &RobotSimulationController::onRobotAxisJointAnglesChanged);
	connect(frame, &RobotFrameSettingsWidget::framesChanged, this, &RobotSimulationController::onRobotCoordinateFramesChanged);
	connect(frame, &RobotFrameSettingsWidget::captureToolFromTcpRequested, this, &RobotSimulationController::onCaptureToolFrameFromTcp);
	connect(frame, &RobotFrameSettingsWidget::captureUserFrameFromTcpRequested, this,
		&RobotSimulationController::onCaptureUserFrameFromTcp);
	connect(frame, &RobotFrameSettingsWidget::resetToolFrameRequested, this, &RobotSimulationController::onResetToolFrame);
}

void RobotSimulationController::attachPlaybackTimer(QTimer* externalTimer)
{
	m_playbackTimer = externalTimer;
	m_ownsPlaybackTimer = false;
	if (m_playbackTimer)
	{
		connect(m_playbackTimer, &QTimer::timeout, this, &RobotSimulationController::onRobotSimulationTick);
	}
}

void RobotSimulationController::onUrdfImportRequested(const QString& urdfPath)
{
	if (m_host)
	{
		m_host->registerUrdfRobot(urdfPath, false);
	}
}
void RobotSimulationController::syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || !data->hasPoseProperty())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg)
	{
		return;
	}
	const QString backendId = QString::fromStdString(data->id());
	bool isSceneRoot = false;
	const int instIdx = doc->robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instIdx < 0)
	{
		return;
	}
	RobotPerLinkKinematicsSlice slice;
	if (!doc->robotPerLinkKinematicsForInstance(instIdx, slice))
	{
		return;
	}
	const BackendVec3 p = data->pose();
	const BackendVec3 r = data->rotation();
	const osg::Quat q = engine::eulerDegToQuat(r.x, r.y, r.z);
	const osg::Matrixd placement =
		ObjectGizmoFrame::outerLocalMatrix(
			osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)), q);

	if (isSceneRoot)
	{
		doc->setRobotBasePlacementWorldForInstance(instIdx, placement);
		slice.robotBasePlacementWorld = placement;
		const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
		QVector<double> localAngles(nj, 0.0);
		int offset = 0;
		for (int i = 0; i < instIdx; ++i)
		{
			offset += doc->robotRevoluteJointCountForInstance(i);
		}
		const int totalJoints = doc->robotRevoluteJointNames().size();
		if (m_aggregatedJointAnglesRad.size() != totalJoints)
		{
			m_aggregatedJointAnglesRad.resize(totalJoints);
		}
		if (m_aggregatedJointAnglesRad.size() >= offset + nj)
		{
			for (int j = 0; j < nj; ++j)
			{
				localAngles[j] = m_aggregatedJointAnglesRad[offset + j];
			}
		}
		else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
		{
			localAngles = m_host->robotAxisControlPage()->jointAnglesRad();
		}
		if (RobotSceneKinematics::applyPerLinkRobotBasePlacement( doc->poseSink(), doc->backend(), slice, localAngles, placement))
		{
			osg->requestRedraw();
		}
		m_host->invalidateInstructionPropertyCache();
		refreshInstructionPoseAxes();
		return;
	}

	osg::Matrixd world;
	if (osg->getBackendRootWorldMatrix(data->id(), world))
	{
		doc->updateRobotLinkOuterBindFromWorld(instIdx, backendId, world);
	}
}

void RobotSimulationController::stopRobotSimulation()
{
	const QVector<double> lastJointAngles = m_programExecutor.jointAnglesRad();
	m_programExecutor.stop();
	if (m_playbackTimer)
	{
		m_playbackTimer->stop();
	}
	if (m_host->simulationCommandPage())
	{
		m_host->simulationCommandPage()->setSimulationRunning(false);
	}
	if (m_simulationDock && m_simulationDock->trajectoryEditPage())
	{
		m_simulationDock->trajectoryEditPage()->setReadOnly(false);
	}
	if (m_host->robotAxisControlPage())
	{
		m_host->robotAxisControlPage()->setInteractionEnabled(true);
		IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
		const int instIdx = m_host->simulationCommandPage() ? m_host->simulationCommandPage()->currentRobotInstanceIndex() : 0;
		if (doc && doc->hasRobotSimulationContext() && instIdx >= 0 && !lastJointAngles.isEmpty())
		{
			const int offset = doc->robotJointOffsetInAggregatedVector(instIdx);
			const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
			if (nj > 0 && m_host->robotAxisControlPage()->jointCount() == nj
				&& offset + nj <= lastJointAngles.size())
			{
				const QVector<double> local = lastJointAngles.mid(offset, nj);
				m_host->robotAxisControlPage()->setJointAnglesRad(local);
				IRobotBackendPoseSink* poseSink = doc->poseSink();
				if (poseSink && m_aggregatedJointAnglesRad.size() == doc->robotRevoluteJointNames().size())
				{
					(void)RobotSceneKinematics::applyJointAnglesForInstance(
						doc, poseSink, instIdx, local, m_aggregatedJointAnglesRad);
				}
			}
		}
	}
	refreshInstructionPoseAxes();
	refreshRobotCoordinateFrameOverlays();
}

void RobotSimulationController::refreshSimulationJointListFromCurrentDoc()
{
	if (!m_host->simulationCommandPage() || !m_host->robotAxisControlPage())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (doc)
	{
		m_host->simulationCommandPage()->setProgramStore(&doc->robotProgramStore());
	}
	if (m_programEditService && doc)
	{
		m_programEditService->bindStore(&doc->robotProgramStore());
	}
	if (m_trajectoryEditSession && doc)
	{
		m_trajectoryEditSession->bindStore(&doc->robotProgramStore());
	}
	if (m_simulationDock && m_simulationDock->trajectoryEditPage() && doc)
	{
		m_simulationDock->trajectoryEditPage()->bindStore(&doc->robotProgramStore());
		m_simulationDock->trajectoryEditPage()->refreshProgramAndGroupCombos();
	}
	if (doc && doc->hasRobotSimulationContext())
	{
		QStringList labels;
		QStringList backendIds;
		const int n = doc->robotKinematicInstanceCount();
		for (int i = 0; i < n; ++i)
		{
			labels.append(doc->robotDisplayLabelForInstance(i));
			backendIds.append(doc->robotSceneBackendIdForInstance(i));
		}
		// 须在 setRobotInstances 之后再绑定指令树，否则 activeProgram 指向空 backendId 或 QHash 重分配后悬空指针
		m_host->simulationCommandPage()->setRobotInstances(labels, backendIds);

		const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
			? m_host->simulationCommandPage()->currentRobotInstanceIndex()
			: 0;
		const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
		m_host->simulationCommandPage()->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instIdx));

		QStringList tcpLinks;
		QString preferredTcp;
		(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
		QStringList childLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
		QSet<QString> uniq;
		if (!preferredTcp.isEmpty())
		{
			uniq.insert(preferredTcp);
			tcpLinks.push_back(preferredTcp);
		}
		for (const QString& l : childLinks)
		{
			if (l.isEmpty() || uniq.contains(l))
			{
				continue;
			}
			uniq.insert(l);
			tcpLinks.push_back(l);
		}
		m_host->simulationCommandPage()->setTcpLinkOptions(tcpLinks, preferredTcp);

		QVector<double> lower;
		QVector<double> upper;
		doc->robotJointLimitsForInstance(instIdx, lower, upper);
		const QStringList jn = doc->robotRevoluteJointNamesForInstance(instIdx);
		if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
		{
			m_host->robotAxisControlPage()->setJoints(jn, lower, upper);
		}
		else
		{
			m_host->robotAxisControlPage()->clearJoints();
		}

		{
			const int total = doc->robotRevoluteJointNames().size();
			const int oldSize = m_aggregatedJointAnglesRad.size();
			if (m_aggregatedJointAnglesRad.size() != total)
			{
				m_aggregatedJointAnglesRad.resize(total);
				for (int i = oldSize; i < total; ++i)
				{
					m_aggregatedJointAnglesRad[i] = 0.0;
				}
			}
		}
		captureMotionPreviewProgramStartJoints();
		syncRobotFrameSettingsFromDocument(instIdx);
		refreshRobotCoordinateFrameOverlays();
	}
	else
	{
		m_host->simulationCommandPage()->setRobotInstances(QStringList(), QStringList());
		m_host->simulationCommandPage()->setRevoluteJointNames(QStringList());
		m_host->simulationCommandPage()->setTcpLinkOptions(QStringList(), QString());
		m_host->robotAxisControlPage()->clearJoints();
		m_aggregatedJointAnglesRad.clear();
		m_motionPreviewProgramStartJointRad.clear();
		m_host->simulationCommandPage()->bindProgramTree();
	}
}

void RobotSimulationController::restoreAggregatedJointStateAfterProjectLoad(
	const QVector<double>& allJointAnglesRad)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int total = doc->robotRevoluteJointNames().size();
	if (allJointAnglesRad.size() != total)
	{
		return;
	}
	m_aggregatedJointAnglesRad = allJointAnglesRad;
	const int instIdx = m_host->simulationCommandPage()
		&& m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_host->robotAxisControlPage() && nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		QVector<double> local(nj);
		for (int j = 0; j < nj; ++j)
		{
			local[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
		QSignalBlocker blocker(m_host->robotAxisControlPage());
		m_host->robotAxisControlPage()->setJointAnglesRad(local);
	}
	captureMotionPreviewProgramStartJoints();
	refreshInstructionPoseAxes();
}

void RobotSimulationController::applyProgramStartPoseAfterProjectLoad()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!doc || !poseSink || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (nj <= 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	bool appliedStartQ = false;
	if (!motions.empty() && motions.front())
	{
		const QVector<double> startQ =
			RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*motions.front());
		if (startQ.size() == nj)
		{
			appliedStartQ = true;
			const QStringList jnamesAll = doc->robotRevoluteJointNames();
			if (m_aggregatedJointAnglesRad.size() != jnamesAll.size())
			{
				m_aggregatedJointAnglesRad = QVector<double>(jnamesAll.size(), 0.0);
			}
			for (int j = 0; j < nj && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
			{
				m_aggregatedJointAnglesRad[jointOffset + j] = startQ[j];
			}
			if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
			{
				QSignalBlocker blocker(m_host->robotAxisControlPage());
				m_host->robotAxisControlPage()->setJointAnglesRad(startQ);
			}
			(void)RobotSceneKinematics::applyJointAnglesForInstance(
				doc, poseSink, instIdx, startQ, m_aggregatedJointAnglesRad);
			captureMotionPreviewProgramStartJoints();
		}
	}
	for (const auto& ins : program)
	{
		if (!ins || !ins->hasPoseProperty())
		{
			continue;
		}
		const QVector<double> taughtQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*ins);
		if (taughtQ.size() != nj)
		{
			continue;
		}
		osg::Matrixd tcpWorld;
		tcpWorld.makeIdentity();
		if (!instructionTcpWorldMat4FromTaughtJoints(doc, instIdx, *ins, taughtQ, tcpWorld))
		{
			continue;
		}
		ins->setExtensionProperty(
			"render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(tcpWorld));
	}
	refreshInstructionPoseAxes();
}

void RobotSimulationController::captureMotionPreviewProgramStartJoints()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		m_motionPreviewProgramStartJointRad.clear();
		return;
	}
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	m_motionPreviewProgramStartJointRad = QVector<double>(jnamesAll.size(), 0.0);
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			m_motionPreviewProgramStartJointRad[jointOffset + j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		const QVector<double> local = m_host->robotAxisControlPage()->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			m_motionPreviewProgramStartJointRad[jointOffset + j] = local[j];
		}
	}
}

QVector<double> RobotSimulationController::localJointAnglesForInstance(const int instIdx) const
{
	QVector<double> out;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || instIdx < 0)
	{
		return out;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	out.resize(nj);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		for (int j = 0; j < nj; ++j)
		{
			out[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		out = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	return out;
}

QVector<double> RobotSimulationController::motionPreviewProgramStartJointsLocal(const int nj, const int jointOffset) const
{
	QVector<double> rollingQ(nj, 0.0);
	if (nj <= 0)
	{
		return rollingQ;
	}
	if (m_motionPreviewProgramStartJointRad.size() > jointOffset)
	{
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			rollingQ[j] = m_motionPreviewProgramStartJointRad[jointOffset + j];
		}
		return rollingQ;
	}
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		return m_host->robotAxisControlPage()->jointAnglesRad();
	}
	return rollingQ;
}

void RobotSimulationController::syncRobotFrameSettingsFromDocument(const int instanceIndex)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->robotFrameSettingsPage() || instanceIndex < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	QStringList linkNames;
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	QSet<QString> uniq;
	for (const QString& l : childLinks)
	{
		if (!l.isEmpty() && !uniq.contains(l))
		{
			uniq.insert(l);
			linkNames.push_back(l);
		}
	}
	m_host->robotFrameSettingsPage()->setLinkNameOptions(linkNames);
	m_host->robotFrameSettingsPage()->setCoordinateFrames(doc->robotCoordinateFramesForInstance(instanceIndex));
	if (m_host->robotFrameSettingsPage())
	{
		m_host->robotFrameSettingsPage()->setUseChinese(m_host->useChinese());
	}
}

void RobotSimulationController::onRobotCoordinateFramesChanged()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	doc->robotCoordinateFramesForInstance(instIdx) = m_host->robotFrameSettingsPage()->coordinateFrames();
	m_host->invalidateInstructionPropertyCache();
	refreshRobotCoordinateFrameOverlays();
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode()
			&& osg->isTcpDragTeachActive())
		{
			const RobotCoordinate::RobotCoordinateFrameSet& frames =
				doc->robotCoordinateFramesForInstance(instIdx);
			if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
			{
				osg->updateTcpDragTeachToolLocalOnFlange(
					RobotSimulationMath::osgMatrixFromRobotRigidFrame(tool->T_flange_tool));
			}
			const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
			if (!urdfPath.isEmpty())
			{
				QStringList revoluteChildLinks;
				QString fallbackFlange;
				(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
				if (!revoluteChildLinks.isEmpty())
				{
					fallbackFlange = revoluteChildLinks.back();
				}
				const QVector<double> jointQ = localJointAnglesForInstance(instIdx);
				engine::RigidTransform fkTarget{};
				if (!jointQ.isEmpty()
					&& RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
						urdfPath, jointQ, frames, fallbackFlange, fkTarget, nullptr, nullptr))
				{
					osg->updateTcpDragTeachFromTarget(fkTarget, false);
				}
			}
		}
	}
	refreshInstructionPoseAxes();
	if (const std::shared_ptr<RobotInstruction::Base> active = m_host->activeInstructionForProperty())
	{
		m_host->refreshInstructionPropertyPanel(active, false);
		const bool tcpDragActive = m_host->simulationCommandPage()
			&& m_host->simulationCommandPage()->tcpDragTeachMode();
		if (!tcpDragActive && RobotInstruction::isMotionWaypointType(active->type()))
		{
			applyRobotPoseForInstructionPreview(active);
		}
	}
}

void RobotSimulationController::onCaptureToolFrameFromTcp()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	QString err;
	if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	const BackendMat4 T_base_tcp =
		RobotCoordinate::tcpInBaseFromPose(pose.x, pose.y, pose.z, euler.x, euler.y, euler.z);
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames);
	std::string flangeLink = activeTool ? RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool) : std::string();
	if (flangeLink.empty())
	{
		flangeLink = RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath, m_host->simulationCommandPage()->selectedTcpLink()).toStdString();
	}
	QVector<double> q;
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() > 0)
	{
		q = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	QHash<QString, osg::Matrixd> linkWorld;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, q, linkWorld, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	const QString flangeQ = QString::fromStdString(flangeLink);
	if (!linkWorld.contains(flangeQ))
	{
		return;
	}
	const BackendMat4 T_base_flange =
		RobotMatrixOsg::backendColMajorFromMatrix(linkWorld.value(flangeQ));
	BackendMat4 invFlange{};
	if (!backend_mat4_invert_rigid(T_base_flange, invFlange))
	{
		return;
	}
	BackendMat4 T_flange_tool{};
	backend_mat4_multiply(invFlange, T_base_tcp, T_flange_tool);
	RobotCoordinate::RobotToolFrame* target = nullptr;
	for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
	{
		if (tf.id == frames.activeToolFrameId)
		{
			target = &tf;
			break;
		}
	}
	if (!target && !frames.toolFrames.empty())
	{
		target = &frames.toolFrames.front();
	}
	if (!target)
	{
		return;
	}
	target->T_flange_tool = RobotCoordinate::mat4ToFrame(T_flange_tool);
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::onResetToolFrame()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
	{
		if (tf.id == frames.activeToolFrameId)
		{
			tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
			break;
		}
	}
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::onCaptureUserFrameFromTcp()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	QString err;
	if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	RobotCoordinate::RobotUserFrame* target = nullptr;
	for (RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
	{
		if (uf.id == frames.activeUserFrameId)
		{
			target = &uf;
			break;
		}
	}
	if (!target && !frames.userFrames.empty())
	{
		target = &frames.userFrames.front();
	}
	if (!target)
	{
		return;
	}
	target->T_base_user.positionMm[0] = pose.x;
	target->T_base_user.positionMm[1] = pose.y;
	target->T_base_user.positionMm[2] = pose.z;
	target->T_base_user.eulerDeg[0] = euler.x;
	target->T_base_user.eulerDeg[1] = euler.y;
	target->T_base_user.eulerDeg[2] = euler.z;
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::refreshRobotCoordinateFrameOverlays(
	const std::shared_ptr<RobotInstruction::Base>& highlightInstruction,
	const QVector<double>* jointAnglesRadLocal)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!osg || !doc || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotRootId = doc->robotSceneBackendIdForInstance(instIdx);
	if (robotRootId.isEmpty())
	{
		return;
	}
	osg->clearRobotFrameOverlays(robotRootId.toStdString());
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	if (!frames.showToolFrameInScene && !frames.showUserFramesInScene)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	QVector<double> jointQ;
	if (jointAnglesRadLocal && !jointAnglesRadLocal->isEmpty())
	{
		jointQ = *jointAnglesRadLocal;
	}
	else
	{
		const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
		if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			jointQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				jointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() > 0)
		{
			jointQ = m_host->robotAxisControlPage()->jointAnglesRad();
		}
	}
	const bool perLink = doc->robotUsesPerLinkBackendsForInstance(instIdx);
	const QString baseLinkBackendId = doc->robotFrameWorldReferenceBackendId(instIdx);
	std::string highlightToolId = frames.activeToolFrameId;
	if (highlightInstruction && highlightInstruction->hasPoseProperty())
	{
		if (const RobotCoordinate::RobotToolFrame* insTool = RobotCoordinate::resolveToolFrameForExtension(
				frames, highlightInstruction->extensionProperties()))
		{
			highlightToolId = insTool->id;
		}
	}
	RobotOsgUi::RobotFrameOverlayUpdate upd;
	upd.robotRootBackendId = robotRootId.toStdString();
	upd.showToolFrames = frames.showToolFrameInScene;
	upd.showUserFrames = frames.showUserFramesInScene;
	for (const RobotCoordinate::RobotToolFrame& tool : frames.toolFrames)
	{
		// H1: instruction TCP axes already mark this tool's target; skip duplicate triad on flange.
		if (highlightInstruction && tool.id == highlightToolId)
		{
			continue;
		}
		RobotOsgUi::RobotFrameOverlayUpdate::ToolEntry te;
		te.name = tool.name;
		te.active = (tool.id == highlightToolId);
		// Waypoint axes (refreshInstructionPoseAxes) mark instruction TCP; tool overlays use flange+T_flange_tool only.
		if (perLink)
		{
			const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
			te.mountBackendId = RobotSimulationMath::linkMeshBackendIdForInstance(doc, instIdx, flangeLink).toStdString();
			te.localMatrix = RobotSimulationMath::osgMatrixFromRobotRigidFrame(tool.T_flange_tool);
			// H2: empty mount would fall back to asm root with flange-local matrix → wrong link (e.g. Link3).
			if (te.mountBackendId.empty())
			{
				continue;
			}
		}
		else
		{
			te.mountBackendId.clear();
			te.localMatrix = RobotSimulationMath::osgMatrixFromBackendMat4(RobotSimulationMath::toolTcpInBaseFromFk(urdfPath, jointQ, frames, tool));
		}
		upd.toolFrames.push_back(std::move(te));
	}
	for (const RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
	{
		RobotOsgUi::RobotFrameOverlayUpdate::UserEntry ue;
		ue.name = uf.name;
		ue.mountBackendId = perLink ? baseLinkBackendId.toStdString() : std::string();
		ue.localMatrix = RobotSimulationMath::osgMatrixFromRobotRigidFrame(uf.T_base_user);
		upd.userFrames.push_back(std::move(ue));
	}
	osg->setRobotFrameOverlays(upd);
}

void RobotSimulationController::refreshRobotCoordinateFrameOverlaysForPlayback()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage())
	{
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	std::shared_ptr<RobotInstruction::Base> highlight;
	if (const RobotInstruction::Base* activeMotion = m_programExecutor.activeMotion())
	{
		for (const std::shared_ptr<RobotInstruction::Base>& ins :
			m_host->simulationCommandPage()->instructionList())
		{
			if (ins && ins.get() == activeMotion)
			{
				highlight = ins;
				break;
			}
		}
	}
	if (!highlight)
	{
		const auto insList = m_host->simulationCommandPage()->instructionList();
		for (auto it = insList.rbegin(); it != insList.rend(); ++it)
		{
			if (*it && (*it)->hasPoseProperty())
			{
				highlight = *it;
				break;
			}
		}
	}
	QVector<double> jointQ;
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		jointQ.resize(nj);
		for (int j = 0; j < nj; ++j)
		{
			jointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	refreshRobotCoordinateFrameOverlays(highlight, jointQ.isEmpty() ? nullptr : &jointQ);
}

void RobotSimulationController::onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId)
{
	(void)sceneBackendId;
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
	{
		onSimulationTcpDragTeachModeChanged(false);
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage() || instanceIndex < 0)
	{
		return;
	}
	syncRobotFrameSettingsFromDocument(instanceIndex);
	refreshRobotCoordinateFrameOverlays();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	m_host->simulationCommandPage()->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instanceIndex));

	QStringList tcpLinks;
	QString preferredTcp;
	(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	QSet<QString> uniq;
	if (!preferredTcp.isEmpty())
	{
		uniq.insert(preferredTcp);
		tcpLinks.push_back(preferredTcp);
	}
	for (const QString& l : childLinks)
	{
		if (l.isEmpty() || uniq.contains(l))
		{
			continue;
		}
		uniq.insert(l);
		tcpLinks.push_back(l);
	}
	m_host->simulationCommandPage()->setTcpLinkOptions(tcpLinks, preferredTcp);

	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instanceIndex, lower, upper);
	const QStringList jn = doc->robotRevoluteJointNamesForInstance(instanceIndex);
	if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
	{
		m_host->robotAxisControlPage()->setJoints(jn, lower, upper);
	}
	captureMotionPreviewProgramStartJoints();
	m_host->invalidateInstructionPropertyCache();
	refreshRobotCoordinateFrameOverlays();
}

void RobotSimulationController::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_programExecutor.isRunning() || m_tcpDragApplyingIk)
	{
		return;
	}
	const bool tcpDragActive = m_host->simulationCommandPage()
		&& m_host->simulationCommandPage()->tcpDragTeachMode();
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!doc || !poseSink)
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage() ? m_host->simulationCommandPage()->currentRobotInstanceIndex() : 0;
	if (instIdx < 0)
	{
		return;
	}
	if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
	{
		m_aggregatedJointAnglesRad = QVector<double>(doc->robotRevoluteJointNames().size(), 0.0);
	}
	const bool applied = RobotSceneKinematics::applyJointAnglesForInstance(
		doc, poseSink, instIdx, jointAnglesRad, m_aggregatedJointAnglesRad);
	if (applied && m_host->osgView())
	{
		m_host->osgView()->requestRedraw();
	}
	if (tcpDragActive)
	{
		syncTcpDragTeachAnchorFromCurrentJoints();
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	refreshRobotCoordinateFrameOverlays();
	if (!m_suppressMotionPreviewStartCapture)
	{
		captureMotionPreviewProgramStartJoints();
		m_host->invalidateInstructionPropertyCache();
	}
}

void RobotSimulationController::onSimulationTcpDragTeachModeChanged(const bool enabled)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!m_host->simulationCommandPage())
	{
		return;
	}
	if (!enabled)
	{
		if (osg && osg->isTcpDragTeachActive())
		{
			osg->endTcpDragTeach();
		}
		syncTcpDragExitJointState();
		m_tcpDragTeachFlangeLink.clear();
		m_tcpDragLastAppliedJointRad.clear();
		m_lastTcpDragTargetValid = false;
		if (IRobotDocumentHost* docOff = m_host ? m_host->document() : nullptr)
		{
			docOff->setSuppressRobotFollowDirtyNotify(false);
			docOff->requestFollowSolveForced();
		}
		if (m_host)
		{
			m_host->runFollowSolveAndSyncForCurrentDocument();
			syncTcpDragExitJointState();
		}
		return;
	}
	if (m_programExecutor.isRunning())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Stop simulation before TCP drag teach."), QStringLiteral("请先停止仿真，再使用末端拖动示教。")));
		}
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !osg || !doc->hasRobotSimulationContext() || !m_host->robotAxisControlPage())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		return;
	}
	m_host->clearBackendObjectSelection(true);
	if (osg->objectSelectionMode())
	{
		osg->setObjectSelectionMode(false);
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	const QString robotRootId = doc->robotSceneBackendIdForInstance(instIdx);
	if (urdfPath.isEmpty() || robotRootId.isEmpty())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		return;
	}
	QStringList revoluteChildLinks;
	QString fallbackFlange;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
	if (!revoluteChildLinks.isEmpty())
	{
		fallbackFlange = revoluteChildLinks.back();
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const bool perLink = doc->robotUsesPerLinkBackendsForInstance(instIdx);
	engine::RigidTransform targetInBase{};
	QString flangeLinkQ;
	if (const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames))
	{
		flangeLinkQ = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool));
	}
	if (!RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
			urdfPath,
			m_host->robotAxisControlPage()->jointAnglesRad(),
			frames,
			flangeLinkQ.isEmpty() ? fallbackFlange : flangeLinkQ,
			targetInBase,
			&flangeLinkQ,
			nullptr))
	{
		RobotInstruction::Vec3 pose{};
		RobotInstruction::Vec3 euler{};
		QString err;
		if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
		{
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(err);
			}
			m_host->simulationCommandPage()->setTcpDragTeachMode(false);
			return;
		}
		targetInBase = engine::RigidTransform::fromTranslationEulerDeg(pose.x, pose.y, pose.z, euler.x, euler.y, euler.z);
		flangeLinkQ = RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	}
	m_tcpDragTeachFlangeLink = flangeLinkQ;
	float modelDiag = 1000.0f;
	if (const QString rootBid = robotRootId; osg->hasBackendObjectBranch(rootBid.toStdString()))
	{
		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		if (osg->tryGetBackendModelCenterMm(rootBid.toStdString(), cx, cy, cz))
		{
			(void)cx;
			(void)cy;
			(void)cz;
		}
	}
	(void)modelDiag;
	std::string mountBackendId = robotRootId.toStdString();
	bool mountOnFlange = false;
	if (perLink && !m_tcpDragTeachFlangeLink.isEmpty())
	{
		const std::string flangeId = RobotSimulationMath::linkMeshBackendIdForInstance(
			doc, instIdx, m_tcpDragTeachFlangeLink.toStdString())
			.toStdString();
		if (!flangeId.empty() && osg->hasBackendObjectBranch(flangeId))
		{
			mountBackendId = flangeId;
			mountOnFlange = true;
		}
	}
	if (!mountOnFlange && !osg->hasBackendObjectBranch(mountBackendId))
	{
		if (!m_tcpDragTeachFlangeLink.isEmpty())
		{
			const std::string flangeId = RobotSimulationMath::linkMeshBackendIdForInstance(
				doc, instIdx, m_tcpDragTeachFlangeLink.toStdString())
				.toStdString();
			if (!flangeId.empty() && osg->hasBackendObjectBranch(flangeId))
			{
				mountBackendId = flangeId;
				mountOnFlange = true;
			}
		}
		if (!osg->hasBackendObjectBranch(mountBackendId))
		{
			const QString refBackendId = doc->robotFrameWorldReferenceBackendId(instIdx);
			if (!refBackendId.isEmpty() && osg->hasBackendObjectBranch(refBackendId.toStdString()))
			{
				mountBackendId = refBackendId.toStdString();
			}
		}
	}
	std::function<bool(osg::Matrixd&)> resolveRobotBaseWorld;
	osg::Matrixd toolLocalOnFlange;
	const osg::Matrixd* toolLocalPtr = nullptr;
	if (mountOnFlange)
	{
		resolveRobotBaseWorld = [this, doc, osg, instIdx](osg::Matrixd& outWorld) -> bool {
			QVector<double> jointQ = localJointAnglesForInstance(instIdx);
			return RobotSimulationMath::robotBaseWorldMatrixForInstance(
				doc, osg, instIdx, outWorld, jointQ.isEmpty() ? nullptr : &jointQ);
		};
		if (const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames))
		{
			toolLocalOnFlange = RobotSimulationMath::osgMatrixFromRobotRigidFrame(activeTool->T_flange_tool);
			toolLocalPtr = &toolLocalOnFlange;
		}
	}
	else if (mountBackendId != robotRootId.toStdString())
	{
		resolveRobotBaseWorld = [this, doc, osg, instIdx](osg::Matrixd& outWorld) -> bool {
			QVector<double> jointQ = localJointAnglesForInstance(instIdx);
			return RobotSimulationMath::robotBaseWorldMatrixForInstance(
				doc, osg, instIdx, outWorld, jointQ.isEmpty() ? nullptr : &jointQ);
		};
	}
	m_tcpDragLastAppliedJointRad.clear();
	m_lastTcpDragTargetValid = false;
	doc->setSuppressRobotFollowDirtyNotify(true);
	doc->clearFollowDirtyBackendIds();
	osg->beginTcpDragTeach(mountBackendId, targetInBase, modelDiag, resolveRobotBaseWorld, toolLocalPtr);
	if (!osg->isTcpDragTeachActive())
	{
		doc->setSuppressRobotFollowDirtyNotify(false);
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Failed to attach TCP drag gizmo."), QStringLiteral("无法挂载 TCP 拖动示教罗盘。")));
		}
	}
	m_lastTcpDragTargetInBase = targetInBase;
	m_lastTcpDragTargetValid = true;
	m_tcpDragTeachIkTimer.start();
}

void RobotSimulationController::syncTcpDragTeachAnchorFromCurrentJoints()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage()
		|| !osg->isTcpDragTeachActive() || m_tcpDragTeachFlangeLink.isEmpty())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	engine::RigidTransform fkTarget{};
	if (!RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
			urdfPath,
			m_host->robotAxisControlPage()->jointAnglesRad(),
			frames,
			m_tcpDragTeachFlangeLink,
			fkTarget,
			nullptr,
			nullptr))
	{
		return;
	}
	osg->updateTcpDragTeachFromTarget(fkTarget, true);
	m_lastTcpDragTargetInBase = fkTarget;
	m_lastTcpDragTargetValid = true;
	m_tcpDragLastAppliedJointRad = localJointAnglesForInstance(instIdx);
}

bool RobotSimulationController::applyTcpDragTeachIkFromPose(
	const double pxMm,
	const double pyMm,
	const double pzMm,
	const double exDeg,
	const double eyDeg,
	const double ezDeg)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage() || m_programExecutor.isRunning())
	{
		return false;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty() || m_tcpDragTeachFlangeLink.isEmpty())
	{
		return false;
	}
	static constexpr int kTcpDragIkMinIntervalMs = 33;
	if (m_tcpDragTeachIkTimer.isValid() && m_tcpDragTeachIkTimer.elapsed() < kTcpDragIkMinIntervalMs)
	{
		return true;
	}
	m_tcpDragTeachIkTimer.restart();
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const int njInst = doc->robotRevoluteJointCountForInstance(instIdx);
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	BackendMat4 toolMat = BackendMat4::identity();
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
	{
		toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
	}
	RobotTeachIk::TeachIkContext ctx;
	ctx.urdfPath = urdfPath;
	ctx.ikLinkName = m_tcpDragTeachFlangeLink;
	ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
	ctx.seedJointRad.clear();
	QVector<double> seedQ;
	if (njInst > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + njInst)
	{
		seedQ.resize(njInst);
		for (int j = 0; j < njInst; ++j)
		{
			seedQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else
	{
		seedQ = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	ctx.seedJointRad.reserve(static_cast<size_t>(seedQ.size()));
	for (double v : seedQ)
	{
		ctx.seedJointRad.push_back(v);
	}
	ctx.useOrientation = true;
	ctx.T_flange_tool = toolMat;
	ctx.maxIkIterations = 20;
	const engine::RigidTransform targetFromEmit = ctx.T_base_target;
	const bool hadPrevTarget = m_lastTcpDragTargetValid;
	const engine::RigidTransform prevTarget = m_lastTcpDragTargetInBase;
	engine::RigidTransform ikTarget = targetFromEmit;
	static constexpr double kTcpDragMaxChaseMmPerIk = 50.0;
	if (hadPrevTarget)
	{
		double tPrev[3]{};
		double tEmit[3]{};
		prevTarget.translationMm(tPrev[0], tPrev[1], tPrev[2]);
		targetFromEmit.translationMm(tEmit[0], tEmit[1], tEmit[2]);
		const double dx = tEmit[0] - tPrev[0];
		const double dy = tEmit[1] - tPrev[1];
		const double dz = tEmit[2] - tPrev[2];
		const double emitLen = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (emitLen > kTcpDragMaxChaseMmPerIk)
		{
			const double s = kTcpDragMaxChaseMmPerIk / emitLen;
			ikTarget = engine::RigidTransform::fromTranslationEulerDeg(
				tPrev[0] + dx * s,
				tPrev[1] + dy * s,
				tPrev[2] + dz * s,
				exDeg,
				eyDeg,
				ezDeg);
		}
	}
	ctx.T_base_target = ikTarget;
	const engine::RigidTransform targetForIkStep = ikTarget;
	const RobotTeachIk::TeachIkResult ik = RobotTeachIk::solveTeachIk(ctx);
	if (!ik.ok)
	{
		return false;
	}
	QVector<double> qRad;
	qRad.reserve(static_cast<int>(ik.jointRad.size()));
	for (double v : ik.jointRad)
	{
		qRad.push_back(v);
	}
	QVector<double> qClamped = clampJointAnglesToInstanceLimits(doc, instIdx, qRad);
	const bool anyClamped = (qClamped.size() == qRad.size())
		&& !std::equal(qClamped.begin(), qClamped.end(), qRad.begin());
	wrapJointAnglesTowardSeed(qClamped, seedQ);
	const bool hasPrevDragQ = (m_tcpDragLastAppliedJointRad.size() == qClamped.size());
	const double maxDeltaIk =
		hasPrevDragQ ? maxJointDeltaRad(qClamped, m_tcpDragLastAppliedJointRad) : 1.0;
	static constexpr double kTcpDragMaxJointStepRad = 0.12;
	if (hasPrevDragQ)
	{
		qClamped = clampJointStepFromPrevious(qClamped, m_tcpDragLastAppliedJointRad, kTcpDragMaxJointStepRad);
	}
	const double maxJointDelta =
		hasPrevDragQ ? maxJointDeltaRad(qClamped, m_tcpDragLastAppliedJointRad) : 1.0;
	static constexpr double kTcpDragMinJointApplyRad = 0.002;
	if (maxJointDelta < kTcpDragMinJointApplyRad)
	{
		return true;
	}
	if (hadPrevTarget && !qClamped.isEmpty())
	{
		engine::RigidTransform fkMotion{};
		if (RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
				urdfPath, qClamped, frames, m_tcpDragTeachFlangeLink, fkMotion, nullptr, nullptr))
		{
			double tPrev[3]{};
			double tTgt[3]{};
			double tFk[3]{};
			prevTarget.translationMm(tPrev[0], tPrev[1], tPrev[2]);
			targetForIkStep.translationMm(tTgt[0], tTgt[1], tTgt[2]);
			fkMotion.translationMm(tFk[0], tFk[1], tFk[2]);
			engine::RigidTransform fkSeedPose{};
			if (RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
					urdfPath, seedQ, frames, m_tcpDragTeachFlangeLink, fkSeedPose, nullptr, nullptr))
			{
				double tSeed[3]{};
				fkSeedPose.translationMm(tSeed[0], tSeed[1], tSeed[2]);
				const double wantDx = tTgt[0] - tPrev[0];
				const double wantDy = tTgt[1] - tPrev[1];
				const double wantDz = tTgt[2] - tPrev[2];
				const double fkDx = tFk[0] - tSeed[0];
				const double fkDy = tFk[1] - tSeed[1];
				const double fkDz = tFk[2] - tSeed[2];
				const double wantLen = std::sqrt(wantDx * wantDx + wantDy * wantDy + wantDz * wantDz);
				const double fkLen = std::sqrt(fkDx * fkDx + fkDy * fkDy + fkDz * fkDz);
				const double alignDot = wantDx * fkDx + wantDy * fkDy + wantDz * fkDz;
				const double alignRatio =
					(wantLen > 1e-6 && fkLen > 1e-6) ? (alignDot / (wantLen * fkLen)) : 1.0;
				static constexpr double kTcpDragMinWantLenMm = 2.0;
				static constexpr double kTcpDragMinAlignRatio = 0.35;
				const bool rejectOpposite = (wantLen >= kTcpDragMinWantLenMm && alignDot < 0.0);
				const bool rejectMisaligned =
					(wantLen >= 5.0 && fkLen >= kTcpDragMinWantLenMm
					 && alignRatio < kTcpDragMinAlignRatio);
				if (rejectOpposite || rejectMisaligned)
				{
					return true;
				}
			}
		}
	}
	m_lastTcpDragTargetInBase = targetForIkStep;
	m_lastTcpDragTargetValid = true;
	m_tcpDragLastAppliedJointRad = qClamped;
	if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
	{
		m_aggregatedJointAnglesRad = QVector<double>(doc->robotRevoluteJointNames().size(), 0.0);
	}
	for (int j = 0; j < njInst && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
	{
		m_aggregatedJointAnglesRad[jointOffset + j] = qClamped[j];
	}
	m_tcpDragApplyingIk = true;
	m_suppressMotionPreviewStartCapture = true;
	(void)RobotSceneKinematics::applyJointAnglesForInstance(
		doc, doc->poseSink(), instIdx, qClamped, m_aggregatedJointAnglesRad);
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == njInst)
	{
		m_host->robotAxisControlPage()->setJointAnglesRadSilent(qClamped);
	}
	if (anyClamped && m_host->runInfoPage())
	{
		m_host->appendRunWarning(m_host->i18n(
			QStringLiteral("TCP drag IK exceeded joint limits; angles were clamped to URDF range."),
			QStringLiteral("末端拖动 IK 超出关节限位，已按 URDF 范围钳位。")));
	}
	m_suppressMotionPreviewStartCapture = false;
	m_tcpDragApplyingIk = false;
	osg->requestRedraw();
	return true;
}

void RobotSimulationController::onTcpDragTeachPoseChanged(
	const double pxMm,
	const double pyMm,
	const double pzMm,
	const double exDeg,
	const double eyDeg,
	const double ezDeg)
{
	(void)applyTcpDragTeachIkFromPose(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
}

void RobotSimulationController::syncTcpDragExitJointState()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QVector<double> local = localJointAnglesForInstance(instIdx);
	if (local.isEmpty())
	{
		return;
	}
	IRobotBackendPoseSink* poseSink = doc->poseSink();
	if (poseSink)
	{
		m_tcpDragApplyingIk = true;
		m_suppressMotionPreviewStartCapture = true;
		(void)RobotSceneKinematics::applyJointAnglesForInstance(
			doc, poseSink, instIdx, local, m_aggregatedJointAnglesRad);
		m_suppressMotionPreviewStartCapture = false;
		m_tcpDragApplyingIk = false;
	}
	if (m_host->robotAxisControlPage()->jointCount() == local.size())
	{
		m_host->robotAxisControlPage()->setJointAnglesRadSilent(local);
	}
	refreshRobotCoordinateFrameOverlays();
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		osg->requestRedraw();
	}
}

void RobotSimulationController::onTcpDragTeachEnded()
{
	if (m_host->simulationCommandPage())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
	}
	if (m_host->simulationCommandPage() && m_host->document())
	{
		const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
		const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
			m_host->simulationCommandPage()->instructions(robotBackendId);
		if (RobotInstruction::collectMotionInstructions(program).empty())
		{
			captureMotionPreviewProgramStartJoints();
		}
	}
}

void RobotSimulationController::onSimulationStopRequested()
{
	stopRobotSimulation();
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(QStringLiteral("Simulation stopped."), QStringLiteral("仿真已停止。")));
	}
}

void RobotSimulationController::onSimulationRunRequested()
{
	onSimulationStartTriggered();
}

void RobotSimulationController::onSimulationExportRequested()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(
				QStringLiteral("Import a robot (URDF) first, then export the program."), QStringLiteral("请先导入机器人(URDF)，再导出程序。")));
		}
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (RobotSimulationMath::buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_instructionController.setDhRows(dhRows);
		}
		else
		{
			m_instructionController.clearDhRows();
		}
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> instructions =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions =
		RobotInstruction::collectMotionInstructions(instructions);
	if (motions.empty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("No motion instructions to export."), QStringLiteral("没有可导出的运动指令。")));
		}
		return;
	}
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath,
		m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	std::vector<RobotInstruction::PlanResult> plans;
	plans.reserve(motions.size());
	int failedCount = 0;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		const RobotInstruction::Base* motionPtr = motions[mi];
		if (!motionPtr)
		{
			RobotInstruction::PlanResult empty{};
			empty.ok = false;
			empty.summary = "Invalid instruction";
			plans.push_back(std::move(empty));
			++failedCount;
			continue;
		}
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*ins);
		RobotInstructionPlanning::prepareMotionInstructionForPlanning(
			*ins,
			rollingQ,
			doc,
			m_host->osgView(),
			instIdx,
			urdfPath,
			defaultTcpLinkName.toStdString(),
			&doc->robotCoordinateFramesForInstance(instIdx));
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		if (!m_instructionController.validate(*ins, &planErr))
		{
			plan.ok = false;
			plan.summary = planErr.empty() ? "Validation failed" : planErr;
			++failedCount;
		}
		else if (!planMotionOnHost(*ins, rollingQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, plan, &planErr))
		{
			plan.ok = false;
			if (!planErr.empty())
			{
				plan.summary = planErr;
			}
			++failedCount;
		}
		RobotInstructionPlanning::restoreInstructionPose(*ins, backup);
		if (plan.ok && !plan.jointTargetsRad.empty()
			&& plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		plans.push_back(std::move(plan));
	}
	RobotProgramExport::RobotProgramExportResult exportResult;
	std::string buildErr;
	if (!RobotProgramExport::buildExportResult(
			motions,
			plans,
			robotBackendId.toStdString(),
			urdfPath.toStdString(),
			exportResult,
			&buildErr))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(QString::fromStdString(buildErr));
		}
		return;
	}
	const QString defaultName = QStringLiteral("robot_program.json");
	const QString path = QFileDialog::getSaveFileName(
		nullptr,
		m_host->i18n(QStringLiteral("Export robot program"), QStringLiteral("导出机器人程序")),
		defaultName,
		m_host->i18n(QStringLiteral("JSON (*.json);;CSV (*.csv)"), QStringLiteral("JSON (*.json);;CSV (*.csv)")));
	if (path.isEmpty())
	{
		return;
	}
	std::string fileBody;
	std::string writeErr;
	const bool asCsv = path.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive);
	const bool okWrite = asCsv ? RobotProgramExport::writeExportResultToCsv(exportResult, fileBody, &writeErr)
								 : RobotProgramExport::writeExportResultToJson(exportResult, fileBody, &writeErr);
	if (!okWrite)
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(QString::fromStdString(writeErr));
		}
		return;
	}
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Cannot write file: %1").arg(path),
					QStringLiteral("无法写入文件：%1").arg(path)));
		}
		return;
	}
	f.write(fileBody.c_str(), static_cast<qint64>(fileBody.size()));
	f.close();
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(
			m_host->i18n(
				QStringLiteral("Exported program to %1 (%2 motion points, %3 IK failures).")
					.arg(path)
					.arg(exportResult.points.size())
					.arg(failedCount),
				QStringLiteral("已导出程序到 %1（%2 个运动点，%3 次 IK 失败）。")
					.arg(path)
					.arg(exportResult.points.size())
					.arg(failedCount)));
	}
	refreshInstructionPoseAxes();
}

bool RobotSimulationController::tryCaptureCurrentRobotTcpPose(
	RobotInstruction::Vec3& outPoseMm,
	RobotInstruction::Vec3& outEulerDeg,
	osg::Matrixd* outTcpLocalMat,
	osg::Matrixd* outTcpRenderWorldMat,
	QString* outTcpLinkName,
	QString* errMsg) const
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !doc->hasRobotSimulationContext())
	{
		if (errMsg)
		{
			*errMsg = m_host->i18n(QStringLiteral("Robot simulation context is not ready."), QStringLiteral("机器人仿真上下文尚未就绪。"));
		}
		return false;
	}
	const int instIdx = m_host->simulationCommandPage() && m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = m_host->i18n(QStringLiteral("URDF path is empty."), QStringLiteral("URDF 路径为空。"));
		}
		return false;
	}
	const QString jointPrefix = doc->robotJointKeyPrefixForInstance(instIdx);
	const QStringList jointsLocal = doc->robotRevoluteJointNamesForInstance(instIdx);
	QVector<double> q(jointsLocal.size(), 0.0);
	QString qSource = QStringLiteral("zero-fallback");
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	osg::Matrixd robotBaseWorld;
	robotBaseWorld.makeIdentity();
	const bool hasRobotBaseWorld = RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, robotBaseWorld);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + q.size())
	{
		for (int j = 0; j < q.size(); ++j)
		{
			q[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
		qSource = QStringLiteral("aggregated");
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() > 0)
	{
		const QVector<double> sliderQ = m_host->robotAxisControlPage()->jointAnglesRad();
		const int nCopy = std::min(q.size(), sliderQ.size());
		for (int j = 0; j < nCopy; ++j)
		{
			q[j] = sliderQ[j];
		}
		if (nCopy == q.size() && nCopy > 0)
		{
			qSource = QStringLiteral("slider");
		}
		else if (nCopy > 0)
		{
			qSource = QStringLiteral("slider-partial");
		}
	}
	const QString lastJointName = jointsLocal.isEmpty() ? QString() : (jointPrefix + jointsLocal.back());
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString computeErr;
	const bool hasLinkFk = UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, q, linkWorldByName, &computeErr);

	QString fallbackFlangeLink;
	{
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			fallbackFlangeLink = revoluteChildLinks.back();
		}
	}
	if (fallbackFlangeLink.isEmpty() && m_host->simulationCommandPage())
	{
		fallbackFlangeLink = m_host->simulationCommandPage()->selectedTcpLink();
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	QString tcpLinkName;
	osg::Matrixd tcpLocal;
	bool hasTcpLocal = false;
	QString tcpSource = QStringLiteral("None");
	osg::Matrixd tcpRenderWorld;
	bool capturedFromScene = false;
	BackendMat4 capturedTargetInBase{};
	bool hasCapturedTargetInBase = false;
	// 优先 URDF 法兰 FK×工具系；per-link 场景 PAT 世界矩阵不可信，禁止先用 SceneFlangeBackend
	if (hasLinkFk)
	{
		if (RobotSimulationMath::targetInBaseFromUrdfFlangeFk(
				urdfPath, q, frames, fallbackFlangeLink, capturedTargetInBase, nullptr, &tcpLinkName))
		{
			tcpLocal = RobotMatrixOsg::matrixFromBackendColMajor(capturedTargetInBase);
			hasTcpLocal = true;
			hasCapturedTargetInBase = true;
			tcpSource = QStringLiteral("UrdfFlangeFk+Tool");
		}
	}
	osg::Matrixd tcpLocalFromHierarchy;
	bool hasHierarchyLocal = false;
	if (!lastJointName.isEmpty())
	{
		if (osg::MatrixTransform* jointMt = doc->robotJointMatrixTransform(lastJointName))
		{
			osg::Matrixd jointWorld;
			if (RobotSimulationMath::matrixFromNodeWorld(jointMt, jointWorld))
			{
				osg::Matrixd invBase;
				invBase.makeIdentity();
				if (hasRobotBaseWorld)
				{
					invBase = osg::Matrixd::inverse(robotBaseWorld);
				}
				tcpLocalFromHierarchy = invBase * jointWorld;
				hasHierarchyLocal = true;
			}
		}
	}
	if (!hasTcpLocal
		&& RobotSimulationMath::captureTcpFromSceneFlangeBackend(
			doc, osg, instIdx, frames, fallbackFlangeLink, robotBaseWorld, tcpLocal, tcpRenderWorld, tcpLinkName, tcpSource))
	{
		hasTcpLocal = true;
		capturedFromScene = true;
	}
	if (!hasTcpLocal && hasHierarchyLocal)
	{
		const BackendMat4 T_flange_tool = RobotSimulationMath::toolMat4ForFrames(frames, nullptr);
		tcpLocal = tcpLocalFromHierarchy * RobotSimulationMath::osgMatrixFromBackendMat4(T_flange_tool);
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
		{
			tcpLinkName = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *tool));
		}
		else
		{
			tcpLinkName = fallbackFlangeLink;
		}
		hasTcpLocal = true;
		tcpSource = QStringLiteral("HierarchyJoint+Tool");
	}
	if (!hasTcpLocal)
	{
		if (errMsg)
		{
			if (!hasLinkFk)
			{
				const QString detail = computeErr.isEmpty()
					? m_host->i18n(QStringLiteral("URDF forward kinematics failed."), QStringLiteral("URDF 正解计算失败。"))
					: computeErr;
				*errMsg = m_host->i18n(
					QStringLiteral("Cannot evaluate TCP: %1").arg(detail),
					QStringLiteral("无法求 TCP：%1").arg(detail));
			}
			else
			{
				std::string flangeLink;
				if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
				{
					flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
				}
				else
				{
					flangeLink = fallbackFlangeLink.toStdString();
				}
				const QString flangeQ = QString::fromStdString(flangeLink);
				if (flangeQ.isEmpty())
				{
					*errMsg = m_host->i18n(
						QStringLiteral("Flange link name is not configured."),
						QStringLiteral("未配置法兰连杆名。"));
				}
				else if (!linkWorldByName.contains(flangeQ))
				{
					*errMsg = m_host->i18n(
						QStringLiteral("Link '%1' not in URDF FK result (check tool frame flange link).")
							.arg(flangeQ),
						QStringLiteral("连杆「%1」不在 URDF 正解结果中（请检查工具系法兰连杆）。")
							.arg(flangeQ));
				}
				else if (!lastJointName.isEmpty() && !doc->robotJointMatrixTransform(lastJointName))
				{
					*errMsg = m_host->i18n(
						QStringLiteral("Per-link robot has no joint scene node '%1'; use URDF FK path.")
							.arg(lastJointName),
						QStringLiteral("每连杆机器人无关节场景节点「%1」；请使用 URDF 正解路径。")
							.arg(lastJointName));
				}
				else
				{
					*errMsg = m_host->i18n(QStringLiteral("Cannot evaluate TCP world transform."), QStringLiteral("无法获取末端世界坐标。"));
				}
			}
		}
		return false;
	}

	// tcpLocal：URDF 基座系 T_base_target（工具系原点）
	const osg::Matrixd tcpWorld = tcpLocal;
	const osg::Matrixd renderWorld = capturedFromScene ? tcpRenderWorld : (tcpWorld * robotBaseWorld);
	// 钀界洏 pose/euler锛氱洿鎺ョ敱 URDF FK 脳 宸ュ叿绯诲緱鍒?RigidTransform锛堜笌 IK/娈嬪樊鍚屼竴璺緞锛夈€?
	if (hasCapturedTargetInBase && hasLinkFk)
	{
		engine::RigidTransform target{};
		QString flangeLinkQ;
		if (RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
				urdfPath, q, frames, fallbackFlangeLink, target, &flangeLinkQ, nullptr))
		{
			target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
			target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
			tcpLocal = engine::osgMatrixFromRigidTransform(target);
			capturedTargetInBase = RobotCoordinate::backendMat4FromRigidTransform(target);
		}
		else
		{
			const engine::RigidTransform target = RobotCoordinate::rigidTransformFromBackendMat4(capturedTargetInBase);
			target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
			target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
			tcpLocal = engine::osgMatrixFromRigidTransform(target);
		}
	}
	else if (hasCapturedTargetInBase)
	{
		const engine::RigidTransform target = RobotCoordinate::rigidTransformFromBackendMat4(capturedTargetInBase);
		target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
		target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
		tcpLocal = engine::osgMatrixFromRigidTransform(target);
	}
	else
	{
		const osg::Vec3d t = tcpWorld.getTrans();
		const osg::Vec3f euler = OsgScene::quatToEulerDeg(tcpWorld.getRotate());
		outPoseMm.x = t.x();
		outPoseMm.y = t.y();
		outPoseMm.z = t.z();
		outEulerDeg.x = euler.x();
		outEulerDeg.y = euler.y();
		outEulerDeg.z = euler.z();
	}
	if (outTcpLocalMat)
	{
		*outTcpLocalMat = tcpWorld;
	}
	if (outTcpRenderWorldMat)
	{
		*outTcpRenderWorldMat = renderWorld;
	}
	if (outTcpLinkName)
	{
		// Planning/IK reference should be a link name (URDF link frame key).
		*outTcpLinkName = tcpLinkName;
	}
	return true;
}

void RobotSimulationController::onSimulationAddInstructionRequested(RobotInstruction::Type type)
{
	if (!m_host->simulationCommandPage())
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	osg::Matrixd tcpLocalMat;
	osg::Matrixd tcpRenderWorldMat;
	QString tcpLinkName;
	QString err;
	const int capInstIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	IRobotDocumentHost* capDoc = m_host->document();
	IRobotOsgViewHost* capOsg = m_host->osgView();
	const QVector<double> qLocal = localJointAnglesForInstance(capInstIdx);
	bool usedGizmoTarget = false;
	if (m_lastTcpDragTargetValid)
	{
		m_lastTcpDragTargetInBase.translationMm(pose.x, pose.y, pose.z);
		m_lastTcpDragTargetInBase.eulerDegForDisplay(euler.x, euler.y, euler.z);
		tcpLocalMat = engine::osgMatrixFromRigidTransform(m_lastTcpDragTargetInBase);
		tcpRenderWorldMat = tcpLocalMat;
		if (capDoc && capOsg)
		{
			osg::Matrixd robotBaseWorld;
			robotBaseWorld.makeIdentity();
			if (RobotSimulationMath::robotBaseWorldMatrixForInstance(
					capDoc, capOsg, capInstIdx, robotBaseWorld, qLocal.isEmpty() ? nullptr : &qLocal))
			{
				tcpRenderWorldMat = tcpLocalMat * robotBaseWorld;
			}
		}
		usedGizmoTarget = true;
		if (capDoc && !capDoc->robotUrdfAbsolutePathForInstance(capInstIdx).isEmpty())
		{
			QStringList revoluteChildLinks;
			QString fallbackFlange;
			(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(
				capDoc->robotUrdfAbsolutePathForInstance(capInstIdx), revoluteChildLinks, nullptr);
			if (!revoluteChildLinks.isEmpty())
			{
				fallbackFlange = revoluteChildLinks.back();
			}
			if (m_host->simulationCommandPage())
			{
				fallbackFlange = RobotSimulationMath::defaultTcpLinkNameForUrdf(
					capDoc->robotUrdfAbsolutePathForInstance(capInstIdx),
					m_host->simulationCommandPage()->selectedTcpLink());
			}
			tcpLinkName = fallbackFlange;
		}
	}
	else if (!tryCaptureCurrentRobotTcpPose(pose, euler, &tcpLocalMat, &tcpRenderWorldMat, &tcpLinkName, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	const std::shared_ptr<RobotInstruction::Base> ins =
		m_host->simulationCommandPage()->appendInstructionFromCurrentPose(type, pose, euler, true);
	m_host->invalidateInstructionPropertyCache();
	if (ins)
	{
		RobotInstruction::writeTargetTransformToInstruction(*ins, m_lastTcpDragTargetValid ? m_lastTcpDragTargetInBase
																							: engine::RigidTransform::fromTranslationEulerDeg(
																								  pose.x, pose.y, pose.z, euler.x, euler.y, euler.z));
		const std::string matCsv = RobotSimulationMath::encodeMatrix4Csv(tcpLocalMat);
		const std::string renderMatCsv = RobotSimulationMath::encodeMatrix4Csv(tcpRenderWorldMat);
		const osg::Matrixd renderWorldToFk = tcpLocalMat * osg::Matrixd::inverse(tcpRenderWorldMat);
		const std::string renderToFkCsv = RobotSimulationMath::encodeMatrix4Csv(renderWorldToFk);
		const osg::Vec3d deltaPosMm = tcpLocalMat.getTrans() - tcpRenderWorldMat.getTrans();
		std::ostringstream deltaOss;
		deltaOss.imbue(std::locale::classic());
		deltaOss << deltaPosMm.x() << "," << deltaPosMm.y() << "," << deltaPosMm.z();
		ins->setExtensionProperty("render.tcpWorldMat4", renderMatCsv);
		ins->setExtensionProperty("render.tcpLocalMat4", matCsv);
		ins->setExtensionProperty("render.tcpLinkName", tcpLinkName.toStdString());
		ins->setExtensionProperty("context.renderWorldToFkMat4", renderToFkCsv);
		ins->setExtensionProperty("context.renderToFkDeltaPosMmCsv", deltaOss.str());
		ins->setExtensionProperty("context.poseFrame", "base_tool_origin");
		ins->setExtensionProperty("context.capturedTcpLinkName", tcpLinkName.toStdString());
		if (IRobotDocumentHost* capDoc = m_host->document())
		{
			const int capInstIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
				? m_host->simulationCommandPage()->currentRobotInstanceIndex()
				: 0;
			const RobotCoordinate::RobotCoordinateFrameSet& capFrames =
				capDoc->robotCoordinateFramesForInstance(capInstIdx);
			if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(capFrames))
			{
				const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
				ins->setExtensionProperty(
					RobotCoordinate::kExtContextToolFrameMat4, RobotCoordinate::encodeMat4Csv(toolMat));
				const std::string flangeLink =
					RobotCoordinate::effectiveFlangeLinkName(capFrames, *tool);
				if (!flangeLink.empty())
				{
					ins->setExtensionProperty("context.flangeLinkName", flangeLink);
				}
				ins->setExtensionProperty("context.activeToolFrameId", tool->id);
				ins->setExtensionProperty(RobotCoordinate::kExtMotionToolFrameId, tool->id);
			}
			if (const RobotCoordinate::RobotUserFrame* uf = RobotCoordinate::activeUserFrame(capFrames))
			{
				ins->setExtensionProperty(RobotCoordinate::kExtMotionUserFrameId, uf->id);
			}
			ins->setExtensionProperty(RobotCoordinate::kExtMotionTargetFrame, "base");
			const QString capUrdf = capDoc->robotUrdfAbsolutePathForInstance(capInstIdx);
			if (!capUrdf.isEmpty())
			{
				ins->setExtensionProperty("context.urdfPath", capUrdf.toStdString());
			}
			if (!tcpLinkName.isEmpty())
			{
				ins->setExtensionProperty("context.tcpLinkName", tcpLinkName.toStdString());
			}
			QStringList revoluteChildLinks;
			QString fallbackFlange;
			if (!capUrdf.isEmpty())
			{
				(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(capUrdf, revoluteChildLinks, nullptr);
				if (!revoluteChildLinks.isEmpty())
				{
					fallbackFlange = revoluteChildLinks.back();
				}
			}
			if (!qLocal.isEmpty())
			{
				ins->setExtensionProperty(
					"context.currentJointRadCsv",
					RobotInstructionPlanning::encodeJointAnglesRadCsv(qLocal));
				osg::Matrixd tcpWorldFromJoints;
				if (instructionTcpWorldMat4FromTaughtJoints(capDoc, capInstIdx, *ins, qLocal, tcpWorldFromJoints))
				{
					BackendMat4 targetInBase = BackendMat4::identity();
					(void)RobotSimulationMath::targetInBaseFromUrdfFlangeFk(
						capUrdf,
						qLocal,
						capFrames,
						fallbackFlange,
						targetInBase,
						ins.get(),
						nullptr);
					const osg::Matrixd tcpLocalFromJoints =
						RobotMatrixOsg::matrixFromBackendColMajor(targetInBase);
					ins->setExtensionProperty(
						"render.tcpWorldMat4",
						RobotSimulationMath::encodeMatrix4Csv(tcpWorldFromJoints));
					ins->setExtensionProperty(
						"render.tcpLocalMat4",
						RobotSimulationMath::encodeMatrix4Csv(tcpLocalFromJoints));
				}
			}
			if (RunLogger::isDiagnosticsEnabled() && m_host->runInfoPage() && m_host->robotAxisControlPage() && !capUrdf.isEmpty())
			{
				QString toolName = QStringLiteral("-");
				if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(capFrames))
				{
					toolName = QString::fromStdString(tool->name);
				}
				BackendMat4 T_target{};
				QString flangeLinkQ;
				if (RobotSimulationMath::targetInBaseFromUrdfFlangeFk(
						capUrdf,
						qLocal,
						capFrames,
						fallbackFlange,
						T_target,
						ins.get(),
						&flangeLinkQ))
				{
					const RobotCoordinate::RobotRigidFrame fTool = RobotCoordinate::mat4ToFrame(T_target);
					const RobotCoordinate::RobotRigidFrame fFlange = fTool;
					m_host->appendRunInfo(
						QStringLiteral(
							"[Teach] tool=%1 path=UrdfFlange*Tool flange=(%2,%3,%4) FK_tool=(%5,%6,%7) pose=(%8,%9,%10) link=%11")
							.arg(toolName)
							.arg(fFlange.positionMm[0], 0, 'f', 2)
							.arg(fFlange.positionMm[1], 0, 'f', 2)
							.arg(fFlange.positionMm[2], 0, 'f', 2)
							.arg(fTool.positionMm[0], 0, 'f', 2)
							.arg(fTool.positionMm[1], 0, 'f', 2)
							.arg(fTool.positionMm[2], 0, 'f', 2)
							.arg(ins->pose().x, 0, 'f', 2)
							.arg(ins->pose().y, 0, 'f', 2)
							.arg(ins->pose().z, 0, 'f', 2)
							.arg(flangeLinkQ));
				}
			}
		}
		{
			const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
			const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
				m_host->simulationCommandPage()->instructions(robotBackendId);
			if (RobotInstruction::collectMotionInstructions(program).size() <= 1)
			{
				captureMotionPreviewProgramStartJoints();
			}
		}
		m_skipInstructionPreviewOnce = true;
		onSimulationInstructionSelectionChanged(ins);
		m_lastTcpDragTargetValid = false;
		if (ins->hasMotionAxisConfigurationProperty() && m_host->robotAxisControlPage()
			&& m_host->robotAxisControlPage()->jointCount() > 0)
		{
			QVector<double> seedQ = m_host->robotAxisControlPage()->jointAnglesRad();
			RobotInstruction::FeasibleMotionAxisConfigurationOptions feasible =
				feasibleMotionAxisConfigurationOptionsForInstruction(ins, &seedQ);
			m_host->applySuggestedAxisPresetFromSeedIfNeeded(ins, seedQ, feasible);
			ins->setExtensionProperty("context.axisConfigSeeded", "1");
		}
	}
	else
	{
		refreshInstructionPoseAxes();
	}
}

void RobotSimulationController::invalidateFeasibleAxisConfigurationCache()
{
	m_cachedFeasibleAxisInstructionId.clear();
	m_cachedFeasibleAxisFingerprint.clear();
	m_cachedFeasibleAxisSeedJointRad.clear();
	m_cachedFeasibleAxisOptions = {};
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions RobotSimulationController::feasibleMotionAxisConfigurationOptionsForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	QVector<double>* outSeedJointRad)
{
	RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
	if (!instruction || !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return out;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return out;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return out;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return out;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return out;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (RobotSimulationMath::buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_instructionController.setDhRows(dhRows);
		}
		else
		{
			m_instructionController.clearDhRows();
		}
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const std::string targetId = instruction->id();
	int targetMotionIndex = -1;
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i] && motions[i]->id() == targetId)
		{
			targetMotionIndex = static_cast<int>(i);
			break;
		}
	}
	if (targetMotionIndex < 0)
	{
		return out;
	}
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath,
		m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const RobotCoordinate::RobotCoordinateFrameSet& framesForFeasible =
		doc->robotCoordinateFramesForInstance(instIdx);
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	for (int mi = 0; mi < targetMotionIndex; ++mi)
	{
		RobotInstruction::Base* motionIns = const_cast<RobotInstruction::Base*>(motions[static_cast<size_t>(mi)]);
		if (!motionIns)
		{
			continue;
		}
		const QVector<double> taughtQ =
			RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*motionIns);
		if (taughtQ.size() == nj
			&& RobotInstructionPlanning::shouldUseTaughtJointCsv(*motionIns, &framesForFeasible))
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = taughtQ[j];
			}
			continue;
		}
		const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*motionIns);
		RobotInstructionPlanning::prepareMotionInstructionForPlanning(
			*motionIns,
			rollingQ,
			doc,
			m_host->osgView(),
			instIdx,
			urdfPath,
			defaultTcpLinkName.toStdString(),
			&framesForFeasible);
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		if (planMotionOnHost(*motionIns, rollingQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, plan, &planErr)
			&& !plan.jointTargetsRad.empty()
			&& plan.jointTargetsRad.size() == static_cast<size_t>(nj))
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		RobotInstructionPlanning::restoreInstructionPose(*motionIns, backup);
	}
	QString fingerprint = QString::fromStdString(instruction->id());
	if (instruction->hasPoseProperty())
	{
		const RobotInstruction::Vec3 p = instruction->pose();
		const RobotInstruction::Vec3 e = instruction->eulerDeg();
		fingerprint += QStringLiteral("|%1,%2,%3|%4,%5,%6")
							.arg(p.x, 0, 'g', 8)
							.arg(p.y, 0, 'g', 8)
							.arg(p.z, 0, 'g', 8)
							.arg(e.x, 0, 'g', 8)
							.arg(e.y, 0, 'g', 8)
							.arg(e.z, 0, 'g', 8);
	}
	fingerprint += QStringLiteral("|mi=%1").arg(targetMotionIndex);
	for (int j = 0; j < rollingQ.size(); ++j)
	{
		fingerprint += QLatin1Char(',') + QString::number(rollingQ[j], 'g', 8);
	}
	osg::Matrixd fpBaseWorld;
	fpBaseWorld.makeIdentity();
	if (RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, m_host->osgView(), instIdx, fpBaseWorld, &rollingQ))
	{
		fingerprint += QStringLiteral("|bw=%1,%2,%3")
							.arg(fpBaseWorld(3, 0), 0, 'g', 8)
							.arg(fpBaseWorld(3, 1), 0, 'g', 8)
							.arg(fpBaseWorld(3, 2), 0, 'g', 8);
	}
	if (m_cachedFeasibleAxisInstructionId == QString::fromStdString(instruction->id())
		&& m_cachedFeasibleAxisFingerprint == fingerprint && !m_cachedFeasibleAxisOptions.presetTokens.empty())
	{
		if (outSeedJointRad)
		{
			*outSeedJointRad = m_cachedFeasibleAxisSeedJointRad;
		}
		return m_cachedFeasibleAxisOptions;
	}

	const RobotInstructionPlanning::MotionPoseBackup targetBackup = RobotInstructionPlanning::backupInstructionPose(*instruction);
	RobotInstructionPlanning::prepareMotionInstructionForPlanning(
		*instruction,
		rollingQ,
		doc,
		m_host->osgView(),
		instIdx,
		urdfPath,
		defaultTcpLinkName.toStdString(),
		&doc->robotCoordinateFramesForInstance(instIdx));
	out = m_instructionController.queryFeasibleMotionAxisConfigurationOptions(*instruction);
	RobotInstructionPlanning::restoreInstructionPose(*instruction, targetBackup);

	m_cachedFeasibleAxisInstructionId = QString::fromStdString(instruction->id());
	m_cachedFeasibleAxisFingerprint = fingerprint;
	m_cachedFeasibleAxisOptions = out;
	m_cachedFeasibleAxisSeedJointRad = rollingQ;
	if (outSeedJointRad)
	{
		*outSeedJointRad = rollingQ;
	}
	return out;
}

void RobotSimulationController::onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (instruction)
	{
		m_host->clearBackendObjectSelection(true);
	}
	m_host->refreshInstructionPropertyPanel(instruction);
	if (instruction && instruction->hasMotionAxisConfigurationProperty())
	{
		const auto& ext = instruction->extensionProperties();
		if (instruction->motionAxisConfiguration().preset == "AUTO"
			&& ext.find("context.axisConfigSeeded") == ext.end())
		{
			QVector<double> seedQ;
			const RobotInstruction::FeasibleMotionAxisConfigurationOptions feasible =
				feasibleMotionAxisConfigurationOptionsForInstruction(instruction, &seedQ);
			m_host->applySuggestedAxisPresetFromSeedIfNeeded(instruction, seedQ, feasible);
			instruction->setExtensionProperty("context.axisConfigSeeded", "1");
			m_host->refreshInstructionPropertyPanel(instruction, false);
		}
	}
	const bool tcpDragActive = m_host->simulationCommandPage()
		&& m_host->simulationCommandPage()->tcpDragTeachMode();
	if (!tcpDragActive)
	{
		applyRobotPoseForInstructionPreview(instruction);
	}
	refreshInstructionPoseAxes();
}

void RobotSimulationController::applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_skipInstructionPreviewOnce)
	{
		m_skipInstructionPreviewOnce = false;
		return;
	}
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
	{
		return;
	}
	if (m_programExecutor.isRunning() || !instruction
		|| !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !poseSink || !osg || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (RobotSimulationMath::buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_instructionController.setDhRows(dhRows);
		}
		else
		{
			m_instructionController.clearDhRows();
		}
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const std::string targetId = instruction->id();
	int targetMotionIndex = -1;
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i] && motions[i]->id() == targetId)
		{
			targetMotionIndex = static_cast<int>(i);
			break;
		}
	}
	if (targetMotionIndex < 0)
	{
		return;
	}
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath,
		m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	bool chainReplanned = false;
	for (int mi = 0; mi <= targetMotionIndex; ++mi)
	{
		RobotInstruction::Base* motionIns = const_cast<RobotInstruction::Base*>(motions[static_cast<size_t>(mi)]);
		if (!motionIns)
		{
			continue;
		}
		const QVector<double> taughtQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*motionIns);
		const RobotCoordinate::RobotCoordinateFrameSet& framesForPlan =
			doc->robotCoordinateFramesForInstance(instIdx);
		bool useTaughtCsv = !chainReplanned && taughtQ.size() == nj
			&& RobotInstructionPlanning::shouldUseTaughtJointCsv(*motionIns, &framesForPlan);
		if (useTaughtCsv)
		{
			const double taughtResidual = targetResidualMmForInstruction(
				urdfPath, taughtQ, framesForPlan, defaultTcpLinkName, *motionIns);
			if (taughtResidual < 0.0 || taughtResidual > kTaughtReuseResidualMm)
			{
				useTaughtCsv = false;
			}
		}
		if (mi == targetMotionIndex || (!useTaughtCsv && !taughtQ.isEmpty()))
		{
			const auto& ext = motionIns->extensionProperties();
			const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
			const auto itCtxTool = ext.find("context.activeToolFrameId");
			const std::string motionToolId = (itMotion != ext.end()) ? itMotion->second : std::string();
			const std::string ctxToolId = (itCtxTool != ext.end()) ? itCtxTool->second : std::string();
		}
		if (useTaughtCsv)
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = taughtQ[j];
			}
			continue;
		}
		const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*motionIns);
		RobotInstructionPlanning::prepareMotionInstructionForPlanning(
			*motionIns,
			rollingQ,
			doc,
			osg,
			instIdx,
			urdfPath,
			defaultTcpLinkName.toStdString(),
			&framesForPlan);
		std::string planErr;
		if (!m_instructionController.validate(*motionIns, &planErr))
		{
			RobotInstructionPlanning::restoreInstructionPose(*motionIns, backup);
			if (m_host->runInfoPage() && mi == targetMotionIndex)
			{
				const QString pointTag = QString::fromStdString(
					RobotInstruction::formatMotionPointName(RobotInstruction::motionPointIndex(*motionIns)));
				m_host->appendRunWarning(
					pointTag.isEmpty()
						? QString::fromStdString(planErr)
						: QStringLiteral("%1: %2").arg(pointTag, QString::fromStdString(planErr)));
			}
			return;
		}
		RobotInstruction::PlanResult plan{};
		if (!planMotionOnHost(*motionIns, rollingQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, plan, &planErr))
		{
			RobotInstructionPlanning::restoreInstructionPose(*motionIns, backup);
			if (m_host->runInfoPage() && mi == targetMotionIndex)
			{
				const QString pointTag = QString::fromStdString(
					RobotInstruction::formatMotionPointName(RobotInstruction::motionPointIndex(*motionIns)));
				m_host->appendRunWarning(
					pointTag.isEmpty()
						? QString::fromStdString(planErr)
						: QStringLiteral("%1: %2").arg(pointTag, QString::fromStdString(planErr)));
			}
			return;
		}
		if (!plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(nj))
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		chainReplanned = true;
		RobotInstructionPlanning::restoreInstructionPose(*motionIns, backup);
	}
	const QVector<double> rollingQBeforeClamp = rollingQ;
	const QVector<double> rollingQClamped = clampJointAnglesToInstanceLimits(doc, instIdx, rollingQ);
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	if (m_aggregatedJointAnglesRad.size() != jnamesAll.size())
	{
		m_aggregatedJointAnglesRad = QVector<double>(jnamesAll.size(), 0.0);
	}
	for (int j = 0; j < nj && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
	{
		m_aggregatedJointAnglesRad[jointOffset + j] = rollingQ[j];
	}
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		m_suppressMotionPreviewStartCapture = true;
		const QSignalBlocker blocker(m_host->robotAxisControlPage());
		m_host->robotAxisControlPage()->setJointAnglesRad(rollingQ);
		m_suppressMotionPreviewStartCapture = false;
	}
	(void)RobotSceneKinematics::applyJointAnglesForInstance(
		doc, poseSink, instIdx, rollingQ, m_aggregatedJointAnglesRad);
	refreshRobotCoordinateFrameOverlays(instruction, &rollingQ);
	osg->requestRedraw();
}

namespace
{
/// 用示教关节 URDF 正解 + 文档基座位姿算世界 TCP，不依赖当前场景关节状态
bool instructionTcpWorldMat4FromTaughtJoints(
	IRobotDocumentHost* doc,
	int instIdx,
	const RobotInstruction::Base& ins,
	const QVector<double>& taughtQ,
	osg::Matrixd& outTcpWorld)
{
	if (!doc || instIdx < 0 || taughtQ.isEmpty())
	{
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	QString fallbackFlangeLink;
	{
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			fallbackFlangeLink = revoluteChildLinks.back();
		}
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	BackendMat4 targetInBase = BackendMat4::identity();
	QString tcpLinkName;
	if (!RobotSimulationMath::targetInBaseFromUrdfFlangeFk(
			urdfPath, taughtQ, frames, fallbackFlangeLink, targetInBase, &ins, &tcpLinkName))
	{
		return false;
	}
	const osg::Matrixd tcpLocal = RobotMatrixOsg::matrixFromBackendColMajor(targetInBase);
	osg::Matrixd robotBaseWorld;
	robotBaseWorld.makeIdentity();
	RobotPerLinkKinematicsSlice slice;
	if (doc->robotPerLinkKinematicsForInstance(instIdx, slice))
	{
		robotBaseWorld = slice.robotBasePlacementWorld;
	}
	else if (!RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, nullptr, instIdx, robotBaseWorld))
	{
		return false;
	}
	outTcpWorld = tcpLocal * robotBaseWorld;
	return true;
}

osg::Matrixd tcpLocalFromPoseFields(const RobotInstruction::Base& ins)
{
	engine::RigidTransform T_pose{};
	if (RobotInstruction::readTargetTransformFromInstruction(ins, T_pose))
	{
		return engine::osgMatrixFromRigidTransform(T_pose);
	}
	return osg::Matrixd();
}

/// 基座系 T_base_target：pose/euler → BackendMat4 → OSG（与 capture/IK 同一套刚体矩阵）
bool instructionTcpLocalMatrix(const RobotInstruction::Base& ins, osg::Matrixd& outTcpLocal)
{
	outTcpLocal = tcpLocalFromPoseFields(ins);
	return true;
}

/// 指令点显示：落盘 T_base_tool（含该点冻结工具系），挂轨迹世界层，不随当前关节 FK 移动
bool fillInstructionPoseAxisMount(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	const RobotInstruction::Base& ins,
	bool lineMotion,
	bool reachable,
	RobotOsgUi::InstructionPoseAxis& axis,
	const QVector<double>* jointAnglesRadLocal = nullptr)
{
	engine::RigidTransform T_target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(ins, T_target))
	{
		return false;
	}
	const osg::Matrixd tcpLocal = engine::osgMatrixFromRigidTransform(T_target);

	axis.lineMotion = lineMotion;
	axis.reachable = reachable;
	axis.hasLocalMatrix = false;
	axis.robotBackendId.clear();
	axis.mountTcpOnPatRoot = false;
	axis.urdfTcpAttachLinkName.clear();

	// 指令点世界位姿以 render.tcpWorldMat4 为准（示教时冻结），不随当前机器人姿态变化
	osg::Matrixd T_world = tcpLocal;
	bool positioned = false;
	const auto itWorld = ins.extensionProperties().find("render.tcpWorldMat4");
	if (itWorld != ins.extensionProperties().end() && !itWorld->second.empty())
	{
		osg::Matrixd T_cached;
		if (RobotSimulationMath::decodeMatrix4Csv(itWorld->second, T_cached))
		{
			T_world = T_cached;
			positioned = true;
		}
	}
	if (!positioned && doc && osg && instIdx >= 0 && jointAnglesRadLocal && !jointAnglesRadLocal->isEmpty())
	{
		osg::Matrixd baseWorld;
		baseWorld.makeIdentity();
		if (RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, baseWorld, jointAnglesRadLocal))
		{
			T_world = tcpLocal * baseWorld;
		}
	}
	axis.positionMm = osg::Vec3f(
		static_cast<float>(T_world(3, 0)),
		static_cast<float>(T_world(3, 1)),
		static_cast<float>(T_world(3, 2)));
	axis.eulerDeg = OsgScene::quatToEulerDeg(T_world.getRotate());
	return true;
}
} // namespace

void RobotSimulationController::syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (!instruction || !instruction->hasPoseProperty())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	osg::Matrixd tcpLocal = tcpLocalFromPoseFields(*instruction);
	instruction->setExtensionProperty("render.tcpLocalMat4", RobotSimulationMath::encodeMatrix4Csv(tcpLocal));
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::resolveToolFrameForExtension(
			frames, instruction->extensionProperties()))
	{
		instruction->setExtensionProperty("context.activeToolFrameId", tool->id);
	}
	osg::Matrixd tcpWorld;
	tcpWorld.makeIdentity();
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	QVector<double> taughtQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*instruction);
	if (taughtQ.size() == nj
		&& instructionTcpWorldMat4FromTaughtJoints(doc, instIdx, *instruction, taughtQ, tcpWorld))
	{
		instruction->setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(tcpWorld));
	}
	else
	{
		osg::Matrixd robotBaseWorld;
		robotBaseWorld.makeIdentity();
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
		QVector<double> syncJointQ;
		if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			syncJointQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				syncJointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		if (RobotSimulationMath::robotBaseWorldMatrixForInstance(
				doc, osg, instIdx, robotBaseWorld, syncJointQ.isEmpty() ? nullptr : &syncJointQ))
		{
			instruction->setExtensionProperty(
				"render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(tcpLocal * robotBaseWorld));
		}
	}
}

QHash<QString, bool> RobotSimulationController::computeMotionReachabilityForCurrentProgram()
{
	QHash<QString, bool> reachability;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return reachability;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return reachability;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions =
		RobotInstruction::collectMotionInstructions(program);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return reachability;
	}
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath,
		m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		const RobotInstruction::Base* motionPtr = motions[mi];
		if (!motionPtr)
		{
			continue;
		}
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*ins);
		RobotInstructionPlanning::prepareMotionInstructionForPlanning(
			*ins,
			rollingQ,
			doc,
			m_host->osgView(),
			instIdx,
			urdfPath,
			defaultTcpLinkName.toStdString(),
			&doc->robotCoordinateFramesForInstance(instIdx));
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		const bool ok = planMotionOnHost(*ins, rollingQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, plan, &planErr)
			&& plan.ok;
		reachability.insert(QString::fromStdString(ins->id()), ok);
		RobotInstructionPlanning::restoreInstructionPose(*ins, backup);
		if (ok && !plan.jointTargetsRad.empty()
			&& plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
	}
	return reachability;
}

void RobotSimulationController::refreshInstructionPoseAxes()
{
	static bool s_matrixConventionSelfTestDone = false;
	if (!s_matrixConventionSelfTestDone)
	{
		s_matrixConventionSelfTestDone = true;
		std::vector<std::string> matrixTestFailures;
		const bool matrixOk = RobotMatrixOsg::runConventionSelfTest(matrixTestFailures);
		if (m_host->runInfoPage())
		{
			if (matrixOk)
			{
				if (RunLogger::isDiagnosticsEnabled())
				{
					m_host->appendRunInfo(
						QStringLiteral("[Matrix self-test] BackendMat4/OSG convention OK"));
				}
			}
			else
			{
				m_host->appendRunWarning(
					QStringLiteral("[Matrix self-test] failed %1 checks; pose axes may be wrong.")
						.arg(static_cast<int>(matrixTestFailures.size())));
				for (const std::string& msg : matrixTestFailures)
				{
					m_host->appendRunWarning(QString::fromStdString(msg));
				}
			}
		}
	}

	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!osg || !m_host->simulationCommandPage())
	{
		if (osg)
		{
			osg->clearInstructionPoseAxes();
		}
		return;
	}
	const QHash<QString, bool> reachability = computeMotionReachabilityForCurrentProgram();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> insList = m_host->simulationCommandPage()->instructionList();
	std::vector<RobotOsgUi::InstructionPoseAxis> axes;
	axes.reserve(insList.size());
	const int axisInstIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	QVector<double> axisJointQ;
	if (doc && axisInstIdx >= 0)
	{
		const int nj = doc->robotRevoluteJointCountForInstance(axisInstIdx);
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(axisInstIdx);
		if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			axisJointQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				axisJointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
		{
			axisJointQ = m_host->robotAxisControlPage()->jointAnglesRad();
		}
	}
	for (const auto& ins : insList)
	{
		if (!ins || !ins->hasPoseProperty())
		{
			continue;
		}
		const auto itReach = reachability.constFind(QString::fromStdString(ins->id()));
		const bool reachable = (itReach == reachability.constEnd()) ? true : itReach.value();
		RobotOsgUi::InstructionPoseAxis a;
		if (!fillInstructionPoseAxisMount(
				doc,
				osg,
				axisInstIdx,
				*ins,
				ins->type() == RobotInstruction::Type::LINE,
				reachable,
				a,
				nullptr))
		{
			continue;
		}
		axes.push_back(a);
	}
	if (axes.empty())
	{
		osg->clearInstructionPoseAxes();
		return;
	}
	osg->setInstructionPoseAxes(axes);

}

void RobotSimulationController::onSimulationStartTriggered()
{
	if (m_programExecutor.isRunning())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage())
	{
		return;
	}
	if (!doc->hasRobotSimulationContext())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(
				QStringLiteral("Import a robot (URDF) first, then add simulation commands."), QStringLiteral("请先导入机器人(URDF)，再添加仿真指令。")));
		}
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (RobotSimulationMath::buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_instructionController.setDhRows(dhRows);
		}
		else
		{
			m_instructionController.clearDhRows();
			if (m_host->runInfoPage())
			{
				m_host->appendRunInfo(
					m_host->i18n(
						QStringLiteral("DH rows not built: %1").arg(dhErr),
						QStringLiteral("DH rows not built: %1").arg(dhErr)));
			}
		}
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(
				QStringLiteral("No revolute joints in URDF (joints need type=\"revolute\" or \"continuous\" and an axis)."), QStringLiteral("URDF 中无可旋转关节（需 type=“revolute/continuous” 及 axis）。")));
		}
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> instructions =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	if (instructions.empty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Add at least one instruction row."), QStringLiteral("请至少添加一条指令。")));
		}
		return;
	}
	const std::vector<const RobotInstruction::Base*> motions =
		RobotInstruction::collectMotionInstructions(instructions);
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath,
		m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QVector<double> initialAngles(jnamesAll.size(), 0.0);
	if (m_motionPreviewProgramStartJointRad.size() == jnamesAll.size())
	{
		initialAngles = m_motionPreviewProgramStartJointRad;
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		const QVector<double> local = m_host->robotAxisControlPage()->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < initialAngles.size(); ++j)
		{
			initialAngles[jointOffset + j] = local[j];
		}
	}
	m_aggregatedJointAnglesRad = initialAngles;

	std::vector<RobotInstruction::PlanResult> planResults;
	planResults.reserve(motions.size());
	QVector<double> rollingQ(nj, 0.0);
	for (int j = 0; j < nj; ++j)
	{
		rollingQ[j] = initialAngles[jointOffset + j];
	}
	bool chainReplanned = false;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		const RobotInstruction::Base* motionPtr = motions[mi];
		if (!motionPtr)
		{
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(m_host->i18n(
					QStringLiteral("Instruction row is invalid."), QStringLiteral("指令行无效。")));
			}
			return;
		}
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const QVector<double> taughtQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*ins);
		const RobotCoordinate::RobotCoordinateFrameSet& framesForRun =
			doc->robotCoordinateFramesForInstance(instIdx);
		bool useTaughtCsv = !chainReplanned && taughtQ.size() == nj
			&& RobotInstructionPlanning::shouldUseTaughtJointCsv(*ins, &framesForRun);
		if (useTaughtCsv)
		{
			const double taughtResidual = targetResidualMmForInstruction(
				urdfPath, taughtQ, framesForRun, defaultTcpLinkName, *ins);
			if (taughtResidual < 0.0 || taughtResidual > kTaughtReuseResidualMm)
			{
				useTaughtCsv = false;
			}
		}
		const auto& ext = ins->extensionProperties();
		const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
		const auto itCtxTool = ext.find("context.activeToolFrameId");
		const std::string motionToolId = (itMotion != ext.end()) ? itMotion->second : std::string();
		const std::string ctxToolId = (itCtxTool != ext.end()) ? itCtxTool->second : std::string();
		if (useTaughtCsv)
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = taughtQ[j];
			}
			const QVector<double> rollingQBeforeClamp = rollingQ;
			const QVector<double> rollingQClamped = clampJointAnglesToInstanceLimits(doc, instIdx, rollingQ);
			RobotInstruction::PlanResult plan{};
			plan.ok = true;
			plan.plannerName = "taughtJointCsv";
			plan.summary = "Use context.currentJointRadCsv from teach capture";
			plan.durationSec = RobotInstructionPlanning::motionDurationSecFromInstruction(*ins);
			plan.jointTargetsRad.reserve(static_cast<size_t>(nj));
			for (int j = 0; j < nj; ++j)
			{
				plan.jointTargetsRad.push_back(rollingQ[j]);
			}
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				initialAngles[jointOffset + j] = rollingQ[j];
			}
			planResults.push_back(std::move(plan));
			continue;
		}
		const RobotInstructionPlanning::MotionPoseBackup poseBackup = RobotInstructionPlanning::backupInstructionPose(*ins);
		RobotInstructionPlanning::prepareMotionInstructionForPlanning(
			*ins,
			rollingQ,
			doc,
			m_host->osgView(),
			instIdx,
			urdfPath,
			defaultTcpLinkName.toStdString(),
			&framesForRun);
		std::string planErr;
		if (!m_instructionController.validate(*ins, &planErr))
		{
			RobotInstructionPlanning::restoreInstructionPose(*ins, poseBackup);
			if (m_host->runInfoPage())
			{
				const QString msg = !planErr.empty() ? QString::fromStdString(planErr)
													 : m_host->i18n(QStringLiteral("Instruction validation failed."), QStringLiteral("指令校验失败。"));
				m_host->appendRunWarning(msg);
			}
			return;
		}
		RobotInstruction::PlanResult plan{};
		if (!planMotionOnHost(*ins, rollingQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, plan, &planErr))
		{
			RobotInstructionPlanning::restoreInstructionPose(*ins, poseBackup);
			if (m_host->runInfoPage())
			{
				const QString msg = !planErr.empty() ? QString::fromStdString(planErr)
													 : m_host->i18n(QStringLiteral("Instruction planning failed."), QStringLiteral("指令规划失败。"));
				m_host->appendRunWarning(msg);
			}
			return;
		}
		if (!plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		chainReplanned = true;
		const QVector<double> rollingQBeforeClamp = rollingQ;
		const QVector<double> rollingQClamped = clampJointAnglesToInstanceLimits(doc, instIdx, rollingQ);
		RobotInstructionPlanning::restoreInstructionPose(*ins, poseBackup);
		if (plan.durationSec > 1e-6)
		{
			ins->setExtensionProperty(
				"motion.durationSec",
				QString::number(plan.durationSec, 'f', 3).toStdString());
		}
		for (int j = 0; j < rollingQ.size(); ++j)
		{
			initialAngles[jointOffset + j] = rollingQ[j];
		}
		planResults.push_back(std::move(plan));
	}
	if (m_host->simulationCommandPage())
	{
		m_host->simulationCommandPage()->refreshInstructionList();
	}
	QString err;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!poseSink
		|| !m_programExecutor.tryStart(
			doc, poseSink, &m_simulationIoSink, instIdx, instructions, planResults, initialAngles, &err))
	{
		if (m_host->runInfoPage())
		{
			if (err.contains(QLatin1String("Invalid joint index")))
			{
				m_host->appendRunWarning(m_host->i18n(QStringLiteral("Invalid joint index in simulation command."), QStringLiteral("仿真指令关节索引无效。")));
			}
			else if (!err.isEmpty())
			{
				m_host->appendRunWarning(err);
			}
		}
		return;
	}
	if (m_host->robotAxisControlPage())
	{
		m_host->robotAxisControlPage()->setInteractionEnabled(false);
	}
	m_host->simulationCommandPage()->setSimulationRunning(true);
	if (m_simulationDock && m_simulationDock->trajectoryEditPage())
	{
		m_simulationDock->trajectoryEditPage()->setReadOnly(true);
	}
	m_playbackTimer->start();
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(QStringLiteral("Simulation started."), QStringLiteral("仿真已开始。")));
	}
}

void RobotSimulationController::logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad)
{
	if (!m_host->runInfoPage())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || finalJointAnglesRad.isEmpty())
	{
		return;
	}
	const auto insList = m_host->simulationCommandPage()->instructionList();
	std::shared_ptr<RobotInstruction::Base> targetIns;
	for (auto it = insList.rbegin(); it != insList.rend(); ++it)
	{
		if (*it && (*it)->hasPoseProperty())
		{
			targetIns = *it;
			break;
		}
	}
	if (!targetIns)
	{
		return;
	}

	const RobotInstruction::Vec3 targetPose = targetIns->pose();
	QString tcpLinkName;
	{
		const auto& ext = targetIns->extensionProperties();
		const auto itCaptured = ext.find("context.capturedTcpLinkName");
		if (itCaptured != ext.end() && !itCaptured->second.empty())
		{
			tcpLinkName = QString::fromStdString(itCaptured->second);
		}
		if (tcpLinkName.isEmpty())
		{
			const auto itTcp = ext.find("context.tcpLinkName");
			if (itTcp != ext.end() && !itTcp->second.empty())
			{
				tcpLinkName = QString::fromStdString(itTcp->second);
			}
		}
	}

	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
		? m_host->simulationCommandPage()->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, finalJointAnglesRad, linkWorldByName, &fkErr))
	{
		m_host->appendRunWarning(m_host->i18n(QStringLiteral("Forward kinematics failed: %1").arg(fkErr), QStringLiteral("正解失败：%1").arg(fkErr)));
		return;
	}
}

void RobotSimulationController::onRobotSimulationTick()
{
	if (!m_programExecutor.isRunning())
	{
		m_playbackTimer->stop();
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	const RobotInstructionPlaybackTickResult r = m_programExecutor.tick(doc, poseSink);
	m_aggregatedJointAnglesRad = m_programExecutor.jointAnglesRad();
	if (doc && osg)
	{
		refreshRobotCoordinateFrameOverlaysForPlayback();
		osg->requestRedraw();
	}
	switch (r)
	{
	case RobotInstructionPlaybackTickResult::Continue:
		break;
	case RobotInstructionPlaybackTickResult::Finished:
		logPlaybackFrameComparison(m_programExecutor.jointAnglesRad());
		refreshRobotCoordinateFrameOverlaysForPlayback();
		stopRobotSimulation();
		if (m_host->runInfoPage())
		{
			m_host->appendRunInfo(
				m_host->i18n(QStringLiteral("Simulation finished."), QStringLiteral("仿真已结束。")));
		}
		break;
	case RobotInstructionPlaybackTickResult::Aborted:
		stopRobotSimulation();
		break;
	}
}

bool RobotSimulationController::planMotionOnHost(
	RobotInstruction::Base& instruction,
	const QVector<double>& seedJointRad,
	const int instanceIndex,
	const QString& urdfPath,
	const QString& defaultTcpLinkName,
	const QString& sceneRootBackendId,
	RobotInstruction::PlanResult& plan,
	std::string* planErr) const
{
	if (!m_host)
	{
		return false;
	}
	return m_host->planRobotMotionInstruction(instruction, seedJointRad, instanceIndex, urdfPath, defaultTcpLinkName,
		sceneRootBackendId, plan, planErr);
}
