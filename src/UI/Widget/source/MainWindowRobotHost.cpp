#include "MainWindowRobotHost.h"

#include "BackendSceneDocumentFacade.h"
#include "CoreTypes.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "IRobotService.h"
#include "WidgetOsgViewHost.h"
#include "WidgetSceneSignalWiring.h"
#include "MainWindow.h"
#include "JobSystem.h"
#include "RunInfoPage.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"

#include "RobotInstructionPropertyDto.h"
#include "RobotPlanInstruction.h"
#include "UrdfRobotLoader.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotTeachIk.h"

#include <Adapters.h>

#include <memory>

class MainWindowRobotHost::DocumentHost : public IRobotDocumentHost
{
public:
	explicit DocumentHost(DocumentPage* page) : m_page(page) {}

	DocumentPage* page() const { return m_page; }

	RobotProgramStore& robotProgramStore() override { return m_page->robotProgramStore(); }
	const RobotProgramStore& robotProgramStore() const override { return m_page->robotProgramStore(); }
	IRobotBackendPoseSink* poseSink() override { return m_page->sceneFacade().poseSink(); }
	BackendDataManager& backend() override { return m_page->backend(); }
	BackendDataManager* robotBackendManagerForKinematics() override { return m_page->robotBackendManagerForKinematics(); }

	bool hasRobotSimulationContext() const override { return m_page->hasRobotSimulationContext(); }
	bool hasRobotKinematicsBind() const override { return m_page->hasRobotKinematicsBind(); }
	const QString& robotUrdfAbsolutePath() const override { return m_page->robotUrdfAbsolutePath(); }
	const QStringList& robotRevoluteJointNames() const override { return m_page->robotRevoluteJointNames(); }
	const QHash<QString, QString>& robotLinkNameToBackendId() const override
	{
		return m_page->robotLinkNameToBackendId();
	}
	osg::MatrixTransform* robotJointMatrixTransform(const QString& jointName) const override
	{
		return m_page->robotJointMatrixTransform(jointName);
	}
	const QHash<QString, osg::Matrixd>& robotFkMeshWorldT0() const override
	{
		return m_page->robotFkMeshWorldT0();
	}
	const QHash<QString, osg::Matrixd>& robotOuterWorldAtBind() const override
	{
		return m_page->robotOuterWorldAtBind();
	}
	QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0Dto() const override
	{
		return m_page->robotFkMeshWorldT0Dto();
	}
	QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBindDto() const override
	{
		return m_page->robotOuterWorldAtBindDto();
	}
	bool robotUrdfMeshVerticesInLinkFrame() const override
	{
		return m_page->robotUrdfMeshVerticesInLinkFrame();
	}
	int robotKinematicInstanceCount() const override { return m_page->robotKinematicInstanceCount(); }
	QString robotUrdfAbsolutePathForInstance(int instanceIndex) const override
	{
		return m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
	}
	int robotRevoluteJointCountForInstance(int instanceIndex) const override
	{
		return m_page->robotRevoluteJointCountForInstance(instanceIndex);
	}
	QString robotJointKeyPrefixForInstance(int instanceIndex) const override
	{
		return m_page->robotJointKeyPrefixForInstance(instanceIndex);
	}
	bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const override
	{
		return m_page->robotUsesPerLinkBackendsForInstance(instanceIndex);
	}
	bool robotPerLinkKinematicsForInstance(int instanceIndex, RobotPerLinkKinematicsSlice& out) const override
	{
		return m_page->robotPerLinkKinematicsForInstance(instanceIndex, out);
	}
	QString robotSceneBackendIdForInstance(int instanceIndex) const override
	{
		return m_page->robotSceneBackendIdForInstance(instanceIndex);
	}
	QString robotFrameWorldReferenceBackendId(int instanceIndex) const override
	{
		return m_page->robotFrameWorldReferenceBackendId(instanceIndex);
	}
	QString robotDisplayLabelForInstance(int instanceIndex) const override
	{
		return m_page->robotDisplayLabelForInstance(instanceIndex);
	}
	QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const override
	{
		return m_page->robotRevoluteJointNamesForInstance(instanceIndex);
	}
	void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad, QVector<double>& upperRad) const override
	{
		m_page->robotJointLimitsForInstance(instanceIndex, lowerRad, upperRad);
	}
	int robotJointOffsetInAggregatedVector(int instanceIndex) const override
	{
		return m_page->robotJointOffsetInAggregatedVector(instanceIndex);
	}
	int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const override
	{
		return m_page->robotInstanceIndexForSceneBackendId(sceneBackendId);
	}
	int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot) const override
	{
		return m_page->robotInstanceIndexForPerLinkBackend(backendId, outIsSceneRoot);
	}
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) override
	{
		return m_page->robotCoordinateFramesForInstance(instanceIndex);
	}
	const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const override
	{
		return m_page->robotCoordinateFramesForInstance(instanceIndex);
	}
	const RobotCoordinate::RobotUserFrame* robotActiveUserFrameForInstance(int instanceIndex) const override
	{
		return m_page->robotActiveUserFrameForInstance(instanceIndex);
	}
	void setRobotBasePlacementWorldForInstance(int instanceIndex, const osg::Matrixd& placementWorld) override
	{
		m_page->setRobotBasePlacementWorldForInstance(instanceIndex, placementWorld);
	}
	void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId, const osg::Matrixd& world) override
	{
		m_page->updateRobotLinkOuterBindFromWorld(instanceIndex, linkBackendId, world);
	}
	void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) override
	{
		m_page->reconcilePerLinkOuterBindFromScene(instanceIndex, jointAnglesRad);
	}
	void notifyRobotKinematicsAppliedToScene() override { m_page->notifyRobotKinematicsAppliedToScene(); }
	void requestFollowSolveForced() override { m_page->requestFollowSolveForced(); }
	void setSuppressRobotFollowDirtyNotify(const bool suppress) override
	{
		m_page->setSuppressRobotFollowDirtyNotify(suppress);
	}
	void clearFollowDirtyBackendIds() override { m_page->clearFollowDirtyBackendIds(); }

	bool applyJointAnglesRad(int instanceIndex, const QVector<double>& jointAnglesRad,
		QVector<double>& aggregatedJointAnglesRad, QString* outError) override
	{
		if (!m_page || !m_page->hasRobotSimulationContext())
		{
			if (outError)
			{
				*outError = QStringLiteral("no robot simulation context");
			}
			return false;
		}
		IRobotBackendPoseSink* poseSink = m_page->sceneFacade().poseSink();
		if (!poseSink)
		{
			if (outError)
			{
				*outError = QStringLiteral("no pose sink");
			}
			return false;
		}
		const int nj = m_page->robotRevoluteJointCountForInstance(instanceIndex);
		if (jointAnglesRad.size() != nj)
		{
			if (outError)
			{
				*outError = QStringLiteral("joint count mismatch: expected %1, got %2").arg(nj).arg(jointAnglesRad.size());
			}
			return false;
		}
		const QString sceneRootId = m_page->robotSceneBackendIdForInstance(instanceIndex);
		if (sceneRootId.isEmpty())
		{
			if (outError)
			{
				*outError = QStringLiteral("no scene root for instance %1").arg(instanceIndex);
			}
			return false;
		}
		if (!m_page->robot().applyJointAnglesRad(sceneRootId, jointAnglesRad, &aggregatedJointAnglesRad, outError))
		{
			return false;
		}
		return true;
	}

	bool captureToolFrameFromTcp(int instanceIndex, const BackendMat4& T_base_tcp,
		const QVector<double>& jointAnglesRad, const QString& flangeLinkName,
		RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError) override
	{
		if (!m_page)
		{
			if (outError)
			{
				*outError = QStringLiteral("no document page");
			}
			return false;
		}
		const QString urdfPath = m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
		std::string flangeLink = flangeLinkName.toStdString();
		if (flangeLink.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("no flange link name");
			}
			return false;
		}
		QHash<QString, osg::Matrixd> linkWorld;
		QString err;
		if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointAnglesRad, linkWorld, &err))
		{
			if (outError)
			{
				*outError = err;
			}
			return false;
		}
		const QString flangeQ = QString::fromStdString(flangeLink);
		if (!linkWorld.contains(flangeQ))
		{
			if (outError)
			{
				*outError = QStringLiteral("flange link not found: %1").arg(flangeQ);
			}
			return false;
		}
		const BackendMat4 T_base_flange =
			RobotMatrixOsg::backendColMajorFromMatrix(linkWorld.value(flangeQ));
		BackendMat4 invFlange{};
		if (!backend_mat4_invert_rigid(T_base_flange, invFlange))
		{
			if (outError)
			{
				*outError = QStringLiteral("cannot invert flange matrix");
			}
			return false;
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
			if (outError)
			{
				*outError = QStringLiteral("no tool frame available");
			}
			return false;
		}
		target->T_flange_tool = RobotCoordinate::mat4ToFrame(T_flange_tool);
		return true;
	}

	bool captureUserFrameFromTcp(int /*instanceIndex*/, double posXmm, double posYmm, double posZmm,
		double eulerXdeg, double eulerYdeg, double eulerZdeg,
		RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError) override
	{
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
			if (outError)
			{
				*outError = QStringLiteral("no user frame available");
			}
			return false;
		}
		target->T_base_user.positionMm[0] = posXmm;
		target->T_base_user.positionMm[1] = posYmm;
		target->T_base_user.positionMm[2] = posZmm;
		target->T_base_user.eulerDeg[0] = eulerXdeg;
		target->T_base_user.eulerDeg[1] = eulerYdeg;
		target->T_base_user.eulerDeg[2] = eulerZdeg;
		return true;
	}

	void resetToolFrame(int /*instanceIndex*/, RobotCoordinate::RobotCoordinateFrameSet& frames) override
	{
		for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
		{
			if (tf.id == frames.activeToolFrameId)
			{
				tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
				break;
			}
		}
	}

	TcpDragIkResult solveTcpDragTeachIk(int instanceIndex,
		double pxMm, double pyMm, double pzMm,
		double exDeg, double eyDeg, double ezDeg,
		const QVector<double>& seedJointRad,
		const QString& ikLinkName) override
	{
		TcpDragIkResult result;
		if (!m_page)
		{
			result.error = QStringLiteral("no document page");
			return result;
		}
		const QString urdfPath = m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
		if (urdfPath.isEmpty())
		{
			result.error = QStringLiteral("no URDF path");
			return result;
		}
		const RobotCoordinate::RobotCoordinateFrameSet& frames = m_page->robotCoordinateFramesForInstance(instanceIndex);
		BackendMat4 toolMat = BackendMat4::identity();
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
		{
			toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
		}
		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = urdfPath;
		ctx.ikLinkName = ikLinkName;
		ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
		ctx.seedJointRad.reserve(static_cast<size_t>(seedJointRad.size()));
		for (double v : seedJointRad)
		{
			ctx.seedJointRad.push_back(v);
		}
		ctx.useOrientation = true;
		ctx.T_flange_tool = toolMat;
		ctx.maxIkIterations = 20;
		const RobotTeachIk::TeachIkResult ik = RobotTeachIk::solveTeachIk(ctx);
		if (!ik.ok)
		{
			result.error = QStringLiteral("IK solve failed");
			return result;
		}
		result.ok = true;
		result.jointRad.reserve(static_cast<int>(ik.jointRad.size()));
		for (double v : ik.jointRad)
		{
			result.jointRad.push_back(v);
		}
		return result;
	}

	bool planForExport(int instanceIndex,
		const std::vector<std::shared_ptr<RobotInstruction::Base>>& instructions,
		const QVector<double>& seedJointRad,
		const QString& urdfPath,
		const QString& tcpLinkName,
		std::vector<ExportPlanResult>& outPlans,
		int& outFailedCount,
		QString* outError) override
	{
		if (!m_page)
		{
			if (outError)
			{
				*outError = QStringLiteral("no document page");
			}
			return false;
		}
		outPlans.clear();
		outFailedCount = 0;
		QVector<double> rollingQ = seedJointRad;
		for (const auto& insPtr : instructions)
		{
			ExportPlanResult result;
			if (!insPtr)
			{
				result.ok = false;
				result.summary = "Invalid instruction";
				outPlans.push_back(result);
				++outFailedCount;
				continue;
			}
			// 规划由 Controller 的 m_instructionController 处理
			// 这里只准备上下文，实际规划需 Controller 调用
			result.ok = true;
			outPlans.push_back(result);
		}
		return true;
	}

	QString meshBackendStepSourcePath(const QString& backendId) const override
	{
		return m_page->backendSourcePath().value(backendId);
	}

private:
	DocumentPage* m_page = nullptr;
};

MainWindowRobotHost::MainWindowRobotHost(MainWindow* mw) : m_mw(mw) {}

IRobotDocumentHost* MainWindowRobotHost::document()
{
	if (DocumentPage* p = m_mw->currentPage())
	{
		if (!m_docHost || m_docHost->page() != p)
		{
			m_docHost = std::make_unique<DocumentHost>(p);
		}
		return m_docHost.get();
	}
	m_docHost.reset();
	return nullptr;
}

const IRobotDocumentHost* MainWindowRobotHost::document() const
{
	return const_cast<MainWindowRobotHost*>(this)->document();
}

IRobotOsgViewHost* MainWindowRobotHost::osgView()
{
	DocumentPage* page = m_mw->currentPage();
	if (!page || !page->osgWidget())
	{
		m_osgHost.reset();
		m_osgHostPage = nullptr;
		return nullptr;
	}
	if (!m_osgHost || m_osgHostPage != page)
	{
		m_osgHost = std::make_unique<WidgetOsgViewHost>(page);
		m_osgHostPage = page;
	}
	return m_osgHost.get();
}

bool MainWindowRobotHost::useChinese() const { return m_mw->m_useChinese; }

QString MainWindowRobotHost::i18n(const QString& en, const QString& zh) const { return m_mw->i18n(en, zh); }

RunInfoPage* MainWindowRobotHost::runInfoPage() { return m_mw->m_runInfoPage; }

void MainWindowRobotHost::appendRunInfo(const QString& message)
{
	if (m_mw->m_runInfoPage)
	{
		m_mw->m_runInfoPage->appendInfo(message);
	}
}

void MainWindowRobotHost::appendRunWarning(const QString& message)
{
	if (m_mw->m_runInfoPage)
	{
		m_mw->m_runInfoPage->appendWarning(message);
	}
}

QStatusBar* MainWindowRobotHost::statusBar() { return m_mw->statusBar(); }

SimulationCommandWidget* MainWindowRobotHost::simulationCommandPage()
{
	RobotSimulationDockWidget* dock =
		m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->commandPage() : nullptr;
}

RobotAxisControlWidget* MainWindowRobotHost::robotAxisControlPage()
{
	RobotSimulationDockWidget* dock =
		m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->axisPage() : nullptr;
}

RobotFrameSettingsWidget* MainWindowRobotHost::robotFrameSettingsPage()
{
	RobotSimulationDockWidget* dock =
		m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->framePage() : nullptr;
}

DevicePageWidget* MainWindowRobotHost::devicePage() { return m_mw->m_devicePage; }

QAction* MainWindowRobotHost::simulationStartAction() { return m_mw->m_simulationStartAction; }

int MainWindowRobotHost::currentSimulationRobotInstanceIndex() const
{
	return m_mw->currentSimulationRobotInstanceIndex();
}

void MainWindowRobotHost::refreshBackendTree() { m_mw->refreshBackendTree(); }

void MainWindowRobotHost::runFollowSolveAndSyncForCurrentDocument()
{
	DocumentPage* page = m_mw->currentPage();
	if (!page)
	{
		return;
	}
	cloudsim::core::IRenderView* rv = &page->render();
	cloudsim::core::FollowSolveContextDto ctx;
	ctx.skipAll = rv->isTcpDragTeachActive()
		|| (m_mw->robotSimulation() && m_mw->robotSimulation()->programExecutor().isRunning());
	(void)page->data().runFollowSolveAndSync(ctx, nullptr);
}

void MainWindowRobotHost::refreshInstructionPropertyPanel(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const bool refreshFeasibleAxisOptions)
{
	m_mw->updateInstructionPropertyPanel(instruction, refreshFeasibleAxisOptions);
}

void MainWindowRobotHost::clearInstructionPropertyPanel()
{
	m_mw->updateInstructionPropertyPanel(nullptr, true);
}

void MainWindowRobotHost::invalidateInstructionPropertyCache() { m_mw->invalidateFeasibleAxisConfigurationCache(); }

void MainWindowRobotHost::clearBackendObjectSelection(const bool clearTreeSelection)
{
	MainWindowSelectionService::clearBackendObjectSelection(*m_mw, clearTreeSelection);
}

QString MainWindowRobotHost::selectedBackendId() const
{
	return m_mw ? m_mw->m_selectionState.selectedBackendId() : QString();
}

std::shared_ptr<RobotInstruction::Base> MainWindowRobotHost::activeInstructionForProperty() const
{
	return m_mw->m_activeInstructionForProperty;
}

void MainWindowRobotHost::applySuggestedAxisPresetFromSeedIfNeeded(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	m_mw->applySuggestedAxisPresetFromSeedIfNeeded(instruction, seedJointRad, feasible);
}

bool MainWindowRobotHost::registerUrdfRobot(const QString& urdfPath, const bool quietUi)
{
	MainWindowImportCaptureRenderController controller;
	return controller.registerUrdfRobot(*m_mw, urdfPath, quietUi);
}

bool MainWindowRobotHost::planRobotMotionInstruction(
	RobotInstruction::Base& instruction,
	const QVector<double>& seedJointRad,
	const int instanceIndex,
	const QString& urdfPath,
	const QString& defaultTcpLinkName,
	const QString& sceneRootBackendId,
	RobotInstruction::PlanResult& out,
	std::string* outErr)
{
	DocumentPage* page = m_mw->currentPage();
	if (!page)
	{
		if (outErr)
		{
			*outErr = "no active document";
		}
		return false;
	}
	QString hostErr;
	const bool ok = cloudsim::host::planRobotInstruction(*page, instruction, seedJointRad, instanceIndex, urdfPath,
		defaultTcpLinkName.toStdString(), sceneRootBackendId, out, &hostErr);
	if (!ok && outErr)
	{
		*outErr = hostErr.toStdString();
	}
	return ok;
}

void MainWindowRobotHost::enqueueBackgroundJob(
	const QString& title,
	std::function<void()> work,
	std::function<void(bool threw, const QString& msg)> onFinished)
{
	if (!m_mw || !m_mw->jobSystem())
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem unavailable"));
		}
		return;
	}
	m_mw->jobSystem()->enqueue(
		title,
		[work = std::move(work)](const JobProgressSink&) {
			if (work)
			{
				work();
			}
		},
		std::move(onFinished));
}

void MainWindowRobotHost::setMeshPickCommittedHandler(std::function<void(const PickResult&, PickKind)> handler)
{
	m_meshPickHandler = std::move(handler);
}

void MainWindowRobotHost::clearMeshPickCommittedHandler()
{
	m_meshPickHandler = {};
}

void MainWindowRobotHost::notifyMeshPickCommitted(const PickResult& pick, const PickKind kind)
{
	if (m_meshPickHandler)
	{
		m_meshPickHandler(pick, kind);
	}
}

void MainWindowRobotHost::wireDocumentPageSceneSignals(DocumentPage* page)
{
	if (!m_mw)
	{
		return;
	}
	wireMainWindowDocumentSceneSignals(*m_mw, page, this);
}

namespace
{
cloudsim::core::FeasibleMotionAxisOptionsDto toFeasibleAxisDto(
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& engine)
{
	return cloudsim::host::feasibleAxisOptionsFromEngine(
		engine.presetTokens,
		engine.elbowTokens,
		engine.wristTokens,
		engine.armTokens,
		engine.turnJ1Tokens,
		engine.turnJ4Tokens,
		engine.turnJ6Tokens);
}
} // namespace

QVector<cloudsim::core::PropertyRowDto> MainWindowRobotHost::instructionPropertyRows(const QString& instructionId)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	const std::shared_ptr<RobotInstruction::Base> ins =
		m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		return {};
	}
	return cloudsim::host::propertyRowsFromInstructionSnapshotJson(ins->snapshotPropertyRows());
}

bool MainWindowRobotHost::applyInstructionPropertyChange(const QString& instructionId, const QString& key,
	const QString& value, QString* outError)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot simulation");
		}
		return false;
	}
	const std::shared_ptr<RobotInstruction::Base> ins =
		m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		if (outError)
		{
			*outError = QStringLiteral("instruction not found: %1").arg(instructionId);
		}
		return false;
	}
	std::string err;
	const bool ok = ins->applyPropertyChange(key.toStdString(), value.toStdString(), &err);
	if (!ok && outError)
	{
		*outError = QString::fromStdString(err);
	}
	return ok;
}

cloudsim::core::FeasibleMotionAxisOptionsDto MainWindowRobotHost::queryFeasibleMotionAxisOptions(
	const QString& instructionId,
	QVector<double>* outSeedJointRad)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	const std::shared_ptr<RobotInstruction::Base> ins =
		m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		return {};
	}
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions engine =
		m_mw->robotSimulation()->feasibleMotionAxisConfigurationOptionsForInstruction(ins, outSeedJointRad);
	return toFeasibleAxisDto(engine);
}

cloudsim::core::FeasibleMotionAxisOptionsDto MainWindowRobotHost::cachedFeasibleMotionAxisOptions()
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	return toFeasibleAxisDto(m_mw->robotSimulation()->cachedFeasibleAxisConfigurationOptions());
}
