#include "MainWindowRobotHost.h"

#include "BackendSceneDocumentFacade.h"
#include "DocumentPage.h"
#include "WidgetDocumentAccess.h"
#include "MainWindow.h"
#include "JobSystem.h"
#include "RunInfoPage.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "OsgWidget.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"
#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "../RobotWidget/inc/IRobotOsgViewHost.h"
#include "../RobotWidget/inc/RobotOsgUiTypes.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"

#include "RobotPlanInstruction.h"
#include "RobotSceneKinematics.h"
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
		if (aggregatedJointAnglesRad.size() != m_page->robotRevoluteJointNames().size())
		{
			aggregatedJointAnglesRad.resize(m_page->robotRevoluteJointNames().size());
		}
		if (!RobotSceneKinematics::applyJointAnglesForInstance(
				m_page, poseSink, instanceIndex, jointAnglesRad, aggregatedJointAnglesRad))
		{
			if (outError)
			{
				*outError = QStringLiteral("applyJointAnglesForInstance failed");
			}
			return false;
		}
		m_page->notifyRobotKinematicsAppliedToScene();
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

class MainWindowRobotHost::OsgViewHost : public IRobotOsgViewHost
{
public:
	explicit OsgViewHost(OsgWidget* osg) : m_osg(osg) {}

	IRobotBackendPoseSink* poseSink() override { return m_osg; }
	void requestRedraw() override { m_osg->requestRedraw(); }

	bool objectSelectionMode() const override { return m_osg->objectSelectionMode(); }
	void setObjectSelectionMode(bool enabled) override { m_osg->setObjectSelectionMode(enabled); }
	void clearBackendObjectSelection() override { m_osg->setSelectionActive(false); }
	void setSelectionActive(bool active) override { m_osg->setSelectionActive(active); }
	void setTransformGizmoFrame(int worldOrLocal) override
	{
		m_osg->setTransformGizmoFrame(worldOrLocal == 0 ? OsgScene::TransformGizmoFrame::World
														: OsgScene::TransformGizmoFrame::Local);
	}

	bool hasBackendObjectBranch(const std::string& backendId) const override
	{
		return m_osg->hasBackendObjectBranch(backendId);
	}
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override
	{
		return m_osg->getBackendRootWorldMatrix(backendId, outWorld);
	}
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const override
	{
		return m_osg->tryGetBackendModelCenterMm(backendId, cx, cy, cz);
	}

	void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes) override
	{
		std::vector<OsgWidget::InstructionPoseAxis> converted;
		converted.reserve(axes.size());
		for (const auto& a : axes)
		{
			OsgWidget::InstructionPoseAxis o;
			o.positionMm = a.positionMm;
			o.eulerDeg = a.eulerDeg;
			o.lineMotion = a.lineMotion;
			o.reachable = a.reachable;
			o.robotBackendId = a.robotBackendId.empty() ? a.backendId : a.robotBackendId;
			o.mountTcpOnPatRoot = a.mountTcpOnPatRoot;
			o.hasLocalMatrix = a.hasLocalMatrix;
			if (a.hasLocalMatrix)
			{
				for (int i = 0; i < 16; ++i)
				{
					o.localMatrix[i] = a.localMatrix[i];
				}
			}
			o.urdfTcpAttachLinkName = a.urdfTcpAttachLinkName;
			converted.push_back(o);
		}
		m_osg->setInstructionPoseAxes(converted);
	}
	void clearInstructionPoseAxes() override { m_osg->clearInstructionPoseAxes(); }
	void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points) override
	{
		std::vector<OsgWidget::RawTrajectoryOverlayVertex> converted;
		converted.reserve(points.size());
		for (const RobotOsgUi::RawTrajectoryOverlayVertex& v : points)
		{
			OsgWidget::RawTrajectoryOverlayVertex o;
			o.positionMm = v.positionMm;
			o.reachable = v.reachable;
			converted.push_back(o);
		}
		m_osg->setRawTrajectoryOverlay(converted);
	}
	void clearRawTrajectoryOverlay() override { m_osg->clearRawTrajectoryOverlay(); }
	void setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames) override
	{
		std::vector<OsgWidget::RawTrajectoryOverlayFrame> converted;
		converted.reserve(frames.size());
		for (const RobotOsgUi::RawTrajectoryOverlayFrame& f : frames)
		{
			OsgWidget::RawTrajectoryOverlayFrame o;
			o.positionMm = f.positionMm;
			o.eulerDeg = f.eulerDeg;
			o.reachable = f.reachable;
			converted.push_back(o);
		}
		m_osg->setRawTrajectoryOverlayFrames(converted);
	}
	void clearRawTrajectoryOverlayFrames() override { m_osg->clearRawTrajectoryOverlayFrames(); }
	void setCameraFollowBackendId(const std::string& backendId) override
	{
		m_osg->setCameraFollowBackendId(backendId);
	}

	void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update) override
	{
		OsgWidget::RobotFrameOverlayUpdate u;
		u.robotRootBackendId = update.robotRootBackendId;
		u.showToolFrames = update.showToolFrames;
		u.showUserFrames = update.showUserFrames;
		for (const auto& te : update.toolFrames)
		{
			OsgWidget::RobotFrameOverlayUpdate::ToolEntry e;
			e.name = te.name;
			e.mountBackendId = te.mountBackendId;
			e.localMatrix = te.localMatrix;
			e.active = te.active;
			u.toolFrames.push_back(e);
		}
		for (const auto& ue : update.userFrames)
		{
			OsgWidget::RobotFrameOverlayUpdate::UserEntry e;
			e.name = ue.name;
			e.mountBackendId = ue.mountBackendId;
			e.localMatrix = ue.localMatrix;
			u.userFrames.push_back(e);
		}
		m_osg->setRobotFrameOverlays(u);
	}
	void clearRobotFrameOverlays(const std::string& robotRootBackendId) override
	{
		m_osg->clearRobotFrameOverlays(robotRootBackendId);
	}

	bool isTcpDragTeachActive() const override { return m_osg->isTcpDragTeachActive(); }
	void endTcpDragTeach() override { m_osg->endTcpDragTeach(); }
	void beginTcpDragTeach(
		const std::string& mountBackendId,
		const engine::RigidTransform& T_base_target,
		float modelDiagonalMm,
		std::function<bool(osg::Matrixd& outRobotBaseWorld)> resolveRobotBaseWorld,
		const osg::Matrixd* toolLocalOnFlange) override
	{
		m_osg->beginTcpDragTeach(mountBackendId, T_base_target, modelDiagonalMm, resolveRobotBaseWorld, toolLocalOnFlange);
	}
	void updateTcpDragTeachFromTarget(
		const engine::RigidTransform& T_base_target,
		bool syncTargetInBase = true) override
	{
		m_osg->updateTcpDragTeachFromTarget(T_base_target, syncTargetInBase);
	}
	void updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange) override
	{
		m_osg->updateTcpDragTeachToolLocalOnFlange(toolLocalOnFlange);
	}

	void setMeshLinePickMode(const bool enabled) override { m_osg->setMeshLinePickMode(enabled); }
	void setMeshFacePickMode(const bool enabled) override { m_osg->setMeshFacePickMode(enabled); }
	bool meshLinePickMode() const override { return m_osg->meshLinePickMode(); }
	bool meshFacePickMode() const override { return m_osg->meshFacePickMode(); }
	void setMeshPickScopeBackendId(const std::string& backendId) override
	{
		m_osg->syncSelectionForBackendId(backendId);
		m_osg->setSelectionActive(true);
	}

private:
	OsgWidget* m_osg = nullptr;
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
	if (OsgWidget* o = m_mw->currentOsgWidget())
	{
		if (!m_osgHost || m_osgHostWidget != o)
		{
			m_osgHost = std::make_unique<OsgViewHost>(o);
			m_osgHostWidget = o;
		}
		return m_osgHost.get();
	}
	m_osgHost.reset();
	m_osgHostWidget = nullptr;
	return nullptr;
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
	OsgWidget* osg = widgetOsgFromPage(page);
	if (page && osg)
	{
		m_mw->runBackendFollowSolveAndSync(*page, *osg);
	}
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
