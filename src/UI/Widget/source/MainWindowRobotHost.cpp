#include "MainWindowRobotHost.h"

#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "OsgWidget.h"

#include "../OsgWidgetCore/inc/OsgScene.h"
#include "../RobotWidget/inc/IRobotOsgViewHost.h"
#include "../RobotWidget/inc/RobotOsgUiTypes.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"

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
