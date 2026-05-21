#include "MainWindow.h"

#include "MainWindowRobotHost.h"
#include "RobotInstructionProgram.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include <QMessageBox>
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"

SimulationCommandWidget* MainWindow::simulationCommandPage() const
{
	RobotSimulationDockWidget* dock =
		m_robotSimulation ? m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->commandPage() : nullptr;
}

void MainWindow::refreshSimulationJointListFromCurrentDoc()
{
	if (m_robotSimulation)
	{
		m_robotSimulation->refreshSimulationJointListFromCurrentDoc();
	}
}

void MainWindow::syncRobotFrameSettingsFromDocument(const int instanceIndex)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->syncRobotFrameSettingsFromDocument(instanceIndex);
	}
}

void MainWindow::refreshRobotCoordinateFrameOverlays(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->refreshRobotCoordinateFrameOverlays(instruction);
	}
}

void MainWindow::applyRobotPoseForInstructionPreview(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->applyRobotPoseForInstructionPreview(instruction);
	}
}

void MainWindow::syncInstructionRenderMatricesFromPose(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->syncInstructionRenderMatricesFromPose(instruction);
	}
}

void MainWindow::refreshInstructionPoseAxes()
{
	if (m_robotSimulation)
	{
		m_robotSimulation->refreshInstructionPoseAxes();
	}
}

void MainWindow::stopRobotSimulation()
{
	if (m_robotSimulation)
	{
		m_robotSimulation->stopRobotSimulation();
	}
}

void MainWindow::syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->syncRobotKinematicsAfterPoseEdit(data);
	}
}

bool MainWindow::tryCaptureCurrentRobotTcpPose(
	RobotInstruction::Vec3& outPoseMm,
	RobotInstruction::Vec3& outEulerDeg,
	osg::Matrixd* outTcpLocalMat,
	osg::Matrixd* outTcpRenderWorldMat,
	QString* outTcpLinkName,
	QString* errMsg) const
{
	return m_robotSimulation
		? m_robotSimulation->tryCaptureCurrentRobotTcpPose(
			  outPoseMm, outEulerDeg, outTcpLocalMat, outTcpRenderWorldMat, outTcpLinkName, errMsg)
		: false;
}

int MainWindow::currentSimulationRobotInstanceIndex() const
{
	if (SimulationCommandWidget* cmd = simulationCommandPage())
	{
		return cmd->currentRobotInstanceIndex();
	}
	return 0;
}

void MainWindow::onUrdfImportRequested(const QString& urdfPath)
{
	if (urdfPath.isEmpty())
	{
		return;
	}
	if (!currentOsgWidget())
	{
		QMessageBox::warning(this, QStringLiteral("URDF"), QStringLiteral("No active 3D view."));
		return;
	}
	if (m_robotSimulation)
	{
		m_robotSimulation->onUrdfImportRequested(urdfPath);
	}
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions
MainWindow::feasibleMotionAxisConfigurationOptionsForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	QVector<double>* outSeedJointRad)
{
	if (m_robotSimulation)
	{
		return m_robotSimulation->feasibleMotionAxisConfigurationOptionsForInstruction(
			instruction, outSeedJointRad);
	}
	return {};
}

void MainWindow::onSimulationStartTriggered()
{
	if (m_robotSimulation) m_robotSimulation->onSimulationStartTriggered();
}
void MainWindow::onSimulationRunRequested()
{
	if (m_robotSimulation) m_robotSimulation->onSimulationRunRequested();
}
void MainWindow::onSimulationStopRequested()
{
	if (m_robotSimulation) m_robotSimulation->onSimulationStopRequested();
}
void MainWindow::onSimulationAddInstructionRequested(RobotInstruction::Type type)
{
	if (m_robotSimulation) m_robotSimulation->onSimulationAddInstructionRequested(type);
}
void MainWindow::onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation) m_robotSimulation->onSimulationInstructionSelectionChanged(instruction);
}
void MainWindow::onRobotSimulationTick()
{
	if (m_robotSimulation) m_robotSimulation->onRobotSimulationTick();
}
void MainWindow::onSimulationExportRequested()
{
	if (m_robotSimulation) m_robotSimulation->onSimulationExportRequested();
}
void MainWindow::onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId)
{
	if (m_robotSimulation) m_robotSimulation->onSimulationRobotSelectionChanged(instanceIndex, sceneBackendId);
}
void MainWindow::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_robotSimulation) m_robotSimulation->onRobotAxisJointAnglesChanged(jointAnglesRad);
}
void MainWindow::onSimulationTcpDragTeachModeChanged(bool enabled)
{
	if (m_robotSimulation) m_robotSimulation->onSimulationTcpDragTeachModeChanged(enabled);
}
void MainWindow::onTcpDragTeachPoseChanged(
	double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->onTcpDragTeachPoseChanged(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
	}
}
void MainWindow::onTcpDragTeachEnded()
{
	if (m_robotSimulation) m_robotSimulation->onTcpDragTeachEnded();
}
void MainWindow::onRobotCoordinateFramesChanged()
{
	if (m_robotSimulation) m_robotSimulation->onRobotCoordinateFramesChanged();
}
void MainWindow::onCaptureToolFrameFromTcp()
{
	if (m_robotSimulation) m_robotSimulation->onCaptureToolFrameFromTcp();
}
void MainWindow::onCaptureUserFrameFromTcp()
{
	if (m_robotSimulation) m_robotSimulation->onCaptureUserFrameFromTcp();
}
void MainWindow::onResetToolFrame()
{
	if (m_robotSimulation) m_robotSimulation->onResetToolFrame();
}
