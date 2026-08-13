/// @file MainWindowRobotStubs.cpp
/// @brief MainWindowRobotStubs 实现

#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "DocumentPage.h"
#include "IRobotService.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "RobotInstructionProgram.h"
#include "WidgetRenderAccess.h"

#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>

SimulationCommandWidget* MainWindow::simulationCommandPage() const
{
	RobotSimulationDockWidget* dock = m_robotSimulation ? m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->commandPage() : nullptr;
}

bool MainWindow::resolveTrajectoryWorkpieceForAi(QString* outBackendId, QString* outStepPath)
{
	if (!m_robotSimulation)
	{
		return false;
	}
	QString backendId;
	QString stepPath;
	if (!m_robotSimulation->resolveTrajectoryWorkpiece(backendId, stepPath))
	{
		return false;
	}
	if (outBackendId)
	{
		*outBackendId = backendId;
	}
	if (outStepPath)
	{
		*outStepPath = stepPath;
	}
	return true;
}

bool MainWindow::showAiFeatureCandidatePreviewForAi(const std::string& previewJsonUtf8, QString* outError)
{
	if (!m_robotSimulation)
	{
		if (outError)
		{
			*outError = QStringLiteral("机器人仿真未就绪");
		}
		return false;
	}
	return m_robotSimulation->showAiFeatureCandidatePreview(QByteArray::fromStdString(previewJsonUtf8), outError);
}

void MainWindow::clearAiFeatureCandidatePreviewForAi()
{
	if (m_robotSimulation)
	{
		m_robotSimulation->clearAiFeatureCandidatePreview();
	}
}

bool MainWindow::commitAiTrajectoryFeaturesForAi(const std::string& featurePlanJsonUtf8, QString* outSummary,
												 QString* outError)
{
	if (!m_robotSimulation)
	{
		if (outError)
		{
			*outError = QStringLiteral("机器人仿真未就绪");
		}
		return false;
	}
	return m_robotSimulation->commitAiTrajectoryFeatures(QByteArray::fromStdString(featurePlanJsonUtf8), outSummary,
														 outError);
}

int MainWindow::proposeAndConfirmTrajectoryPlanForAi(const std::string& planInUtf8, std::string* planOutUtf8,
													 QString* outError, const bool showRetry)
{
	if (planOutUtf8)
		planOutUtf8->clear();
	if (!m_robotSimulation)
	{
		if (outError)
			*outError = QStringLiteral("机器人仿真未就绪");
		return 0;
	}
	QByteArray out;
	const int code = m_robotSimulation->proposeAndConfirmTrajectoryPlan(QByteArray::fromStdString(planInUtf8), out,
																		outError, showRetry);
	if (planOutUtf8 && code == 1)
		*planOutUtf8 = out.toStdString();
	return code;
}

bool MainWindow::loadBoundTrajectoryPlanForAi(std::string* planOutUtf8, QString* outError)
{
	if (planOutUtf8)
		planOutUtf8->clear();
	if (!m_robotSimulation)
	{
		if (outError)
			*outError = QStringLiteral("机器人仿真未就绪");
		return false;
	}
	QByteArray out;
	if (!m_robotSimulation->loadBoundTrajectoryPlanForAi(out, outError))
		return false;
	if (planOutUtf8)
		*planOutUtf8 = out.toStdString();
	return true;
}

bool MainWindow::reviseAiTrajectoryPlanForAi(const std::string& planJsonUtf8, QString* outSummary, QString* outError)
{
	if (!m_robotSimulation)
	{
		if (outError)
			*outError = QStringLiteral("机器人仿真未就绪");
		return false;
	}
	return m_robotSimulation->reviseAiTrajectoryPlan(QByteArray::fromStdString(planJsonUtf8), outSummary, outError);
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

void MainWindow::refreshRobotCoordinateFrameOverlays(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->refreshRobotCoordinateFrameOverlays(instruction);
	}
}

void MainWindow::applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->applyRobotPoseForInstructionPreview(instruction);
	}
}

void MainWindow::syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction)
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

void MainWindow::syncRobotKinematicsAfterPoseEdit(const QString& backendId)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->syncRobotKinematicsAfterPoseEdit(backendId);
	}
}

bool MainWindow::tryCaptureCurrentRobotTcpPose(RobotInstruction::Vec3& outPoseMm, RobotInstruction::Vec3& outEulerDeg,
											   osg::Matrixd* outTcpLocalMat, osg::Matrixd* outTcpRenderWorldMat,
											   QString* outTcpLinkName, QString* errMsg) const
{
	return m_robotSimulation ? m_robotSimulation->tryCaptureCurrentRobotTcpPose(
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

void MainWindow::focusBackendInTree(const std::string& backendId)
{
	focusBackendInTreeLocal(QString::fromStdString(backendId));
}

QString MainWindow::selectedBackendId() const
{
	return m_selectionState.selectedBackendId();
}

JobSystem* MainWindow::jobSystem()
{
	return m_jobSystem;
}

QMenuBar* MainWindow::menuBar()
{
	return QMainWindow::menuBar();
}

QStatusBar* MainWindow::statusBar()
{
	return QMainWindow::statusBar();
}

void MainWindow::onUrdfImportRequested(const QString& urdfPath)
{
	if (urdfPath.isEmpty())
	{
		return;
	}
	if (!renderWidgetFromPage(currentPage()))
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
	const std::shared_ptr<RobotInstruction::Base>& instruction, QVector<double>* outSeedJointRad)
{
	if (!instruction)
	{
		return {};
	}
	DocumentPage* page = currentPage();
	if (page)
	{
		const cloudsim::core::FeasibleMotionAxisOptionsDto dto =
			page->robot().queryFeasibleMotionAxisOptions(QString::fromStdString(instruction->id()), outSeedJointRad);
		RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
		auto fill = [](std::vector<std::string>& dest, const QStringList& src)
		{
			dest.reserve(static_cast<size_t>(src.size()));
			for (const QString& t : src)
			{
				dest.push_back(t.toStdString());
			}
		};
		fill(out.presetTokens, dto.presetTokens);
		fill(out.elbowTokens, dto.elbowTokens);
		fill(out.wristTokens, dto.wristTokens);
		fill(out.armTokens, dto.armTokens);
		fill(out.turnJ1Tokens, dto.turnJ1Tokens);
		fill(out.turnJ4Tokens, dto.turnJ4Tokens);
		fill(out.turnJ6Tokens, dto.turnJ6Tokens);
		return out;
	}
	if (m_robotSimulation)
	{
		return m_robotSimulation->feasibleMotionAxisConfigurationOptionsForInstruction(instruction, outSeedJointRad);
	}
	return {};
}

void MainWindow::onSimulationStartTriggered()
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationStartTriggered();
}
void MainWindow::onSimulationRunRequested()
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationRunRequested();
}
void MainWindow::onSimulationStopRequested()
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationStopRequested();
}
void MainWindow::onSimulationAddInstructionRequested(RobotInstruction::Type type)
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationAddInstructionRequested(type);
}
void MainWindow::onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationInstructionSelectionChanged(instruction);
}
void MainWindow::onRobotSimulationTick()
{
	if (m_robotSimulation)
		m_robotSimulation->onRobotSimulationTick();
}
void MainWindow::onSimulationExportRequested()
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationExportRequested();
}
void MainWindow::onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId)
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationRobotSelectionChanged(instanceIndex, sceneBackendId);
}
void MainWindow::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_robotSimulation)
		m_robotSimulation->onRobotAxisJointAnglesChanged(jointAnglesRad);
}
void MainWindow::onSimulationTcpDragTeachModeChanged(bool enabled)
{
	if (m_robotSimulation)
		m_robotSimulation->onSimulationTcpDragTeachModeChanged(enabled);
}
void MainWindow::onTcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg,
										   double ezDeg)
{
	if (m_robotSimulation)
	{
		m_robotSimulation->onTcpDragTeachPoseChanged(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
	}
}
void MainWindow::onTcpDragTeachEnded()
{
	if (m_robotSimulation)
		m_robotSimulation->onTcpDragTeachEnded();
}
void MainWindow::onRobotCoordinateFramesChanged()
{
	if (m_robotSimulation)
		m_robotSimulation->onRobotCoordinateFramesChanged();
}
void MainWindow::onCaptureToolFrameFromTcp()
{
	if (m_robotSimulation)
		m_robotSimulation->onCaptureToolFrameFromTcp();
}
void MainWindow::onCaptureUserFrameFromTcp()
{
	if (m_robotSimulation)
		m_robotSimulation->onCaptureUserFrameFromTcp();
}
void MainWindow::onResetToolFrame()
{
	if (m_robotSimulation)
		m_robotSimulation->onResetToolFrame();
}
