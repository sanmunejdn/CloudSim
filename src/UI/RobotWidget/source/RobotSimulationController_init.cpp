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
