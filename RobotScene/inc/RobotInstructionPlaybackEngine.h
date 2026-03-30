#pragma once

#include "RobotSimulationTypes.h"

#include "robot_scene_global.h"

#include <QElapsedTimer>
#include <QHash>
#include <QString>
#include <QVector>

#include <osg/Matrixd>

#include <string>
#include <unordered_map>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;

enum class RobotInstructionPlaybackTickResult
{
	Continue,
	Finished,
	Aborted
};

/// Joint-space command queue playback: interpolates per segment and updates OSG using bind snapshots.
/// No QTimer; MainWindow calls \ref tick at \ref RobotSimulation::kPlaybackTimerIntervalMs.
class ROBOT_SCENE_API RobotInstructionPlaybackEngine
{
public:
	bool isRunning() const { return m_running; }

	void stop();

	bool tryStart(
		IRobotSimulationDocument* doc,
		IRobotBackendPoseSink* osg,
		const QVector<RobotSimulationCommand>& queue,
		const QVector<double>& initialJointAnglesRad,
		QString* errorOut);

	RobotInstructionPlaybackTickResult tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);

	const QVector<double>& jointAnglesRad() const { return m_jointAnglesRad; }

private:
	bool m_running = false;
	QVector<RobotSimulationCommand> m_queue;
	int m_segmentIndex = 0;
	QVector<double> m_jointAnglesRad;
	QHash<QString, osg::Matrixd> m_fkMeshWorldT0;
	std::unordered_map<std::string, osg::Matrixd> m_outerWorldAtStart;
	double m_segAngleStartRad = 0.0;
	double m_segAngleTargetRad = 0.0;
	QElapsedTimer m_segmentTimer;
};
