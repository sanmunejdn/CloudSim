#ifndef ROBOTSCENE_ROBOTINSTRUCTIONPLAYBACKENGINE_H
#define ROBOTSCENE_ROBOTINSTRUCTIONPLAYBACKENGINE_H

/// @file RobotInstructionPlaybackEngine.h
/// @brief Joint-space command queue playback: interpolates per segment and updates OSG using bind snapshots.

#include "robot_scene_global.h"

#include "RobotInstructionController.h"
#include "RobotSimulationTypes.h"

#include <QElapsedTimer>
#include <QHash>
#include <QString>
#include <QVector>
#include <string>
#include <unordered_map>

#include <osg/Matrixd>

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

	bool tryStart(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
				  const QVector<RobotSimulationCommand>& queue, const QVector<double>& initialJointAnglesRad,
				  QString* errorOut);

	/// Adapter path: start playback from controller plan results while reusing legacy joint interpolation executor.
	bool tryStartFromPlanResults(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
								 const QVector<RobotSimulationCommand>& legacyQueue,
								 const std::vector<RobotInstruction::PlanResult>& planResults,
								 const QVector<double>& initialJointAnglesRad, QString* errorOut);

	RobotInstructionPlaybackTickResult tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);

	const QVector<double>& jointAnglesRad() const { return m_jointAnglesRad; }

private:
	bool applyPlannedJointStateAtProgress(double u);
	void startNewSegmentFromCurrentState();
	double resolveSegmentTargetRad(int segmentIndex, double defaultTargetRad) const;
	const RobotInstruction::PlanResult* currentPlanResult() const;

private:
	bool m_running = false;
	QVector<RobotSimulationCommand> m_queue;
	std::vector<RobotInstruction::PlanResult> m_planResults;
	int m_segmentIndex = 0;
	QVector<double> m_jointAnglesRad;
	QHash<QString, osg::Matrixd> m_fkMeshWorldT0;
	std::unordered_map<std::string, osg::Matrixd> m_outerWorldAtStart;
	QVector<double> m_segStartJointAngles;
	double m_segAngleStartRad = 0.0;
	double m_segAngleTargetRad = 0.0;
	double m_segDurationSec = 0.0;
	QElapsedTimer m_segmentTimer;
};

#endif // ROBOTSCENE_ROBOTINSTRUCTIONPLAYBACKENGINE_H
