#ifndef ROBOTSCENE_ROBOTPROGRAMEXECUTOR_H
#define ROBOTSCENE_ROBOTPROGRAMEXECUTOR_H

/// @file RobotProgramExecutor.h
/// @brief 单运动学实例执行机器人程序（运动+逻辑）

#include "robot_scene_global.h"

#include "IRobotIoSink.h"
#include "RobotInstructionController.h"
#include "RobotInstructionPlaybackEngine.h"

#include <QElapsedTimer>
#include <QHash>
#include <QVector>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;

/// 单运动学实例执行机器人程序（运动+逻辑）
class ROBOT_SCENE_API RobotProgramExecutor
{
public:
	bool isRunning() const { return m_running; }
	void stop();

	bool tryStart(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, IRobotIoSink* io, int robotInstanceIndex,
				  const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
				  const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
				  const QVector<double>& initialJointAnglesRad, QString* errorOut);

	RobotInstructionPlaybackTickResult tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);

	const QVector<double>& jointAnglesRad() const { return m_jointAnglesRad; }
	const RobotInstruction::Base* activeMotion() const { return m_activeMotion; }
	const RobotInstruction::Base* currentInstruction() const;
	/// 当前运动段进度 [0,1]；非运动中为 1
	double motionSegmentProgress01() const;
	/// 因规划失败停机时的摘要；正常结束为空
	const std::string& lastAbortSummary() const { return m_lastAbortSummary; }
	bool abortedDueToFailedPlan() const { return m_abortedDueToFailedPlan; }

	/// 运行中懒规划：用最终结果替换占位 PlanResult
	bool updateMotionPlanResult(const RobotInstruction::Base* ins, const RobotInstruction::PlanResult& plan);
	const RobotInstruction::PlanResult* motionPlanResult(const RobotInstruction::Base* ins) const;

private:
	struct ListFrame
	{
		std::vector<std::shared_ptr<RobotInstruction::Base>> steps;
		size_t pc = 0;
		const RobotInstruction::WhileInstruction* whileIns = nullptr;
		int whileIteration = 0;
	};

	bool evaluateCondition(const RobotInstruction::Condition& c) const;
	bool advanceProgramStep(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);
	bool startMotionSegment(const RobotInstruction::Base& ins);
	bool tickMotionSegment(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);
	bool applyJointAngles(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg);
	const RobotInstruction::PlanResult* planForMotion(const RobotInstruction::Base& ins) const;

	bool m_running = false;
	bool m_abortedDueToFailedPlan = false;
	std::string m_lastAbortSummary;
	int m_robotInstanceIndex = 0;
	int m_jointOffset = 0;
	int m_jointCount = 0;
	IRobotIoSink* m_io = nullptr;

	std::vector<ListFrame> m_stack;
	std::unordered_map<const RobotInstruction::Base*, size_t> m_motionPlanIndex;

	std::vector<RobotInstruction::PlanResult> m_motionPlanResults;
	QVector<double> m_jointAnglesRad;

	bool m_inMotion = false;
	QElapsedTimer m_segmentTimer;
	double m_segDurationSec = 0.0;
	QVector<double> m_segStartJointAngles;
	const RobotInstruction::Base* m_activeMotion = nullptr;

	QHash<QString, osg::Matrixd> m_fkMeshWorldT0;
	std::unordered_map<std::string, osg::Matrixd> m_outerWorldAtStart;

	static constexpr int kMaxWhileIterations = 10000;
};

#endif // ROBOTSCENE_ROBOTPROGRAMEXECUTOR_H
