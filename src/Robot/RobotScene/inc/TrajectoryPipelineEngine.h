#pragma once



#include "RawTrajectory.h"
#include "RobotInstructionModel.h"

#include "TrajectoryPathAdapters.h"

#include "TrajectoryPipelineTypes.h"

#include "UnifiedTrajectory.h"

#include "robot_scene_global.h"



#include <cstddef>

#include <functional>

#include <optional>

#include <string>

#include <vector>



namespace RobotInstruction

{



enum class ROBOT_SCENE_API TrajectoryOpPhase : std::uint8_t

{

	Geometry = 0,

	Structural,

	Display

};



ROBOT_SCENE_API TrajectoryOpPhase trajectoryOpPhase(TrajectoryOpKind kind);



/// 统一 IR 管道：committed + draft 顺序重放，支持节点缓存与局部重跑

class ROBOT_SCENE_API TrajectoryPipelineEngine

{

public:

	using RawRebuildFn = RawToUnifiedRebuildFn;



	TrajectoryPipelineEngine();

	~TrajectoryPipelineEngine();



	void clear();



	void setUsingRaw(bool usingRaw);

	void setSourceRaw(RawTrajectory raw);

	void setRawRebuildFn(RawRebuildFn rebuild);

	void setProgramContext(const RobotProgram* program);



	void setUnifiedBaseline(UnifiedTrajectory baseline);

	void setPendingPreRawOps(std::vector<TrajectoryOpDescriptor> ops);

	void setCommittedOps(std::vector<TrajectoryOpDescriptor> ops);

	void setDraftOps(std::vector<TrajectoryOpDescriptor> ops);



	void invalidateFrom(std::size_t stepIndex);

	bool executeFull(std::string* errMsg = nullptr);

	bool executeFrom(std::size_t stepIndex, std::string* errMsg = nullptr);



	const UnifiedTrajectory& result() const { return m_result; }

	const RawTrajectory& rawWorking() const { return m_rawWorking; }

	std::size_t stepCount() const { return m_steps.size(); }



private:

	enum class StepKind : std::uint8_t

	{

		CommittedGeometry = 0,

		DraftGeometry

	};



	struct PipelineStep

	{

		TrajectoryOpDescriptor op{};

		StepKind kind = StepKind::DraftGeometry;

		std::optional<UnifiedTrajectory> cachedUnified;

	};



	void rebuildStepList();

	bool restoreStateBeforeStep(std::size_t stepIndex, std::string* errMsg);

	bool runStep(PipelineStep& step, UnifiedTrajectory& unified, std::string* errMsg);

	bool applyGeometryOp(const TrajectoryOpDescriptor& op, UnifiedTrajectory& unified, std::string* errMsg);



	bool m_usingRaw = false;

	RawTrajectory m_sourceRaw{};

	RawTrajectory m_rawWorking{};

	RawRebuildFn m_rawRebuild;

	const RobotProgram* m_program = nullptr;

	std::vector<TrajectoryOpDescriptor> m_pendingPreRaw;

	std::vector<TrajectoryOpDescriptor> m_committedOps;

	std::vector<TrajectoryOpDescriptor> m_draftOps;

	std::vector<PipelineStep> m_steps;

	UnifiedTrajectory m_result{};

	UnifiedTrajectory m_baseline{};

	bool m_baselineValid = false;

};



ROBOT_SCENE_API bool runTrajectoryPipelineEngineSelfCheck(std::string* errMsg = nullptr);



} // namespace RobotInstruction

