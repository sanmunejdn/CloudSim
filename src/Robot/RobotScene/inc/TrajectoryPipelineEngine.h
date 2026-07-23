#ifndef ROBOTSCENE_TRAJECTORYPIPELINEENGINE_H
#define ROBOTSCENE_TRAJECTORYPIPELINEENGINE_H

/// @file TrajectoryPipelineEngine.h
/// @brief 统一 IR 管道：单一步骤列表顺序重放，支持节点缓存与局部重跑

#include "robot_scene_global.h"

#include "RawTrajectory.h"
#include "RobotInstructionModel.h"
#include "TrajectoryPathAdapters.h"
#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <IExternalAxisSearchService.h>
#include <RigidTransform.h>

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

/// 统一 IR 管道：单一步骤列表顺序重放，支持节点缓存与局部重跑
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
	/// 非空则缓存当前 TCP（基座系）；空则清除
	void setWorkpieceReferenceInBase(const engine::RigidTransform* pose);
	/// 解析 Frame 后端世界位姿为外部 TCP；空则仅用手动六参数
	using ExternalTcpFrameResolveFn =
		std::function<bool(const std::string& backendId, engine::RigidTransform& out, std::string* errMsg)>;
	void setExternalTcpFrameResolver(ExternalTcpFrameResolveFn resolver);

	void setExternalAxisSearchService(const trajectory_algo::IExternalAxisSearchService* service);
	void setExternalAxisConfigs(std::vector<trajectory_algo::ExternalAxisSearchConfigDto> configs);

	void setUnifiedBaseline(UnifiedTrajectory baseline);
	void setOps(std::vector<TrajectoryOpDescriptor> ops);

	void invalidateFrom(std::size_t stepIndex);
	bool executeFull(std::string* errMsg = nullptr);
	bool executeFrom(std::size_t stepIndex, std::string* errMsg = nullptr);

	const UnifiedTrajectory& result() const { return m_result; }
	const RawTrajectory& rawWorking() const { return m_rawWorking; }
	std::size_t stepCount() const { return m_steps.size(); }

private:
	struct PipelineStep
	{
		TrajectoryOpDescriptor op{};
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
	bool m_hasWorkpieceReferenceInBase = false;
	engine::RigidTransform m_workpieceReferenceInBase{};
	ExternalTcpFrameResolveFn m_externalTcpFrameResolver;
	const trajectory_algo::IExternalAxisSearchService* m_externalAxisSearch = nullptr;
	std::vector<trajectory_algo::ExternalAxisSearchConfigDto> m_externalAxisConfigs;
	std::vector<TrajectoryOpDescriptor> m_ops;
	std::vector<PipelineStep> m_steps;
	UnifiedTrajectory m_result{};
	UnifiedTrajectory m_baseline{};
	bool m_baselineValid = false;
};

ROBOT_SCENE_API bool runTrajectoryPipelineEngineSelfCheck(std::string* errMsg = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYPIPELINEENGINE_H
