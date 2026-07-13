#include "TrajectoryPipelineEngine.h"

#include "RobotSceneGeometryProjection.h"
#include "TrajectoryOpBridge.h"

#include <ITrajectoryOp.h>
#include <TrajectoryOpExecutionContext.h>

#include <TrajectoryOpParamsParse.h>

#include <cmath>

namespace RobotInstruction
{
namespace
{

bool unifiedPointsNear(
	const UnifiedTrajectory& a,
	const UnifiedTrajectory& b,
	const double posEpsMm)
{
	if (a.points.size() != b.points.size())
	{
		return false;
	}
	for (std::size_t i = 0; i < a.points.size(); ++i)
	{
		const UnifiedTrajectoryPoint& pa = a.points[i];
		const UnifiedTrajectoryPoint& pb = b.points[i];
		if (std::abs(pa.poseMm.x - pb.poseMm.x) > posEpsMm
			|| std::abs(pa.poseMm.y - pb.poseMm.y) > posEpsMm
			|| std::abs(pa.poseMm.z - pb.poseMm.z) > posEpsMm)
		{
			return false;
		}
	}
	return true;
}

} // namespace

TrajectoryPipelineEngine::TrajectoryPipelineEngine() = default;

TrajectoryPipelineEngine::~TrajectoryPipelineEngine() = default;

TrajectoryOpPhase trajectoryOpPhase(const TrajectoryOpKind kind)
{
	if (kind == TrajectoryOpKind::Delete || kind == TrajectoryOpKind::Duplicate)
	{
		return TrajectoryOpPhase::Structural;
	}
	return TrajectoryOpPhase::Geometry;
}

void TrajectoryPipelineEngine::clear()
{
	m_usingRaw = false;
	m_sourceRaw = {};
	m_rawWorking = {};
	m_rawRebuild = nullptr;
	m_program = nullptr;
	m_ops.clear();
	m_steps.clear();
	m_result = {};
	m_baseline = {};
	m_baselineValid = false;
}

void TrajectoryPipelineEngine::setUsingRaw(const bool usingRaw)
{
	m_usingRaw = usingRaw;
}

void TrajectoryPipelineEngine::setSourceRaw(RawTrajectory raw)
{
	m_sourceRaw = std::move(raw);
	m_rawWorking = m_sourceRaw;
}

void TrajectoryPipelineEngine::setRawRebuildFn(RawRebuildFn rebuild)
{
	m_rawRebuild = std::move(rebuild);
}

void TrajectoryPipelineEngine::setProgramContext(const RobotProgram* program)
{
	m_program = program;
}

void TrajectoryPipelineEngine::setUnifiedBaseline(UnifiedTrajectory baseline)
{
	m_baseline = std::move(baseline);
	m_result = m_baseline;
	m_baselineValid = !m_baseline.points.empty();
}

void TrajectoryPipelineEngine::setOps(std::vector<TrajectoryOpDescriptor> ops)
{
	m_ops = std::move(ops);
	rebuildStepList();
}

void TrajectoryPipelineEngine::rebuildStepList()
{
	m_steps.clear();
	for (const TrajectoryOpDescriptor& op : m_ops)
	{
		m_steps.push_back(PipelineStep{ op, {} });
	}
	invalidateFrom(0);
}

void TrajectoryPipelineEngine::invalidateFrom(const std::size_t stepIndex)
{
	for (std::size_t i = stepIndex; i < m_steps.size(); ++i)
	{
		m_steps[i].cachedUnified.reset();
	}
	m_baselineValid = false;
}

bool TrajectoryPipelineEngine::restoreStateBeforeStep(
	const std::size_t stepIndex,
	std::string* errMsg)
{
	if (stepIndex == 0)
	{
		if (m_usingRaw)
		{
			if (!m_rawRebuild)
			{
				if (errMsg)
				{
					*errMsg = "raw rebuild callback missing";
				}
				return false;
			}
			m_rawWorking = m_sourceRaw;
			if (!m_rawRebuild(m_sourceRaw, m_baseline, errMsg))
			{
				return false;
			}
			m_baseline.ctx = m_sourceRaw.ctx;
			m_baseline.sourceFeatureJson = m_sourceRaw.sourceFeatureJson;
		}
		else if (!m_baselineValid)
		{
			if (errMsg)
			{
				*errMsg = "unified baseline not set";
			}
			return false;
		}
		m_baselineValid = true;
		m_result = m_baseline;
		return true;
	}
	const std::size_t prev = stepIndex - 1;
	PipelineStep& prevStep = m_steps[prev];
	if (!prevStep.cachedUnified.has_value())
	{
		if (errMsg)
		{
			*errMsg = "pipeline cache missing before partial rerun";
		}
		return false;
	}
	m_result = *prevStep.cachedUnified;
	return true;
}

bool TrajectoryPipelineEngine::applyGeometryOp(
	const TrajectoryOpDescriptor& op,
	UnifiedTrajectory& unified,
	std::string* errMsg)
{
	ensureTrajectoryOpBuiltinsRegistered();
	const trajectory_algo::ITrajectoryOp* impl = trajectoryOpGet(op.kind);
	if (!impl)
	{
		if (errMsg)
		{
			*errMsg = "trajectory op not registered";
		}
		return false;
	}
	trajectory_algo::TrajectoryOpExecutionContext ctx{};
	ctx.program = m_program;
	ctx.geometryProjection = &robotSceneGeometryProjection();
	if (impl->processPath(op, unified, ctx, errMsg))
	{
		return true;
	}
	if (errMsg && errMsg->empty())
	{
		*errMsg = std::string("trajectory op failed: ") + impl->displayName(false);
	}
	return false;
}

bool TrajectoryPipelineEngine::runStep(
	PipelineStep& step,
	UnifiedTrajectory& unified,
	std::string* errMsg)
{
	if (!applyGeometryOp(step.op, unified, errMsg))
	{
		return false;
	}
	step.cachedUnified = unified;
	m_result = unified;
	return true;
}

bool TrajectoryPipelineEngine::executeFrom(const std::size_t stepIndex, std::string* errMsg)
{
	if (stepIndex > m_steps.size())
	{
		if (errMsg)
		{
			*errMsg = "step index out of range";
		}
		return false;
	}
	if (stepIndex > 0 && !m_steps[stepIndex - 1].cachedUnified.has_value())
	{
		if (!executeFrom(0, errMsg))
		{
			return false;
		}
	}
	invalidateFrom(stepIndex);
	UnifiedTrajectory working{};
	if (stepIndex == 0)
	{
		if (!restoreStateBeforeStep(0, errMsg))
		{
			return false;
		}
		working = m_baseline;
	}
	else if (!restoreStateBeforeStep(stepIndex, errMsg))
	{
		return false;
	}
	else
	{
		working = m_result;
	}
	for (std::size_t i = stepIndex; i < m_steps.size(); ++i)
	{
		if (!runStep(m_steps[i], working, errMsg))
		{
			return false;
		}
	}
	m_result = working;
	return true;
}

bool TrajectoryPipelineEngine::executeFull(std::string* errMsg)
{
	if (m_usingRaw)
	{
		return executeFrom(0, errMsg);
	}
	if (!m_baselineValid)
	{
		if (errMsg)
		{
			*errMsg = "unified baseline not set";
		}
		return false;
	}
	m_result = m_baseline;
	invalidateFrom(0);
	UnifiedTrajectory working = m_baseline;
	for (PipelineStep& step : m_steps)
	{
		if (!runStep(step, working, errMsg))
		{
			return false;
		}
	}
	m_result = working;
	return true;
}

bool runTrajectoryPipelineEngineSelfCheck(std::string* errMsg)
{
	ensureTrajectoryOpBuiltinsRegistered();

	TrajectoryPipelineEngine engine;
	RawTrajectory raw{};
	raw.points.push_back(TrajectoryPoint{ Vec3{ 0.0, 0.0, 0.0 }, Vec3{}, 0.0, 0.0, true });
	raw.points.push_back(TrajectoryPoint{ Vec3{ 100.0, 0.0, 0.0 }, Vec3{}, 0.0, 0.0, true });
	engine.setUsingRaw(true);
	engine.setSourceRaw(raw);
	engine.setRawRebuildFn([](const RawTrajectory& source, UnifiedTrajectory& out, std::string* localErr) {
		return unifiedTrajectoryFromRaw(source, out, localErr);
	});

	TrajectoryOpDescriptor resample{};
	resample.kind = TrajectoryOpKind::Resample;
	trajectory_algo::writeResampleParams(resample.params, RobotInstruction::ResampleParams{ 10.0 });

	TrajectoryOpDescriptor translate{};
	translate.kind = TrajectoryOpKind::Translate;
	RobotInstruction::TranslateParams translateParams{};
	translateParams.frame = TransformReferenceFrame::World;
	translateParams.dxMm = 10.0;
	translateParams.endDxMm = 10.0;
	trajectory_algo::writeTranslateParams(translate.params, translateParams);

	engine.setOps({ resample, translate });
	if (!engine.executeFull(errMsg))
	{
		return false;
	}
	const UnifiedTrajectory fullResult = engine.result();
	if (fullResult.points.size() < 3U)
	{
		if (errMsg)
		{
			*errMsg = "resample+translate produced too few points";
		}
		return false;
	}

	engine.setOps({ resample, translate });
	engine.invalidateFrom(1);
	if (!engine.executeFrom(1, errMsg))
	{
		return false;
	}
	if (!unifiedPointsNear(fullResult, engine.result(), 1e-3))
	{
		if (errMsg)
		{
			*errMsg = "executeFrom partial result mismatch";
		}
		return false;
	}
	return true;
}

} // namespace RobotInstruction
