#include "RawTrajectory.h"

#include "GeometryRef.h"
#include "RawTrajectoryMath.h"
#include "RobotInstructionTransform.h"
#include "RobotProgramCatalog.h"
#include "RobotInstructionProgram.h"

#include <FeatureListDocument.h>
#include <RigidTransform.h>

#include <json.hpp>

#include <unordered_set>

namespace RobotInstruction
{

namespace
{

bool isRawPathSegmentStart(const std::size_t index, const std::vector<std::size_t>& segmentEndExclusive)
{
	if (index == 0U)
	{
		return true;
	}
	for (const std::size_t end : segmentEndExclusive)
	{
		if (index == end)
		{
			return true;
		}
	}
	return false;
}

} // namespace

bool importRawPathToTrajectory(
	const geoalgo::RawPath& path,
	FrameStrategy strategy,
	RawTrajectory& out,
	std::string* errMsg)
{
	if (path.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty raw path";
		}
		return false;
	}
	out = RawTrajectory{};
	out.segmentEndExclusive = path.segmentEndExclusive;
	if (!path.sourceFeatureId.empty())
	{
		nlohmann::json j;
		j["schemaVersion"] = 2;
		j["features"] = nlohmann::json::array({{{"featureId", path.sourceFeatureId}}});
		out.sourceFeatureJson = j.dump();
	}
	out.points.reserve(path.points.size());
	for (std::size_t i = 0; i < path.points.size(); ++i)
	{
		const geoalgo::RawPathPoint& rp = path.points[i];
		TrajectoryPoint tp;
		tp.poseMm = Vec3{rp.positionMm.x, rp.positionMm.y, rp.positionMm.z};
		Vec3 zAxis{0.0, 0.0, 1.0};
		Vec3 xHint{1.0, 0.0, 0.0};
		if (strategy == FrameStrategy::SurfaceNormalZ && rp.hasNormal)
		{
			zAxis = Vec3{rp.normal.x, rp.normal.y, rp.normal.z};
		}
		if (rp.hasTangent)
		{
			xHint = Vec3{rp.tangent.x, rp.tangent.y, rp.tangent.z};
		}
		else if (i + 1U < path.points.size() && !isRawPathSegmentStart(i + 1U, path.segmentEndExclusive))
		{
			const auto& n = path.points[i + 1U].positionMm;
			xHint = Vec3{n.x - rp.positionMm.x, n.y - rp.positionMm.y, n.z - rp.positionMm.z};
		}
		tp.eulerDeg = eulerFromFrame(zAxis, xHint);
		out.points.push_back(tp);
	}
	return true;
}

bool applyRawTrajectoryOp(const RawTrajectoryOpDescriptor& op, RawTrajectory& trajectory, std::string* errMsg)
{
	(void)op;
	(void)trajectory;
	if (errMsg)
	{
		*errMsg = "raw trajectory op pipeline removed; use UnifiedTrajectory pipeline";
	}
	return false;
}

bool applyRawTrajectoryPipeline(
	const std::vector<RawTrajectoryOpDescriptor>& ops,
	RawTrajectory& trajectory,
	std::string* errMsg)
{
	(void)trajectory;
	if (ops.empty())
	{
		return true;
	}
	if (errMsg)
	{
		*errMsg = "raw trajectory pipeline removed; use UnifiedTrajectory pipeline";
	}
	return false;
}

bool emitRawTrajectoryToProgram(
	const RawTrajectory& trajectory,
	RobotProgram& program,
	std::string* errMsg,
	std::string* outGroupId,
	const std::string* pathPlanInstructionId)
{
	if (trajectory.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty trajectory";
		}
		return false;
	}
	if (pathPlanInstructionId && !pathPlanInstructionId->empty())
	{
		std::unordered_set<std::string> staleMotionIds;
		for (auto it = program.groups.begin(); it != program.groups.end();)
		{
			if (it->role == InstructionGroupRole::PathPlanOutput
				&& it->pathPlanInstructionId == *pathPlanInstructionId)
			{
				for (const std::string& id : it->memberInstructionIds)
				{
					staleMotionIds.insert(id);
				}
				it = program.groups.erase(it);
			}
			else
			{
				++it;
			}
		}
		program.steps.erase(
			std::remove_if(
				program.steps.begin(),
				program.steps.end(),
				[&staleMotionIds](const std::shared_ptr<Base>& ins) {
					return ins && staleMotionIds.count(ins->id()) != 0;
				}),
			program.steps.end());
	}
	else
	{
		program.steps.clear();
		program.groups.clear();
	}

	std::vector<std::pair<std::size_t, std::size_t>> segments;
	segments.reserve(trajectory.segmentEndExclusive.size() + 1U);
	std::size_t segStart = 0U;
	if (trajectory.segmentEndExclusive.empty())
	{
		segments.emplace_back(0U, trajectory.points.size());
	}
	else
	{
		for (const std::size_t end : trajectory.segmentEndExclusive)
		{
			if (end > segStart && end <= trajectory.points.size())
			{
				segments.emplace_back(segStart, end);
				segStart = end;
			}
		}
	}
	if (segments.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty trajectory segments";
		}
		return false;
	}

	std::vector<std::shared_ptr<Base>> newMotion;
	newMotion.reserve(trajectory.points.size());
	const std::string featureId = rawTrajectoryFeatureId(trajectory);
	int idx = 0;
	int segIdx = 0;
	std::string firstGroupId;
	for (const auto& seg : segments)
	{
		std::vector<std::string> memberIds;
		memberIds.reserve(seg.second - seg.first);
		for (std::size_t i = seg.first; i < seg.second; ++i)
		{
			const TrajectoryPoint& tp = trajectory.points[i];
			if (!tp.reachable)
			{
				continue;
			}
			auto ins = std::make_shared<LineInstruction>();
			ins->setName("P" + std::to_string(++idx));
			const engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
				tp.poseMm.x,
				tp.poseMm.y,
				tp.poseMm.z,
				tp.eulerDeg.x,
				tp.eulerDeg.y,
				tp.eulerDeg.z);
			writeTargetTransformToInstruction(*ins, target);
			ins->setBlendRadius(tp.blendRadiusMm);
			if (tp.speedMmPerSec > 0.0)
			{
				ins->setSpeed(tp.speedMmPerSec);
			}
			memberIds.push_back(ins->id());
			newMotion.push_back(std::move(ins));
		}
		if (memberIds.empty())
		{
			continue;
		}
		InstructionGroup group;
		group.id = makeGroupId();
		if (segments.size() > 1U)
		{
			++segIdx;
			group.name = featureId.empty()
				? ("RawTrajectory_S" + std::to_string(segIdx))
				: (featureId + "_S" + std::to_string(segIdx));
		}
		else
		{
			group.name = featureId.empty() ? "RawTrajectory" : featureId;
		}
		group.memberInstructionIds = std::move(memberIds);
		if (pathPlanInstructionId && !pathPlanInstructionId->empty())
		{
			group.role = InstructionGroupRole::PathPlanOutput;
			group.pathPlanInstructionId = *pathPlanInstructionId;
		}
		if (firstGroupId.empty())
		{
			firstGroupId = group.id;
		}
		program.groups.push_back(std::move(group));
	}

	if (newMotion.empty())
	{
		if (errMsg)
		{
			*errMsg = "no reachable points";
		}
		return false;
	}
	for (std::shared_ptr<Base>& motion : newMotion)
	{
		program.steps.push_back(std::move(motion));
	}
	if (program.steps.empty())
	{
		if (errMsg)
		{
			*errMsg = "no reachable points";
		}
		return false;
	}
	if (outGroupId)
	{
		*outGroupId = firstGroupId;
	}
	return true;
}

std::string rawTrajectoryToPreviewPolylineXyz(const RawTrajectory& trajectory)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		arr.push_back(tp.poseMm.x);
		arr.push_back(tp.poseMm.y);
		arr.push_back(tp.poseMm.z);
	}
	return arr.dump();
}

std::string rawTrajectoryWorkpieceBackendId(const RawTrajectory& trajectory)
{
	if (trajectory.sourceFeatureJson.empty())
	{
		return {};
	}
	geoalgo::FeatureListDocument doc{};
	std::string err;
	if (!geometry_backend_ops::featureListFromJson(trajectory.sourceFeatureJson, doc, &err))
	{
		return {};
	}
	return doc.workpiece.backendIdUtf8;
}

std::string rawTrajectoryFeatureId(const RawTrajectory& trajectory)
{
	if (trajectory.sourceFeatureJson.empty())
	{
		return {};
	}
	geoalgo::FeatureListDocument doc{};
	std::string err;
	if (!geometry_backend_ops::featureListFromJson(trajectory.sourceFeatureJson, doc, &err))
	{
		return {};
	}
	if (!doc.features.empty())
	{
		return doc.features.front().featureId;
	}
	return {};
}

std::string rawTrajectoryReachabilityColorsJson(const RawTrajectory& trajectory)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		arr.push_back(tp.reachable ? "#00ff00" : "#ff0000");
	}
	return arr.dump();
}

} // namespace RobotInstruction
