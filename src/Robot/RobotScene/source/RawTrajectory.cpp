/// @file RawTrajectory.cpp
/// @brief Raw �켣

#include "RawTrajectory.h"

#include "GeometryRef.h"
#include "RawTrajectoryMath.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotProgramCatalog.h"

#include <algorithm>
#include <unordered_set>

#include <FeatureListDocument.h>
#include <MeshTrajectory.h>
#include <RigidTransform.h>
#include <json.hpp>


namespace RobotInstruction
{
namespace
{

void applyTrajectoryPointToInstruction(Base& ins, const TrajectoryPoint& tp)
{
	engine::RigidTransform target;
	if (tp.hasQuat)
	{
		target = engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z),
			Eigen::Quaterniond(tp.quatXyzw[3], tp.quatXyzw[0], tp.quatXyzw[1], tp.quatXyzw[2]));
	}
	else
	{
		target = engine::RigidTransform::fromTranslationEulerDeg(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z, tp.eulerDeg.x,
																 tp.eulerDeg.y, tp.eulerDeg.z);
	}
	writeTargetTransformToInstruction(ins, target);
	ins.setBlendRadius(tp.blendRadiusMm);
	if (tp.speedMmPerSec > 0.0)
		ins.setSpeed(tp.speedMmPerSec);
	else if (!ins.hasSpeedProperty() || ins.speed() <= 0.0)
		ins.setSpeed(200.0);
	// ָ��ֻ�� TCP��jointRad �����滮�Ự��������
	ins.eraseExtensionProperty("context.currentJointRadCsv");
}

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

bool importRawPathToTrajectory(const geoalgo::RawPath& path, FrameStrategy strategy, RawTrajectory& out,
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

bool applyRawTrajectoryPipeline(const std::vector<RawTrajectoryOpDescriptor>& ops, RawTrajectory& trajectory,
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

bool emitRawTrajectoryToProgram(const RawTrajectory& trajectory, RobotProgram& program, std::string* errMsg,
								std::string* outGroupId, const std::string* pathPlanInstructionId)
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
			if (it->role == InstructionGroupRole::PathPlanOutput && it->pathPlanInstructionId == *pathPlanInstructionId)
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
		program.steps.erase(std::remove_if(program.steps.begin(), program.steps.end(),
										   [&staleMotionIds](const std::shared_ptr<Base>& ins)
										   { return ins && staleMotionIds.count(ins->id()) != 0; }),
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
			applyTrajectoryPointToInstruction(*ins, tp);
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
			group.name = featureId.empty() ? ("RawTrajectory_S" + std::to_string(segIdx))
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

bool insertRawTrajectoryBetween(const RawTrajectory& trajectory, RobotProgram& program,
								const std::string& startInstructionId, const std::string& endInstructionId,
								std::string* errMsg)
{
	if (startInstructionId.empty() || endInstructionId.empty() || startInstructionId == endInstructionId)
	{
		if (errMsg)
			*errMsg = "start/end waypoint invalid";
		return false;
	}

	int iStart = -1;
	int iEnd = -1;
	for (int i = 0; i < static_cast<int>(program.steps.size()); ++i)
	{
		const auto& ins = program.steps[static_cast<std::size_t>(i)];
		if (!ins)
			continue;
		if (ins->id() == startInstructionId)
			iStart = i;
		if (ins->id() == endInstructionId)
			iEnd = i;
	}
	if (iStart < 0 || iEnd < 0)
	{
		if (errMsg)
			*errMsg = "start/end waypoint not found in program";
		return false;
	}

	// ������˳���������֮�䣨��滮���յ�˭��˭���޹أ�
	const int iLo = std::min(iStart, iEnd);
	const int iHi = std::max(iStart, iEnd);
	if (iHi <= iLo)
	{
		if (errMsg)
			*errMsg = "start/end order invalid";
		return false;
	}
	const std::string idLo = program.steps[static_cast<std::size_t>(iLo)]->id();
	const std::string idHi = program.steps[static_cast<std::size_t>(iHi)]->id();

	std::unordered_set<std::string> removedIds;
	for (int i = iLo + 1; i < iHi; ++i)
	{
		if (program.steps[static_cast<std::size_t>(i)])
			removedIds.insert(program.steps[static_cast<std::size_t>(i)]->id());
	}
	program.steps.erase(program.steps.begin() + (iLo + 1), program.steps.begin() + iHi);

	for (auto& g : program.groups)
	{
		g.memberInstructionIds.erase(std::remove_if(g.memberInstructionIds.begin(), g.memberInstructionIds.end(),
													 [&removedIds](const std::string& id) {
														 return removedIds.count(id) != 0;
													 }),
									 g.memberInstructionIds.end());
	}

	std::vector<std::shared_ptr<Base>> inserted;
	inserted.reserve(trajectory.points.size());
	int idx = 0;
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		if (!tp.reachable)
			continue;
		auto ins = std::make_shared<LineInstruction>();
		ins->setName("Pmid" + std::to_string(++idx));
		applyTrajectoryPointToInstruction(*ins, tp);
		inserted.push_back(std::move(ins));
	}

	if (inserted.empty())
		return true;

	std::vector<std::string> newIds;
	newIds.reserve(inserted.size());
	for (const auto& ins : inserted)
		newIds.push_back(ins->id());

	const int insertAt = iLo + 1;
	program.steps.insert(program.steps.begin() + insertAt, inserted.begin(), inserted.end());

	// ���յ�����ͬһ���飬�м�����д������Ա���������ؽ���� Pmid �ֳɷ����Ķ���ڵ�
	bool groupUpdated = false;
	for (auto& g : program.groups)
	{
		auto& members = g.memberInstructionIds;
		const auto itLo = std::find(members.begin(), members.end(), idLo);
		const auto itHi = std::find(members.begin(), members.end(), idHi);
		if (itLo == members.end() || itHi == members.end())
			continue;
		members.insert(itLo + 1, newIds.begin(), newIds.end());
		groupUpdated = true;
		break;
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
	// FeatureList v2 �� MeshTrajectorySpec v1 �����ܹ��� sourceFeatureJson
	geoalgo::FeatureListDocument featureDoc{};
	std::string err;
	if (geometry_backend_ops::featureListFromJson(trajectory.sourceFeatureJson, featureDoc, &err) &&
		!featureDoc.workpiece.backendIdUtf8.empty())
	{
		return featureDoc.workpiece.backendIdUtf8;
	}
	geoalgo::MeshTrajectorySpec meshSpec{};
	if (geoalgo::meshTrajectorySpecFromJson(trajectory.sourceFeatureJson, meshSpec, &err) &&
		!meshSpec.workpiece.backendIdUtf8.empty())
	{
		return meshSpec.workpiece.backendIdUtf8;
	}
	return {};
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
