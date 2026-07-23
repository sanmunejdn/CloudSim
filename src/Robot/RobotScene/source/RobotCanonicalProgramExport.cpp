/// @file RobotCanonicalProgramExport.cpp
/// @brief RobotCanonicalProgramExport 实现

#include "RobotCanonicalProgramExport.h"

#include "RobotInstructionCondition.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotProgramExport.h"

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace RobotCanonicalExport
{
namespace
{
using namespace RobotInstruction;

nlohmann::json vec3Json(const Vec3& v)
{
	return nlohmann::json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

nlohmann::json posesFromInstruction(const Base& ins)
{
	nlohmann::json poses = nlohmann::json::object();
	poses["primaryFrame"] = "base_tcp_mm_deg";
	if (ins.hasPoseProperty())
	{
		poses["baseTcp"] = nlohmann::json::object();
		poses["baseTcp"]["positionMm"] = vec3Json(ins.pose());
		poses["baseTcp"]["eulerDeg"] = vec3Json(ins.eulerDeg());
	}
	if (ins.hasViaPoseProperty())
	{
		poses["viaTcp"] = nlohmann::json::object();
		poses["viaTcp"]["positionMm"] = vec3Json(ins.viaPose());
		poses["viaTcp"]["eulerDeg"] = vec3Json(ins.viaEulerDeg());
	}
	const auto& ext = ins.extensionProperties();
	const auto itTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	if (itTool != ext.end())
	{
		poses["toolFrameId"] = itTool->second;
	}
	const auto itUser = ext.find(RobotCoordinate::kExtMotionUserFrameId);
	if (itUser != ext.end())
	{
		poses["userFrameId"] = itUser->second;
	}
	return poses;
}

nlohmann::json kinematicsFromPlan(const PlanResult* plan)
{
	nlohmann::json k = nlohmann::json::object();
	if (!plan)
	{
		k["ikOk"] = false;
		return k;
	}
	k["ikOk"] = plan->ok;
	if (!plan->plannerName.empty())
	{
		k["plannerName"] = plan->plannerName;
	}
	if (plan->durationSec > 0.0)
	{
		k["durationSec"] = plan->durationSec;
	}
	if (!plan->jointTargetsRad.empty())
	{
		k["jointRad"] = plan->jointTargetsRad;
	}
	if (!plan->summary.empty())
	{
		k["ikError"] = plan->summary;
	}
	return k;
}

nlohmann::json logicBlock(const Base& ins)
{
	nlohmann::json logic = nlohmann::json::object();
	if (ins.hasConditionProperty())
	{
		logic["condition"] = conditionToJson(ins.condition());
	}
	if (ins.hasDurationProperty())
	{
		logic["durationSec"] = ins.durationSec();
	}
	if (ins.hasIoPortProperty())
	{
		logic["port"] = ins.ioPort();
	}
	if (ins.hasIoValueProperty())
	{
		logic["digitalValue"] = ins.ioBoolValue();
	}
	if (ins.type() == Type::SET_AO)
	{
		logic["analogValue"] = ins.ioAnalogValue();
	}
	return logic;
}

nlohmann::json motionBlock(const Base& ins)
{
	nlohmann::json m = nlohmann::json::object();
	if (const int pn = motionPointIndex(ins); pn > 0)
	{
		m["pointIndex"] = pn;
	}
	if (ins.hasSpeedProperty())
	{
		m["speed"] = ins.speed();
	}
	if (ins.hasAccelProperty())
	{
		m["accel"] = ins.accel();
	}
	if (ins.hasBlendRadiusProperty())
	{
		m["blendRadius"] = ins.blendRadius();
	}
	return m;
}

bool isExecutableLeaf(const Type t)
{
	return isMotionWaypointType(t) || t == Type::WAIT || t == Type::SET_DO || t == Type::SET_AO;
}

struct BuildState
{
	const std::vector<PlanResult>* motionPlans = nullptr;
	size_t motionPlanCursor = 0;
	std::vector<FlatMotionRef>* flatOut = nullptr;
	int flatIndex = 0;
	bool includePathPlan = false;
};

nlohmann::json buildRecord(const Base& ins, const std::vector<size_t>& path, BuildState& st)
{
	nlohmann::json rec = nlohmann::json::object();
	rec["programStepPath"] = path;
	rec["instructionId"] = ins.id();
	rec["type"] = typeToString(ins.type());
	switch (ins.category())
	{
	case Category::Motion:
		rec["category"] = "Motion";
		break;
	case Category::Planning:
		rec["category"] = "Planning";
		break;
	default:
		rec["category"] = "Logic";
		break;
	}
	rec["name"] = ins.name();
	const bool execLeaf = isExecutableLeaf(ins.type());
	rec["executable"] = execLeaf && !isPathPlanType(ins.type());

	if (isPathPlanType(ins.type()))
	{
		const PathPlanInstruction* pp = asPathPlan(ins);
		if (pp)
		{
			nlohmann::json planning = nlohmann::json::object();
			switch (pp->phase())
			{
			case PathPlanPhase::RawReady:
				planning["phase"] = "raw_ready";
				break;
			case PathPlanPhase::Applied:
				planning["phase"] = "applied";
				break;
			default:
				planning["phase"] = "draft";
				break;
			}
			planning["pipelineOpCount"] = pp->pipeline().size();
			planning["appliedHistoryOpCount"] = pp->appliedHistory().size();
			planning["outputGroupId"] = pp->outputGroupId();
			planning["rawTrajectoryKey"] = pp->rawTrajectoryKey();
			planning["rawRevision"] = pp->rawRevision();
			rec["planning"] = std::move(planning);
		}
		return rec;
	}

	if (ins.category() == Category::Motion || ins.hasPoseProperty())
	{
		rec["motion"] = motionBlock(ins);
		rec["poses"] = posesFromInstruction(ins);
		// 无规划结果时跳过 kinematics，品牌导出万级点时可显著缩小 JSON
		if (st.motionPlans)
		{
			const PlanResult* plan = nullptr;
			if (st.motionPlanCursor < st.motionPlans->size())
			{
				plan = &(*st.motionPlans)[st.motionPlanCursor++];
			}
			rec["kinematics"] = kinematicsFromPlan(plan);
		}
		if (st.flatOut && isMotionWaypointType(ins.type()))
		{
			FlatMotionRef ref;
			ref.flatIndex = st.flatIndex++;
			ref.instructionId = ins.id();
			ref.programStepPath = path;
			ref.pointIndex = motionPointIndex(ins);
			st.flatOut->push_back(ref);
		}
	}
	else
	{
		rec["logic"] = logicBlock(ins);
	}

	return rec;
}

nlohmann::json buildNested(const std::vector<std::shared_ptr<Base>>& steps, std::vector<size_t> pathPrefix,
						   BuildState& st, const bool includePathPlan)
{
	nlohmann::json arr = nlohmann::json::array();
	for (size_t i = 0; i < steps.size(); ++i)
	{
		const auto& ins = steps[i];
		if (!ins)
		{
			continue;
		}
		if (isPathPlanType(ins->type()) && !includePathPlan)
		{
			continue;
		}
		std::vector<size_t> path = pathPrefix;
		path.push_back(i);
		nlohmann::json rec = buildRecord(*ins, path, st);
		if (ins->type() == Type::IF)
		{
			nlohmann::json thenArr = buildNested(ins->nestedSteps(), path, st, includePathPlan);
			nlohmann::json elseArr = buildNested(ins->elseSteps(), path, st, includePathPlan);
			rec["then"] = std::move(thenArr);
			rec["else"] = std::move(elseArr);
		}
		else if (ins->type() == Type::WHILE)
		{
			rec["body"] = buildNested(ins->nestedSteps(), path, st, includePathPlan);
		}
		arr.push_back(std::move(rec));
	}
	return arr;
}

std::string utcNowIso8601()
{
	using clock = std::chrono::system_clock;
	const auto t = clock::to_time_t(clock::now());
	std::tm tmUtc{};
#if defined(_WIN32)
	gmtime_s(&tmUtc, &t);
#else
	gmtime_r(&t, &tmUtc);
#endif
	std::ostringstream oss;
	oss << std::put_time(&tmUtc, "%Y-%m-%dT%H:%M:%SZ");
	return oss.str();
}

nlohmann::json coordinateFramesToJson(const RobotCoordinate::RobotCoordinateFrameSet& set)
{
	nlohmann::json j = nlohmann::json::object();
	j["flangeLinkName"] = set.flangeLinkName;
	j["activeToolFrameId"] = set.activeToolFrameId;
	j["activeUserFrameId"] = set.activeUserFrameId;
	nlohmann::json tools = nlohmann::json::array();
	for (const auto& tf : set.toolFrames)
	{
		nlohmann::json tj = nlohmann::json::object();
		tj["id"] = tf.id;
		tj["name"] = tf.name;
		tj["flangeLinkName"] = tf.flangeLinkName;
		tj["T_flange_tool"] = nlohmann::json::object();
		tj["T_flange_tool"]["positionMm"] = {tf.T_flange_tool.positionMm[0], tf.T_flange_tool.positionMm[1],
											 tf.T_flange_tool.positionMm[2]};
		tj["T_flange_tool"]["eulerDeg"] = {tf.T_flange_tool.eulerDeg[0], tf.T_flange_tool.eulerDeg[1],
										   tf.T_flange_tool.eulerDeg[2]};
		tools.push_back(std::move(tj));
	}
	j["toolFrames"] = std::move(tools);
	nlohmann::json users = nlohmann::json::array();
	for (const auto& uf : set.userFrames)
	{
		nlohmann::json uj = nlohmann::json::object();
		uj["id"] = uf.id;
		uj["name"] = uf.name;
		uj["T_base_user"] = nlohmann::json::object();
		uj["T_base_user"]["positionMm"] = {uf.T_base_user.positionMm[0], uf.T_base_user.positionMm[1],
										   uf.T_base_user.positionMm[2]};
		uj["T_base_user"]["eulerDeg"] = {uf.T_base_user.eulerDeg[0], uf.T_base_user.eulerDeg[1],
										 uf.T_base_user.eulerDeg[2]};
		users.push_back(std::move(uj));
	}
	j["userFrames"] = std::move(users);
	return j;
}
} // namespace

bool buildCanonicalExportV1(const RobotProgram& program, const InstructionRuntimeResolveContext& ctx,
							const CanonicalExportLayout layout, const bool includePathPlanMetadata,
							const std::vector<PlanResult>* motionPlansInDfsOrder, CanonicalProgramExportV1& out,
							std::string* errMsg)
{
	(void)errMsg;
	out = CanonicalProgramExportV1{};
	out.exportedAtUtc = utcNowIso8601();
	out.programId = program.id;
	out.programName = program.name;
	out.layout = layout;
	out.robotInstanceIndex = ctx.robotInstanceIndex;
	out.robotSceneBackendId = ctx.robotSceneBackendId;
	out.urdfPath = ctx.urdfPath;
	if (ctx.coordinateFrames)
	{
		out.coordinateFrames = *ctx.coordinateFrames;
	}

	BuildState st;
	st.motionPlans = motionPlansInDfsOrder;
	st.flatOut = &out.flatMotionSequence;

	if (layout == CanonicalExportLayout::NestedTree)
	{
		out.instructions = buildNested(program.steps, {}, st, includePathPlanMetadata);
	}
	else
	{
		const std::vector<const Base*> motions = collectMotionInstructions(program.steps);
		nlohmann::json flat = nlohmann::json::array();
		size_t planIdx = 0;
		int flatIdx = 0;
		for (const Base* ins : motions)
		{
			if (!ins)
			{
				continue;
			}
			std::vector<size_t> path;
			nlohmann::json rec = buildRecord(*ins, path, st);
			const PlanResult* plan = nullptr;
			if (motionPlansInDfsOrder && planIdx < motionPlansInDfsOrder->size())
			{
				plan = &(*motionPlansInDfsOrder)[planIdx++];
				rec["kinematics"] = kinematicsFromPlan(plan);
			}
			rec["index"] = flatIdx++;
			flat.push_back(std::move(rec));
			if (isMotionWaypointType(ins->type()))
			{
				FlatMotionRef ref;
				ref.flatIndex = st.flatIndex++;
				ref.instructionId = ins->id();
				ref.pointIndex = motionPointIndex(*ins);
				out.flatMotionSequence.push_back(ref);
			}
		}
		out.instructions = std::move(flat);
	}
	return true;
}

bool writeCanonicalExportV1ToJson(const CanonicalProgramExportV1& doc, std::string& outJson, std::string* errMsg,
								  bool prettyPrint)
{
	(void)errMsg;
	nlohmann::json root = nlohmann::json::object();
	root["format"] = kFormatId;
	root["schemaVersion"] = kSchemaVersion;
	root["exportedAt"] = doc.exportedAtUtc;
	root["exportLayout"] = doc.layout == CanonicalExportLayout::NestedTree ? "nested_tree" : "flat_motion";
	root["program"] = nlohmann::json{{"id", doc.programId}, {"name", doc.programName}};
	root["robot"] = nlohmann::json{{"instanceIndex", doc.robotInstanceIndex},
								   {"sceneBackendId", doc.robotSceneBackendId},
								   {"urdfPath", doc.urdfPath}};
	root["coordinateFrames"] = coordinateFramesToJson(doc.coordinateFrames);
	root["instructions"] = doc.instructions;
	nlohmann::json flatArr = nlohmann::json::array();
	for (const FlatMotionRef& ref : doc.flatMotionSequence)
	{
		nlohmann::json item = nlohmann::json::object();
		item["flatIndex"] = ref.flatIndex;
		item["instructionId"] = ref.instructionId;
		item["programStepPath"] = ref.programStepPath;
		item["pointIndex"] = ref.pointIndex;
		flatArr.push_back(std::move(item));
	}
	root["flatMotionSequence"] = std::move(flatArr);
	outJson = prettyPrint ? root.dump(2) : root.dump();
	return true;
}

} // namespace RobotCanonicalExport
