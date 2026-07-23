/// @file RobotInstructionFactory.cpp
/// @brief RobotInstructionFactory 实现

#include "RobotInstructionFactory.h"

#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"

#include <TrajectoryOpDescriptorCodec.h>

namespace RobotInstruction
{
namespace
{
nlohmann::json vec3ToJson(const Vec3& v)
{
	return nlohmann::json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

Vec3 vec3FromJson(const nlohmann::json& j)
{
	Vec3 v;
	if (j.is_object())
	{
		v.x = j.value("x", 0.0);
		v.y = j.value("y", 0.0);
		v.z = j.value("z", 0.0);
	}
	return v;
}

void applyCommonFields(Base& ins, const nlohmann::json& j)
{
	if (j.contains("id") && j["id"].is_string())
	{
		ins.setId(j["id"].get<std::string>());
	}
	if (j.contains("name") && j["name"].is_string())
	{
		ins.setName(j["name"].get<std::string>());
	}
	std::string robotId;
	if (j.contains("robotId") && j["robotId"].is_string())
	{
		robotId = j["robotId"].get<std::string>();
	}
	else if (j.contains("controllerId") && j["controllerId"].is_string())
	{
		robotId = j["controllerId"].get<std::string>();
	}
	if (!robotId.empty())
	{
		ins.setControllerId(robotId);
	}
	if (j.contains("extensions") && j["extensions"].is_object())
	{
		for (auto it = j["extensions"].begin(); it != j["extensions"].end(); ++it)
		{
			if (it.value().is_string())
			{
				ins.setExtensionProperty(it.key(), it.value().get<std::string>());
			}
		}
	}
}

bool parseNested(const nlohmann::json& j, const char* key, std::vector<std::shared_ptr<Base>>& out, std::string* errMsg)
{
	if (!j.contains(key))
	{
		return true;
	}
	const auto& arr = j[key];
	if (!arr.is_array())
	{
		if (errMsg)
		{
			*errMsg = std::string("Expected array for ") + key;
		}
		return false;
	}
	out = createListFromJson(arr, errMsg);
	return errMsg ? errMsg->empty() : true;
}

std::string pathPlanPhaseToString(const PathPlanPhase p)
{
	switch (p)
	{
	case PathPlanPhase::RawReady:
		return "raw_ready";
	case PathPlanPhase::Applied:
		return "applied";
	default:
		return "draft";
	}
}

bool pathPlanPhaseFromString(const std::string& s, PathPlanPhase& out)
{
	if (s == "raw_ready" || s == "RawReady")
	{
		out = PathPlanPhase::RawReady;
		return true;
	}
	if (s == "applied" || s == "Applied")
	{
		out = PathPlanPhase::Applied;
		return true;
	}
	if (s == "draft" || s == "Draft")
	{
		out = PathPlanPhase::Draft;
		return true;
	}
	return false;
}

void writeCommonFields(nlohmann::json& j, const Base& ins)
{
	j["type"] = typeToString(ins.type());
	j["id"] = ins.id();
	j["name"] = ins.name();
	if (!ins.controllerId().empty())
	{
		j["robotId"] = ins.controllerId();
	}
	const auto& ext = ins.extensionProperties();
	if (!ext.empty())
	{
		nlohmann::json extObj = nlohmann::json::object();
		for (const auto& kv : ext)
		{
			extObj[kv.first] = kv.second;
		}
		j["extensions"] = std::move(extObj);
	}
}
} // namespace

std::shared_ptr<Base> createFromJson(const nlohmann::json& j, std::string* errMsg)
{
	if (!j.is_object())
	{
		if (errMsg)
		{
			*errMsg = "Instruction JSON must be an object";
		}
		return nullptr;
	}
	const std::string typeStr = j.value("type", std::string());
	Type type{};
	if (!typeFromString(typeStr, type))
	{
		if (errMsg)
		{
			*errMsg = "Unknown instruction type: " + typeStr;
		}
		return nullptr;
	}

	std::shared_ptr<Base> ins;
	switch (type)
	{
	case Type::PTP:
	{
		auto p = std::make_shared<PtpInstruction>();
		p->setPose(vec3FromJson(j.value("pose", nlohmann::json::object())));
		p->setEulerDeg(vec3FromJson(j.value("eulerDeg", nlohmann::json::object())));
		p->setSpeed(j.value("speed", 100.0));
		p->setAccel(j.value("accel", 100.0));
		if (j.contains("axisConfiguration"))
		{
			p->setMotionAxisConfiguration(motionAxisConfigurationFromJson(j["axisConfiguration"]));
		}
		else if (j.contains("axisConfig"))
		{
			p->setAxisConfig(j["axisConfig"].get<std::string>());
		}
		if (j.contains("pointIndex"))
		{
			setMotionPointIndex(*p, j["pointIndex"].get<int>());
		}
		ins = p;
		{
			engine::RigidTransform t{};
			if (readTargetTransformFromInstruction(*p, t))
			{
				writeTargetTransformToInstruction(*p, t);
			}
		}
		break;
	}
	case Type::LINE:
	{
		auto p = std::make_shared<LineInstruction>();
		p->setPose(vec3FromJson(j.value("pose", nlohmann::json::object())));
		p->setEulerDeg(vec3FromJson(j.value("eulerDeg", nlohmann::json::object())));
		p->setSpeed(j.value("speed", 200.0));
		p->setAccel(j.value("accel", 200.0));
		p->setBlendRadius(j.value("blendRadius", 0.0));
		if (j.contains("axisConfiguration"))
		{
			p->setMotionAxisConfiguration(motionAxisConfigurationFromJson(j["axisConfiguration"]));
		}
		else if (j.contains("axisConfig"))
		{
			p->setAxisConfig(j["axisConfig"].get<std::string>());
		}
		if (j.contains("pointIndex"))
		{
			setMotionPointIndex(*p, j["pointIndex"].get<int>());
		}
		ins = p;
		{
			engine::RigidTransform t{};
			if (readTargetTransformFromInstruction(*p, t))
			{
				writeTargetTransformToInstruction(*p, t);
			}
		}
		break;
	}
	case Type::ARC:
	{
		auto p = std::make_shared<ArcInstruction>();
		p->setPose(vec3FromJson(j.value("pose", nlohmann::json::object())));
		p->setEulerDeg(vec3FromJson(j.value("eulerDeg", nlohmann::json::object())));
		p->setViaPose(vec3FromJson(j.value("viaPose", nlohmann::json::object())));
		p->setViaEulerDeg(vec3FromJson(j.value("viaEulerDeg", nlohmann::json::object())));
		p->setSpeed(j.value("speed", 200.0));
		p->setAccel(j.value("accel", 200.0));
		p->setBlendRadius(j.value("blendRadius", 0.0));
		if (j.contains("axisConfiguration"))
		{
			p->setMotionAxisConfiguration(motionAxisConfigurationFromJson(j["axisConfiguration"]));
		}
		else if (j.contains("axisConfig"))
		{
			p->setAxisConfig(j["axisConfig"].get<std::string>());
		}
		if (j.contains("pointIndex"))
		{
			setMotionPointIndex(*p, j["pointIndex"].get<int>());
		}
		ins = p;
		{
			engine::RigidTransform t{};
			if (readTargetTransformFromInstruction(*p, t))
			{
				writeTargetTransformToInstruction(*p, t);
			}
			engine::RigidTransform via{};
			if (readViaTransformFromInstruction(*p, via))
			{
				writeViaTransformToInstruction(*p, via);
			}
		}
		break;
	}
	case Type::WAIT:
	{
		auto p = std::make_shared<WaitInstruction>();
		p->setDurationSec(j.value("durationSec", 1.0));
		ins = p;
		break;
	}
	case Type::IF:
	{
		auto p = std::make_shared<IfInstruction>();
		p->setCondition(conditionFromJson(j.value("condition", nlohmann::json::object())));
		if (!parseNested(j, "then", p->thenSteps(), errMsg))
		{
			return nullptr;
		}
		if (!parseNested(j, "else", p->elseStepsMut(), errMsg))
		{
			return nullptr;
		}
		ins = p;
		break;
	}
	case Type::WHILE:
	{
		auto p = std::make_shared<WhileInstruction>();
		p->setCondition(conditionFromJson(j.value("condition", nlohmann::json::object())));
		if (!parseNested(j, "body", p->bodySteps(), errMsg))
		{
			return nullptr;
		}
		ins = p;
		break;
	}
	case Type::SET_DO:
	{
		auto p = std::make_shared<SetDigitalOutputInstruction>();
		p->setIoPort(j.value("port", 0));
		p->setIoBoolValue(j.value("value", false));
		ins = p;
		break;
	}
	case Type::SET_AO:
	{
		auto p = std::make_shared<SetAnalogOutputInstruction>();
		p->setIoPort(j.value("port", 0));
		p->setIoAnalogValue(j.value("value", 0.0));
		ins = p;
		break;
	}
	case Type::PathPlan:
	{
		auto p = std::make_shared<PathPlanInstruction>();
		PathPlanPhase phase = PathPlanPhase::Draft;
		const std::string phaseStr = j.value("phase", std::string("draft"));
		pathPlanPhaseFromString(phaseStr, phase);
		p->setPhase(phase);
		p->setOutputGroupId(j.value("outputGroupId", std::string()));
		p->setRawTrajectoryKey(j.value("rawTrajectoryKey", p->id()));
		p->setRawRevision(j.value("rawRevision", 0));
		if (j.contains("sourceFeature"))
		{
			if (j["sourceFeature"].is_object())
			{
				p->setSourceFeatureJson(j["sourceFeature"].dump());
			}
			else if (j["sourceFeature"].is_string())
			{
				p->setSourceFeatureJson(j["sourceFeature"].get<std::string>());
			}
		}
		if (j.contains("pipeline"))
		{
			std::vector<TrajectoryOpDescriptor> pipeline;
			if (trajectory_algo::pipelineFromJson(j["pipeline"], pipeline, errMsg))
			{
				p->setPipeline(std::move(pipeline));
			}
		}
		if (j.contains("appliedHistory"))
		{
			std::vector<TrajectoryOpDescriptor> hist;
			if (trajectory_algo::pipelineFromJson(j["appliedHistory"], hist, errMsg))
			{
				p->appliedHistoryMut() = std::move(hist);
			}
		}
		ins = p;
		break;
	}
	default:
		if (errMsg)
		{
			*errMsg = "Unhandled instruction type";
		}
		return nullptr;
	}

	applyCommonFields(*ins, j);
	return ins;
}

nlohmann::json toJson(const Base& ins)
{
	nlohmann::json j;
	writeCommonFields(j, ins);

	switch (ins.type())
	{
	case Type::PTP:
		j["pose"] = vec3ToJson(ins.pose());
		j["eulerDeg"] = vec3ToJson(ins.eulerDeg());
		j["speed"] = ins.speed();
		j["accel"] = ins.accel();
		if (ins.hasMotionAxisConfigurationProperty())
		{
			nlohmann::json axisCfg = nlohmann::json::object();
			writeMotionAxisConfigurationToJson(ins.motionAxisConfiguration(), axisCfg);
			j["axisConfiguration"] = axisCfg;
			j["axisConfig"] = ins.axisConfig();
		}
		if (const int pointIndex = motionPointIndex(ins); pointIndex > 0)
		{
			j["pointIndex"] = pointIndex;
		}
		break;
	case Type::LINE:
		j["pose"] = vec3ToJson(ins.pose());
		j["eulerDeg"] = vec3ToJson(ins.eulerDeg());
		j["speed"] = ins.speed();
		j["accel"] = ins.accel();
		j["blendRadius"] = ins.blendRadius();
		if (ins.hasMotionAxisConfigurationProperty())
		{
			nlohmann::json axisCfg = nlohmann::json::object();
			writeMotionAxisConfigurationToJson(ins.motionAxisConfiguration(), axisCfg);
			j["axisConfiguration"] = axisCfg;
			j["axisConfig"] = ins.axisConfig();
		}
		if (const int pointIndex = motionPointIndex(ins); pointIndex > 0)
		{
			j["pointIndex"] = pointIndex;
		}
		break;
	case Type::ARC:
		j["pose"] = vec3ToJson(ins.pose());
		j["eulerDeg"] = vec3ToJson(ins.eulerDeg());
		j["viaPose"] = vec3ToJson(ins.viaPose());
		j["viaEulerDeg"] = vec3ToJson(ins.viaEulerDeg());
		j["speed"] = ins.speed();
		j["accel"] = ins.accel();
		j["blendRadius"] = ins.blendRadius();
		if (ins.hasMotionAxisConfigurationProperty())
		{
			nlohmann::json axisCfg = nlohmann::json::object();
			writeMotionAxisConfigurationToJson(ins.motionAxisConfiguration(), axisCfg);
			j["axisConfiguration"] = axisCfg;
			j["axisConfig"] = ins.axisConfig();
		}
		if (const int pointIndex = motionPointIndex(ins); pointIndex > 0)
		{
			j["pointIndex"] = pointIndex;
		}
		break;
	case Type::WAIT:
		j["durationSec"] = ins.durationSec();
		break;
	case Type::IF:
	{
		j["condition"] = conditionToJson(ins.condition());
		nlohmann::json thenArr = nlohmann::json::array();
		for (const auto& step : ins.nestedSteps())
		{
			if (step)
			{
				thenArr.push_back(toJson(*step));
			}
		}
		j["then"] = thenArr;
		nlohmann::json elseArr = nlohmann::json::array();
		for (const auto& step : ins.elseSteps())
		{
			if (step)
			{
				elseArr.push_back(toJson(*step));
			}
		}
		j["else"] = elseArr;
		break;
	}
	case Type::WHILE:
	{
		j["condition"] = conditionToJson(ins.condition());
		nlohmann::json bodyArr = nlohmann::json::array();
		for (const auto& step : ins.nestedSteps())
		{
			if (step)
			{
				bodyArr.push_back(toJson(*step));
			}
		}
		j["body"] = bodyArr;
		break;
	}
	case Type::SET_DO:
		j["port"] = ins.ioPort();
		j["value"] = ins.ioBoolValue();
		break;
	case Type::SET_AO:
		j["port"] = ins.ioPort();
		j["value"] = ins.ioAnalogValue();
		break;
	case Type::PathPlan:
	{
		const PathPlanInstruction* pp = asPathPlan(ins);
		if (!pp)
		{
			break;
		}
		j["phase"] = pathPlanPhaseToString(pp->phase());
		j["outputGroupId"] = pp->outputGroupId();
		j["rawTrajectoryKey"] = pp->rawTrajectoryKey();
		j["rawRevision"] = pp->rawRevision();
		if (!pp->sourceFeatureJson().empty())
		{
			const nlohmann::json feat = nlohmann::json::parse(pp->sourceFeatureJson(), nullptr, false);
			if (!feat.is_discarded())
			{
				j["sourceFeature"] = feat;
			}
		}
		j["pipeline"] = trajectory_algo::pipelineToJson(pp->pipeline());
		j["appliedHistory"] = trajectory_algo::pipelineToJson(pp->appliedHistory());
		break;
	}
	default:
		break;
	}
	return j;
}

std::vector<std::shared_ptr<Base>> createListFromJson(const nlohmann::json& arr, std::string* errMsg)
{
	std::vector<std::shared_ptr<Base>> out;
	if (!arr.is_array())
	{
		if (errMsg)
		{
			*errMsg = "Expected JSON array of instructions";
		}
		return out;
	}
	for (const auto& item : arr)
	{
		std::string itemErr;
		auto ins = createFromJson(item, &itemErr);
		if (!ins)
		{
			if (errMsg)
			{
				*errMsg = itemErr;
			}
			out.clear();
			return out;
		}
		out.push_back(std::move(ins));
	}
	return out;
}

std::shared_ptr<Base> cloneInstruction(const Base& ins)
{
	const nlohmann::json j = toJson(ins);
	std::string err;
	auto cloned = createFromJson(j, &err);
	if (!cloned)
	{
		return nullptr;
	}
	cloned->setId(makeInstructionId());
	return cloned;
}

std::shared_ptr<Base> cloneInstructionPreservingId(const Base& ins)
{
	const nlohmann::json j = toJson(ins);
	std::string err;
	auto cloned = createFromJson(j, &err);
	if (!cloned)
	{
		return nullptr;
	}
	cloned->setId(ins.id());
	return cloned;
}

} // namespace RobotInstruction
