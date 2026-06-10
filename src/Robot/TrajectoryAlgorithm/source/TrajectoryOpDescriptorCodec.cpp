#include "TrajectoryOpDescriptorCodec.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpRegistry.h"

#include <json.hpp>

namespace trajectory_algo
{
namespace
{

std::string kindToString(const RobotInstruction::TrajectoryOpKind kind)
{
	switch (kind)
	{
	case RobotInstruction::TrajectoryOpKind::Translate:
		return "Translate";
	case RobotInstruction::TrajectoryOpKind::Rotate:
		return "Rotate";
	case RobotInstruction::TrajectoryOpKind::Mirror:
		return "Mirror";
	case RobotInstruction::TrajectoryOpKind::Delete:
		return "Delete";
	case RobotInstruction::TrajectoryOpKind::Duplicate:
		return "Duplicate";
	case RobotInstruction::TrajectoryOpKind::Reorder:
		return "Reorder";
	case RobotInstruction::TrajectoryOpKind::Approach:
		return "Approach";
	case RobotInstruction::TrajectoryOpKind::Retract:
		return "Retract";
	case RobotInstruction::TrajectoryOpKind::Resample:
		return "Resample";
	case RobotInstruction::TrajectoryOpKind::OffsetAlongNormal:
		return "OffsetAlongNormal";
	case RobotInstruction::TrajectoryOpKind::OffsetLateral:
		return "OffsetLateral";
	case RobotInstruction::TrajectoryOpKind::SmoothPose:
		return "SmoothPose";
	case RobotInstruction::TrajectoryOpKind::AssignBlend:
		return "AssignBlend";
	case RobotInstruction::TrajectoryOpKind::AssignSpeedZone:
		return "AssignSpeedZone";
	case RobotInstruction::TrajectoryOpKind::Weave:
		return "Weave";
	case RobotInstruction::TrajectoryOpKind::ReachabilityFilter:
		return "ReachabilityFilter";
	case RobotInstruction::TrajectoryOpKind::ExternalAxisSearch:
		return "ExternalAxisSearch";
	case RobotInstruction::TrajectoryOpKind::ProjectToGeometry:
		return "ProjectToGeometry";
	default:
		return "Translate";
	}
}

bool kindFromString(const std::string& s, RobotInstruction::TrajectoryOpKind& out)
{
	if (s == "Translate")
	{
		out = RobotInstruction::TrajectoryOpKind::Translate;
		return true;
	}
	if (s == "Rotate")
	{
		out = RobotInstruction::TrajectoryOpKind::Rotate;
		return true;
	}
	if (s == "Mirror")
	{
		out = RobotInstruction::TrajectoryOpKind::Mirror;
		return true;
	}
	if (s == "Delete")
	{
		out = RobotInstruction::TrajectoryOpKind::Delete;
		return true;
	}
	if (s == "Duplicate")
	{
		out = RobotInstruction::TrajectoryOpKind::Duplicate;
		return true;
	}
	if (s == "Reorder")
	{
		out = RobotInstruction::TrajectoryOpKind::Reorder;
		return true;
	}
	if (s == "Approach")
	{
		out = RobotInstruction::TrajectoryOpKind::Approach;
		return true;
	}
	if (s == "Retract")
	{
		out = RobotInstruction::TrajectoryOpKind::Retract;
		return true;
	}
	if (s == "Resample")
	{
		out = RobotInstruction::TrajectoryOpKind::Resample;
		return true;
	}
	if (s == "OffsetAlongNormal")
	{
		out = RobotInstruction::TrajectoryOpKind::OffsetAlongNormal;
		return true;
	}
	if (s == "OffsetLateral")
	{
		out = RobotInstruction::TrajectoryOpKind::OffsetLateral;
		return true;
	}
	if (s == "SmoothPose")
	{
		out = RobotInstruction::TrajectoryOpKind::SmoothPose;
		return true;
	}
	if (s == "AssignBlend")
	{
		out = RobotInstruction::TrajectoryOpKind::AssignBlend;
		return true;
	}
	if (s == "AssignSpeedZone")
	{
		out = RobotInstruction::TrajectoryOpKind::AssignSpeedZone;
		return true;
	}
	if (s == "Weave")
	{
		out = RobotInstruction::TrajectoryOpKind::Weave;
		return true;
	}
	if (s == "ReachabilityFilter")
	{
		out = RobotInstruction::TrajectoryOpKind::ReachabilityFilter;
		return true;
	}
	if (s == "ExternalAxisSearch")
	{
		out = RobotInstruction::TrajectoryOpKind::ExternalAxisSearch;
		return true;
	}
	if (s == "ProjectToGeometry")
	{
		out = RobotInstruction::TrajectoryOpKind::ProjectToGeometry;
		return true;
	}
	return false;
}

RobotInstruction::TrajectoryOpDescriptor makeLegacyOp(const RobotInstruction::TrajectoryOpKind kind)
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = kind;
	op.scope.kind = RobotInstruction::OpScope::Kind::Group;
	return op;
}

bool expandLegacyRecipeKind(
	const std::string& kindToken,
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& out,
	std::string* errMsg)
{
	(void)errMsg;
	if (kindToken == "RecipeWeld")
	{
		auto resample = makeLegacyOp(RobotInstruction::TrajectoryOpKind::Resample);
		resample.resample.stepMm = 5.0;
		auto offset = makeLegacyOp(RobotInstruction::TrajectoryOpKind::OffsetAlongNormal);
		auto smooth = makeLegacyOp(RobotInstruction::TrajectoryOpKind::SmoothPose);
		auto blend = makeLegacyOp(RobotInstruction::TrajectoryOpKind::AssignBlend);
		blend.assignMotion.blendRadiusMm = 2.0;
		out = { resample, offset, smooth, blend,
			makeLegacyOp(RobotInstruction::TrajectoryOpKind::Approach),
			makeLegacyOp(RobotInstruction::TrajectoryOpKind::Retract) };
		return true;
	}
	if (kindToken == "RecipeGlue")
	{
		auto resample = makeLegacyOp(RobotInstruction::TrajectoryOpKind::Resample);
		resample.resample.stepMm = 3.0;
		auto offset = makeLegacyOp(RobotInstruction::TrajectoryOpKind::OffsetAlongNormal);
		offset.pathOffset.offsetMm = 1.0;
		out = { resample, offset, makeLegacyOp(RobotInstruction::TrajectoryOpKind::SmoothPose),
			makeLegacyOp(RobotInstruction::TrajectoryOpKind::AssignSpeedZone) };
		return true;
	}
	if (kindToken == "RecipeGrind")
	{
		auto resample = makeLegacyOp(RobotInstruction::TrajectoryOpKind::Resample);
		resample.resample.stepMm = 4.0;
		auto offset = makeLegacyOp(RobotInstruction::TrajectoryOpKind::OffsetAlongNormal);
		auto weave = makeLegacyOp(RobotInstruction::TrajectoryOpKind::Weave);
		out = { resample, offset, makeLegacyOp(RobotInstruction::TrajectoryOpKind::SmoothPose), weave,
			makeLegacyOp(RobotInstruction::TrajectoryOpKind::Approach),
			makeLegacyOp(RobotInstruction::TrajectoryOpKind::Retract) };
		return true;
	}
	return false;
}

void writeScopeJson(const RobotInstruction::OpScope& scope, nlohmann::json& out)
{
	out["kind"] = static_cast<int>(scope.kind);
	out["groupId"] = scope.groupId;
	out["pointFrom"] = scope.pointFrom;
	out["pointTo"] = scope.pointTo;
}

bool readScopeJson(const nlohmann::json& j, RobotInstruction::OpScope& scope)
{
	if (j.contains("kind"))
	{
		scope.kind = static_cast<RobotInstruction::OpScope::Kind>(j["kind"].get<int>());
	}
	if (j.contains("groupId"))
	{
		scope.groupId = j["groupId"].get<std::string>();
	}
	if (j.contains("pointFrom"))
	{
		scope.pointFrom = j["pointFrom"].get<int>();
	}
	if (j.contains("pointTo"))
	{
		scope.pointTo = j["pointTo"].get<int>();
	}
	return true;
}

void copyDescriptorDefaults(
	RobotInstruction::TrajectoryOpDescriptor& out,
	const RobotInstruction::TrajectoryOpDescriptor& defaults)
{
	out.translate = defaults.translate;
	out.rotate = defaults.rotate;
	out.duplicateCount = defaults.duplicateCount;
	out.mirrorAxis = defaults.mirrorAxis;
	out.resample = defaults.resample;
	out.pathOffset = defaults.pathOffset;
	out.weave = defaults.weave;
	out.assignMotion = defaults.assignMotion;
	out.approach = defaults.approach;
	out.retract = defaults.retract;
	out.project = defaults.project;
}

} // namespace

nlohmann::json toJson(const RobotInstruction::TrajectoryOpDescriptor& op)
{
	nlohmann::json j;
	j["kind"] = kindToString(op.kind);
	writeScopeJson(op.scope, j["scope"]);

	nlohmann::json params = nlohmann::json::object();
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(op.kind);
	if (algo)
	{
		const std::vector<TrajectoryOpParamField> fields = TrajectoryOpParamAccess::allFieldsForOp(*algo);
		for (const TrajectoryOpParamField& field : fields)
		{
			if (field.type == TrajectoryParamType::Message)
			{
				continue;
			}
			TrajectoryParamValue value{};
			if (!TrajectoryOpParamAccess::read(op, field, value))
			{
				continue;
			}
			switch (value.kind)
			{
			case TrajectoryParamValue::Kind::Double:
				params[field.key] = value.asDouble;
				break;
			case TrajectoryParamValue::Kind::Int:
				params[field.key] = value.asInt;
				break;
			case TrajectoryParamValue::Kind::Bool:
				params[field.key] = value.asBool;
				break;
			case TrajectoryParamValue::Kind::String:
				params[field.key] = value.asString;
				break;
			}
		}
	}
	j["params"] = params;
	return j;
}

bool fromJson(const nlohmann::json& j, RobotInstruction::TrajectoryOpDescriptor& out, std::string* errMsg)
{
	if (!j.is_object())
	{
		if (errMsg)
		{
			*errMsg = "descriptor is not an object";
		}
		return false;
	}
	RobotInstruction::TrajectoryOpKind kind = RobotInstruction::TrajectoryOpKind::Translate;
	if (j.contains("kind"))
	{
		if (j["kind"].is_string())
		{
			const std::string kindToken = j["kind"].get<std::string>();
			std::vector<RobotInstruction::TrajectoryOpDescriptor> legacyExpanded;
			if (expandLegacyRecipeKind(kindToken, legacyExpanded, errMsg))
			{
				if (legacyExpanded.empty())
				{
					if (errMsg)
					{
						*errMsg = "legacy recipe expansion failed";
					}
					return false;
				}
				out = legacyExpanded.front();
				return true;
			}
			if (!kindFromString(kindToken, kind))
			{
				if (errMsg)
				{
					*errMsg = "unknown kind";
				}
				return false;
			}
		}
		else if (j["kind"].is_number_integer())
		{
			kind = static_cast<RobotInstruction::TrajectoryOpKind>(j["kind"].get<int>());
		}
	}
	out.kind = kind;
	if (j.contains("scope"))
	{
		readScopeJson(j["scope"], out.scope);
	}
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(kind);
	if (!algo)
	{
		if (errMsg)
		{
			*errMsg = "unknown trajectory op kind";
		}
		return false;
	}
	RobotInstruction::TrajectoryOpDescriptor defaults = algo->makeDefaultDescriptor(out.scope);
	copyDescriptorDefaults(out, defaults);

	if (j.contains("params") && j["params"].is_object())
	{
		bool hasTranslateEnd = false;
		bool hasRotateEnd = false;
		const std::vector<TrajectoryOpParamField> fields = TrajectoryOpParamAccess::allFieldsForOp(*algo);
		for (const auto& item : j["params"].items())
		{
			const TrajectoryOpParamField* field =
				TrajectoryOpParamAccess::findField(fields, item.key());
			if (!field)
			{
				continue;
			}
			TrajectoryParamValue value{};
			if (field->type == TrajectoryParamType::Double && item.value().is_number())
			{
				value.kind = TrajectoryParamValue::Kind::Double;
				value.asDouble = item.value().get<double>();
			}
			else if ((field->type == TrajectoryParamType::Int || field->type == TrajectoryParamType::Enum)
				&& item.value().is_number_integer())
			{
				value.kind = TrajectoryParamValue::Kind::Int;
				value.asInt = item.value().get<int>();
			}
			else if (field->type == TrajectoryParamType::Bool && item.value().is_boolean())
			{
				value.kind = TrajectoryParamValue::Kind::Bool;
				value.asBool = item.value().get<bool>();
			}
			else if (field->key == "scope.groupId" && item.value().is_string())
			{
				value.kind = TrajectoryParamValue::Kind::String;
				value.asString = item.value().get<std::string>();
			}
			else
			{
				continue;
			}
			TrajectoryOpParamAccess::write(out, *field, value);
			if (item.key() == "translate.endDxMm" || item.key() == "translate.endDyMm" || item.key() == "translate.endDzMm")
			{
				hasTranslateEnd = true;
			}
			if (item.key() == "rotate.endAngleDeg")
			{
				hasRotateEnd = true;
			}
		}
		if (!hasTranslateEnd)
		{
			out.translate.endDxMm = out.translate.dxMm;
			out.translate.endDyMm = out.translate.dyMm;
			out.translate.endDzMm = out.translate.dzMm;
		}
		if (!hasRotateEnd)
		{
			out.rotate.endAngleDeg = out.rotate.angleDeg;
		}
	}
	return true;
}

nlohmann::json pipelineToJson(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		arr.push_back(toJson(op));
	}
	return arr;
}

bool pipelineFromJson(
	const nlohmann::json& j,
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& out,
	std::string* errMsg)
{
	out.clear();
	if (!j.is_array())
	{
		if (errMsg)
		{
			*errMsg = "pipeline is not an array";
		}
		return false;
	}
	for (const nlohmann::json& item : j)
	{
		if (item.contains("kind") && item["kind"].is_string())
		{
			std::vector<RobotInstruction::TrajectoryOpDescriptor> legacyExpanded;
			if (expandLegacyRecipeKind(item["kind"].get<std::string>(), legacyExpanded, errMsg))
			{
				out.insert(out.end(), legacyExpanded.begin(), legacyExpanded.end());
				continue;
			}
		}
		RobotInstruction::TrajectoryOpDescriptor op{};
		if (!fromJson(item, op, errMsg))
		{
			return false;
		}
		out.push_back(op);
	}
	return true;
}

} // namespace trajectory_algo
