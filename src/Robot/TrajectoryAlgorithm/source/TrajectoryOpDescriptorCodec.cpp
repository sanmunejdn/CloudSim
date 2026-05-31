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
			if (!kindFromString(j["kind"].get<std::string>(), kind))
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
	out.translate = defaults.translate;
	out.rotate = defaults.rotate;
	out.duplicateCount = defaults.duplicateCount;
	out.mirrorAxis = defaults.mirrorAxis;

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
