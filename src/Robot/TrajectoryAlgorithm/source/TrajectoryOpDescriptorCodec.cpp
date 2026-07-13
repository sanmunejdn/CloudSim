#include "TrajectoryOpDescriptorCodec.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "TrajectoryOpRegistry.h"

#include <json.hpp>

namespace trajectory_algo
{
namespace
{

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

bool mergeParamsJson(
	RobotInstruction::TrajectoryOpDescriptor& out,
	const ITrajectoryOp& algo,
	const nlohmann::json& paramsJson)
{
	if (!paramsJson.is_object())
	{
		return true;
	}
	const std::vector<TrajectoryOpParamField> fields = TrajectoryOpParamAccess::allFieldsForOp(algo);
	for (const auto& item : paramsJson.items())
	{
		const TrajectoryOpParamField* field = TrajectoryOpParamAccess::findField(fields, item.key());
		if (!field)
		{
			out.params[item.key()] = item.value();
			continue;
		}
		TrajectoryParamValue value{};
		if (field->type == TrajectoryParamType::Double && item.value().is_number())
		{
			value.kind = TrajectoryParamValue::Kind::Double;
			value.asDouble = item.value().get<double>();
		}
		else if ((field->type == TrajectoryParamType::Int || field->type == TrajectoryParamType::Enum)
			&& (item.value().is_number_integer() || item.value().is_number()))
		{
			value.kind = TrajectoryParamValue::Kind::Int;
			value.asInt = item.value().get<int>();
		}
		else if (field->type == TrajectoryParamType::Bool && item.value().is_boolean())
		{
			value.kind = TrajectoryParamValue::Kind::Bool;
			value.asBool = item.value().get<bool>();
		}
		else if (item.value().is_string())
		{
			value.kind = TrajectoryParamValue::Kind::String;
			value.asString = item.value().get<std::string>();
		}
		else if (item.value().is_number())
		{
			value.kind = TrajectoryParamValue::Kind::Double;
			value.asDouble = item.value().get<double>();
		}
		else
		{
			out.params[item.key()] = item.value();
			continue;
		}
		TrajectoryOpParamAccess::write(out, *field, value);
	}
	finalizeTransformDefaultParams(out);
	return true;
}

} // namespace

nlohmann::json toJson(const RobotInstruction::TrajectoryOpDescriptor& op)
{
	nlohmann::json j;
	if (!op.opId.empty())
	{
		j["opId"] = op.opId;
	}
	j["kind"] = TrajectoryOpRegistry::instance().kindToString(op.kind);
	writeScopeJson(op.scope, j["scope"]);
	j["params"] = op.params.is_object() ? op.params : nlohmann::json::object();
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
			if (!TrajectoryOpRegistry::instance().kindFromString(kindToken, kind))
			{
				if (errMsg)
				{
					*errMsg = "unknown kind: " + kindToken;
				}
				return false;
			}
		}
		else if (j["kind"].is_number_integer())
		{
			kind = static_cast<RobotInstruction::TrajectoryOpKind>(j["kind"].get<int>());
		}
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

	RobotInstruction::OpScope scope{};
	scope.kind = RobotInstruction::OpScope::Kind::Group;
	if (j.contains("scope"))
	{
		readScopeJson(j["scope"], scope);
	}

	std::string opId;
	if (j.contains("opId") && j["opId"].is_string())
	{
		opId = j["opId"].get<std::string>();
	}

	out = algo->makeDefaultDescriptor(scope);
	if (!opId.empty())
	{
		out.opId = std::move(opId);
	}

	if (j.contains("params"))
	{
		if (!mergeParamsJson(out, *algo, j["params"]))
		{
			if (errMsg)
			{
				*errMsg = "failed to apply params";
			}
			return false;
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
