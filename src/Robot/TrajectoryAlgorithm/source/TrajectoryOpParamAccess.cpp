/// @file TrajectoryOpParamAccess.cpp
/// @brief TrajectoryOpParamAccess 实现

#include "TrajectoryOpParamAccess.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpParamsParse.h"

namespace trajectory_algo
{
namespace
{
bool endsWith(const std::string& text, const std::string& suffix)
{
	return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool readScopeKind(const RobotInstruction::TrajectoryOpDescriptor& op, TrajectoryParamValue& out)
{
	out.kind = TrajectoryParamValue::Kind::Int;
	out.asInt = static_cast<int>(op.scope.kind);
	return true;
}

bool writeScopeKind(RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryParamValue& in)
{
	op.scope.kind = static_cast<RobotInstruction::OpScope::Kind>(in.asInt);
	return true;
}

bool readScopeField(const RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryOpParamField& field,
					TrajectoryParamValue& out)
{
	if (field.key == "scope.kind")
	{
		return readScopeKind(op, out);
	}
	if (field.key == "scope.groupId")
	{
		out.kind = TrajectoryParamValue::Kind::String;
		out.asString = op.scope.groupId;
		return true;
	}
	if (field.key == "scope.pointFrom")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.scope.pointFrom;
		return true;
	}
	if (field.key == "scope.pointTo")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.scope.pointTo;
		return true;
	}
	return false;
}

bool writeScopeField(RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryOpParamField& field,
					 const TrajectoryParamValue& in)
{
	if (field.key == "scope.kind")
	{
		return writeScopeKind(op, in);
	}
	if (field.key == "scope.groupId")
	{
		op.scope.groupId = in.asString;
		return true;
	}
	if (field.key == "scope.pointFrom")
	{
		op.scope.pointFrom = in.asInt;
		return true;
	}
	if (field.key == "scope.pointTo")
	{
		op.scope.pointTo = in.asInt;
		return true;
	}
	return false;
}

bool readJsonParamField(const nlohmann::json& params, const TrajectoryOpParamField& field, TrajectoryParamValue& out)
{
	if (field.type == TrajectoryParamType::Vec3)
	{
		return false;
	}

	if (!params.contains(field.key))
	{
		return false;
	}
	const nlohmann::json& item = params.at(field.key);
	switch (field.type)
	{
	case TrajectoryParamType::Double:
		if (!item.is_number())
		{
			return false;
		}
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = item.get<double>();
		return true;
	case TrajectoryParamType::Int:
	case TrajectoryParamType::Enum:
		if (!item.is_number_integer() && !item.is_number())
		{
			return false;
		}
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = item.get<int>();
		return true;
	case TrajectoryParamType::Bool:
		if (!item.is_boolean())
		{
			return false;
		}
		out.kind = TrajectoryParamValue::Kind::Bool;
		out.asBool = item.get<bool>();
		return true;
	case TrajectoryParamType::Message:
		// 转换工件 externalTcpBackendId 等：Message 在 UI 上承载字符串 backendId
		if (item.is_string())
		{
			out.kind = TrajectoryParamValue::Kind::String;
			out.asString = item.get<std::string>();
			return true;
		}
		return false;
	default:
		return false;
	}
}

bool writeJsonParamField(nlohmann::json& params, const TrajectoryOpParamField& field, const TrajectoryParamValue& in)
{
	if (field.type == TrajectoryParamType::Vec3)
	{
		return false;
	}

	switch (field.type)
	{
	case TrajectoryParamType::Double:
		if (in.kind != TrajectoryParamValue::Kind::Double)
		{
			return false;
		}
		setTrajectoryParamDouble(params, field.key.c_str(), in.asDouble);
		return true;
	case TrajectoryParamType::Int:
	case TrajectoryParamType::Enum:
		if (in.kind != TrajectoryParamValue::Kind::Int)
		{
			return false;
		}
		setTrajectoryParamInt(params, field.key.c_str(), in.asInt);
		return true;
	case TrajectoryParamType::Bool:
		if (in.kind != TrajectoryParamValue::Kind::Bool)
		{
			return false;
		}
		setTrajectoryParamBool(params, field.key.c_str(), in.asBool);
		return true;
	case TrajectoryParamType::Message:
		if (in.kind != TrajectoryParamValue::Kind::String)
		{
			return false;
		}
		setTrajectoryParamString(params, field.key.c_str(), in.asString);
		return true;
	default:
		return false;
	}
}

bool readExpandedParamField(const nlohmann::json& params, const std::string& key, TrajectoryParamValue& out)
{
	if (!params.contains(key))
	{
		return false;
	}
	const nlohmann::json& item = params.at(key);
	if (item.is_number())
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = item.get<double>();
		return true;
	}
	if (item.is_number_integer())
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = item.get<int>();
		return true;
	}
	if (item.is_boolean())
	{
		out.kind = TrajectoryParamValue::Kind::Bool;
		out.asBool = item.get<bool>();
		return true;
	}
	if (item.is_string())
	{
		out.kind = TrajectoryParamValue::Kind::String;
		out.asString = item.get<std::string>();
		return true;
	}
	return false;
}

bool writeExpandedParamField(nlohmann::json& params, const std::string& key, const TrajectoryParamValue& in)
{
	switch (in.kind)
	{
	case TrajectoryParamValue::Kind::Double:
		setTrajectoryParamDouble(params, key.c_str(), in.asDouble);
		return true;
	case TrajectoryParamValue::Kind::Int:
		setTrajectoryParamInt(params, key.c_str(), in.asInt);
		return true;
	case TrajectoryParamValue::Kind::Bool:
		setTrajectoryParamBool(params, key.c_str(), in.asBool);
		return true;
	case TrajectoryParamValue::Kind::String:
		setTrajectoryParamString(params, key.c_str(), in.asString);
		return true;
	default:
		return false;
	}
}

} // namespace

bool TrajectoryOpParamAccess::read(const RobotInstruction::TrajectoryOpDescriptor& op,
								   const TrajectoryOpParamField& field, TrajectoryParamValue& out)
{
	if (readScopeField(op, field, out))
	{
		return true;
	}
	if (endsWith(field.key, ".x") || endsWith(field.key, ".y") || endsWith(field.key, ".z"))
	{
		return readExpandedParamField(op.params, field.key, out);
	}
	return readJsonParamField(op.params, field, out);
}

bool TrajectoryOpParamAccess::write(RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryOpParamField& field,
									const TrajectoryParamValue& in)
{
	if (writeScopeField(op, field, in))
	{
		return true;
	}
	if (endsWith(field.key, ".x") || endsWith(field.key, ".y") || endsWith(field.key, ".z"))
	{
		return writeExpandedParamField(op.params, field.key, in);
	}
	return writeJsonParamField(op.params, field, in);
}

void TrajectoryOpParamAccess::applyDefaults(RobotInstruction::TrajectoryOpDescriptor& op, const ITrajectoryOp& algo)
{
	if (!op.params.is_object())
	{
		op.params = nlohmann::json::object();
	}
	const std::vector<TrajectoryOpParamField> fields = allFieldsForOp(algo);
	for (const TrajectoryOpParamField& field : fields)
	{
		if (field.type == TrajectoryParamType::Message)
		{
			continue;
		}
		// 作用域由 makeDefaultDescriptor(scope) / JSON.scope 指定；此处若写 defaultInt=Group 会冲掉 P 范围
		if (field.key.size() >= 6 && field.key.compare(0, 6, "scope.") == 0)
		{
			continue;
		}
		if (field.type == TrajectoryParamType::Vec3)
		{
			setTrajectoryParamDouble(op.params, (field.key + field.vec3SuffixX).c_str(), field.defaultDouble);
			setTrajectoryParamDouble(op.params, (field.key + field.vec3SuffixY).c_str(), field.defaultDouble);
			setTrajectoryParamDouble(op.params, (field.key + field.vec3SuffixZ).c_str(), field.defaultDouble);
			continue;
		}
		TrajectoryParamValue value{};
		switch (field.type)
		{
		case TrajectoryParamType::Double:
			value.kind = TrajectoryParamValue::Kind::Double;
			value.asDouble = field.defaultDouble;
			break;
		case TrajectoryParamType::Int:
		case TrajectoryParamType::Enum:
			value.kind = TrajectoryParamValue::Kind::Int;
			value.asInt = field.defaultInt;
			break;
		case TrajectoryParamType::Bool:
			value.kind = TrajectoryParamValue::Kind::Bool;
			value.asBool = field.defaultBool;
			break;
		default:
			continue;
		}
		write(op, field, value);
	}
	finalizeTransformDefaultParams(op);
}

const TrajectoryOpParamField* TrajectoryOpParamAccess::findField(const std::vector<TrajectoryOpParamField>& fields,
																 const std::string& key)
{
	for (const TrajectoryOpParamField& field : fields)
	{
		if (field.key == key)
		{
			return &field;
		}
		const std::string xKey = field.key + field.vec3SuffixX;
		const std::string yKey = field.key + field.vec3SuffixY;
		const std::string zKey = field.key + field.vec3SuffixZ;
		if (key == xKey || key == yKey || key == zKey)
		{
			return &field;
		}
	}
	return nullptr;
}

std::vector<TrajectoryOpParamField> TrajectoryOpParamAccess::allFieldsForOp(const ITrajectoryOp& algo)
{
	return TrajectoryOpConfigRegistry::instance().paramFieldsForOp(algo.kind());
}

} // namespace trajectory_algo
