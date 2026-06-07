#include "TrajectoryOpParamAccess.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpConfigRegistry.h"

namespace trajectory_algo
{
namespace
{

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

bool readScopeField(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
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

bool writeScopeField(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
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

} // namespace

bool TrajectoryOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out)
{
	if (readScopeField(op, field, out))
	{
		return true;
	}
	return TrajectoryOpConfigRegistry::instance().paramRead(op, field, out);
}

bool TrajectoryOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in)
{
	if (writeScopeField(op, field, in))
	{
		return true;
	}
	return TrajectoryOpConfigRegistry::instance().paramWrite(op, field, in);
}

void TrajectoryOpParamAccess::applyDefaults(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const ITrajectoryOp& algo)
{
	const std::vector<TrajectoryOpParamField> fields = allFieldsForOp(algo);
	for (const TrajectoryOpParamField& field : fields)
	{
		if (field.type == TrajectoryParamType::Message)
		{
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
}

const TrajectoryOpParamField* TrajectoryOpParamAccess::findField(
	const std::vector<TrajectoryOpParamField>& fields,
	const std::string& key)
{
	for (const TrajectoryOpParamField& field : fields)
	{
		if (field.key == key)
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
