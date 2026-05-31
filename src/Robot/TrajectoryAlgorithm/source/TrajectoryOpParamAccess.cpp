#include "TrajectoryOpParamAccess.h"

#include "ITrajectoryOp.h"

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

} // namespace

bool TrajectoryOpParamAccess::read(
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
	if (field.key == "translate.frame")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.translate.frame);
		return true;
	}
	if (field.key == "translate.dxMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.dxMm;
		return true;
	}
	if (field.key == "translate.dyMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.dyMm;
		return true;
	}
	if (field.key == "translate.dzMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.dzMm;
		return true;
	}
	if (field.key == "translate.endDxMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.endDxMm;
		return true;
	}
	if (field.key == "translate.endDyMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.endDyMm;
		return true;
	}
	if (field.key == "translate.endDzMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.translate.endDzMm;
		return true;
	}
	if (field.key == "rotate.frame")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.rotate.frame);
		return true;
	}
	if (field.key == "rotate.axisX")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisX;
		return true;
	}
	if (field.key == "rotate.axisY")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisY;
		return true;
	}
	if (field.key == "rotate.axisZ")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisZ;
		return true;
	}
	if (field.key == "rotate.angleDeg")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.angleDeg;
		return true;
	}
	if (field.key == "rotate.endAngleDeg")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.endAngleDeg;
		return true;
	}
	if (field.key == "mirror.axis")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.mirrorAxis;
		return true;
	}
	if (field.key == "structural.duplicateCount")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.duplicateCount;
		return true;
	}
	return false;
}

bool TrajectoryOpParamAccess::write(
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
	if (field.key == "translate.frame")
	{
		op.translate.frame = static_cast<RobotInstruction::TransformReferenceFrame>(in.asInt);
		return true;
	}
	if (field.key == "translate.dxMm")
	{
		op.translate.dxMm = in.asDouble;
		return true;
	}
	if (field.key == "translate.dyMm")
	{
		op.translate.dyMm = in.asDouble;
		return true;
	}
	if (field.key == "translate.dzMm")
	{
		op.translate.dzMm = in.asDouble;
		return true;
	}
	if (field.key == "translate.endDxMm")
	{
		op.translate.endDxMm = in.asDouble;
		return true;
	}
	if (field.key == "translate.endDyMm")
	{
		op.translate.endDyMm = in.asDouble;
		return true;
	}
	if (field.key == "translate.endDzMm")
	{
		op.translate.endDzMm = in.asDouble;
		return true;
	}
	if (field.key == "rotate.frame")
	{
		op.rotate.frame = static_cast<RobotInstruction::TransformReferenceFrame>(in.asInt);
		return true;
	}
	if (field.key == "rotate.axisX")
	{
		op.rotate.axisX = in.asDouble;
		return true;
	}
	if (field.key == "rotate.axisY")
	{
		op.rotate.axisY = in.asDouble;
		return true;
	}
	if (field.key == "rotate.axisZ")
	{
		op.rotate.axisZ = in.asDouble;
		return true;
	}
	if (field.key == "rotate.angleDeg")
	{
		op.rotate.angleDeg = in.asDouble;
		return true;
	}
	if (field.key == "rotate.endAngleDeg")
	{
		op.rotate.endAngleDeg = in.asDouble;
		return true;
	}
	if (field.key == "mirror.axis")
	{
		op.mirrorAxis = in.asInt;
		return true;
	}
	if (field.key == "structural.duplicateCount")
	{
		op.duplicateCount = in.asInt;
		return true;
	}
	return false;
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
	std::vector<TrajectoryOpParamField> fields = trajectoryOpCommonScopeFields();
	const std::vector<TrajectoryOpParamField> algoFields = algo.paramFields();
	fields.insert(fields.end(), algoFields.begin(), algoFields.end());
	return fields;
}

} // namespace trajectory_algo
