// Translate 块参数字段与 descriptor 读写
#include "TranslateOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool TranslateOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("translate.", 0) == 0;
}

bool TranslateOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
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
	return false;
}

bool TranslateOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
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
	return false;
}

std::unique_ptr<IOpParamAccess> makeTranslateOpParamAccess()
{
	return std::make_unique<TranslateOpParamAccess>();
}

} // namespace trajectory_algo
