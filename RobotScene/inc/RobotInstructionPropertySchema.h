#pragma once

#include "../../PropertyCore/inc/PropertySchema.h"
#include "RobotInstructionModel.h"

namespace RobotInstruction
{

inline const property_core::PropertySchema& ptpInstructionPropertySchema()
{
	using namespace property_core;
	static const PropertySchema schema = []() {
		PropertySchema s;
		s.objectTypeId = "robot_instruction.ptp";
		s.schemaVersion = 1;
		s.descriptors = {
			{ "motion.target.pose.x", "Target X (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.pose.y", "Target Y (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.pose.z", "Target Z (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.rx", "Euler RX (deg)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.ry", "Euler RY (deg)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.rz", "Euler RZ (deg)", PropertyType::Double, 0.0 },
			{ "motion.speed", "Speed", PropertyType::Double, 100.0 },
			{ "motion.acc", "Acceleration", PropertyType::Double, 100.0 },
			{ "motion.axisConfig", "Axis Configuration", PropertyType::Enum, std::string("AUTO") }
		};
		s.descriptors.back().constraints.enumConstraint.allowCustom = true;
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& lineInstructionPropertySchema()
{
	using namespace property_core;
	static const PropertySchema schema = []() {
		PropertySchema s;
		s.objectTypeId = "robot_instruction.line";
		s.schemaVersion = 1;
		s.descriptors = {
			{ "motion.target.pose.x", "Target X (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.pose.y", "Target Y (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.pose.z", "Target Z (mm)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.rx", "Euler RX (deg)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.ry", "Euler RY (deg)", PropertyType::Double, 0.0 },
			{ "motion.target.euler.rz", "Euler RZ (deg)", PropertyType::Double, 0.0 },
			{ "motion.speed", "Speed", PropertyType::Double, 200.0 },
			{ "motion.acc", "Acceleration", PropertyType::Double, 200.0 },
			{ "motion.blendRadius", "Blend Radius (mm)", PropertyType::Double, 0.0 }
		};
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& schemaForInstructionType(const Type type)
{
	return type == Type::LINE ? lineInstructionPropertySchema() : ptpInstructionPropertySchema();
}

inline const property_core::PropertyDescriptor* findInstructionPropertyDescriptor(const std::string& key)
{
	if (const auto* descriptor = ptpInstructionPropertySchema().find(key))
	{
		return descriptor;
	}
	return lineInstructionPropertySchema().find(key);
}

} // namespace RobotInstruction
