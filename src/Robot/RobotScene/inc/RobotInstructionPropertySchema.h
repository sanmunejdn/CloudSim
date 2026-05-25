#pragma once


#include "../../Data/PropertyCore/inc/PropertySchema.h"

#include "RobotInstructionAxisConfiguration.h"

#include "RobotInstructionModel.h"


namespace RobotInstruction

{

namespace detail

{

inline property_core::PropertyDescriptor makeEnumDescriptor(

	const char* key,

	const char* label,

	const std::string& defaultValue,

	const std::vector<std::string>& options)

{

	property_core::PropertyDescriptor d;

	d.key = key;

	d.label = label;

	d.type = property_core::PropertyType::Enum;

	d.defaultValue = defaultValue;

	d.constraints.enumConstraint.options = options;

	d.constraints.enumConstraint.allowCustom = false;

	return d;

}

} // namespace detail


inline const property_core::PropertySchema& motionAxisConfigPropertyDescriptors()

{

	using namespace property_core;

	static const std::vector<PropertyDescriptor> axisDescriptors = {

		detail::makeEnumDescriptor(

			"motion.axisConfig.preset",

			"Axis config preset",

			"AUTO",

			motionAxisConfigPresetTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.elbow",

			"Elbow posture",

			"AUTO",

			elbowPostureTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.wrist",

			"Wrist posture",

			"AUTO",

			wristPostureTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.arm",

			"Arm posture",

			"AUTO",

			armPostureTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.turn.j1",

			"J1 turn",

			"AUTO",

			motionAxisTurnTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.turn.j4",

			"J4 turn",

			"AUTO",

			motionAxisTurnTokens()),

		detail::makeEnumDescriptor(

			"motion.axisConfig.turn.j6",

			"J6 turn",

			"AUTO",

			motionAxisTurnTokens()),

	};

	static const PropertySchema schema = []() {

		PropertySchema s;

		s.objectTypeId = "robot_instruction.motion_axis";

		s.schemaVersion = 1;

		s.descriptors = axisDescriptors;

		return s;

	}();

	return schema;

}


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

		};

		for (const PropertyDescriptor& axisDesc : motionAxisConfigPropertyDescriptors().descriptors)

		{

			s.descriptors.push_back(axisDesc);

		}

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

			{ "motion.blendRadius", "Blend Radius (mm)", PropertyType::Double, 0.0 },

		};

		for (const PropertyDescriptor& axisDesc : motionAxisConfigPropertyDescriptors().descriptors)

		{

			s.descriptors.push_back(axisDesc);

		}

		return s;

	}();

	return schema;

}


inline const property_core::PropertySchema& waitInstructionPropertySchema()

{

	using namespace property_core;

	static const PropertySchema schema = []() {

		PropertySchema s;

		s.objectTypeId = "robot_instruction.wait";

		s.schemaVersion = 1;

		s.descriptors = {

			{ "logic.wait.durationSec", "Duration (s)", PropertyType::Double, 1.0 }

		};

		return s;

	}();

	return schema;

}


inline const property_core::PropertySchema& setDoInstructionPropertySchema()

{

	using namespace property_core;

	static const PropertySchema schema = []() {

		PropertySchema s;

		s.objectTypeId = "robot_instruction.set_do";

		s.schemaVersion = 1;

		s.descriptors = {

			{ "logic.io.port", "Port", PropertyType::Double, 0.0 },

			{ "logic.io.digitalValue", "Value (0/1)", PropertyType::Double, 0.0 }

		};

		return s;

	}();

	return schema;

}


inline const property_core::PropertySchema& setAoInstructionPropertySchema()

{

	using namespace property_core;

	static const PropertySchema schema = []() {

		PropertySchema s;

		s.objectTypeId = "robot_instruction.set_ao";

		s.schemaVersion = 1;

		s.descriptors = {

			{ "logic.io.port", "Port", PropertyType::Double, 0.0 },

			{ "logic.io.analogValue", "Analog value", PropertyType::Double, 0.0 }

		};

		return s;

	}();

	return schema;

}


inline const property_core::PropertySchema& schemaForInstructionType(const Type type)

{

	switch (type)

	{

	case Type::LINE:

		return lineInstructionPropertySchema();

	case Type::WAIT:

		return waitInstructionPropertySchema();

	case Type::SET_DO:

		return setDoInstructionPropertySchema();

	case Type::SET_AO:

		return setAoInstructionPropertySchema();

	case Type::PTP:

	default:

		return ptpInstructionPropertySchema();

	}

}


inline const property_core::PropertyDescriptor* findInstructionPropertyDescriptor(const std::string& key)

{

	if (const auto* descriptor = motionAxisConfigPropertyDescriptors().find(key))

	{

		return descriptor;

	}

	if (const auto* descriptor = ptpInstructionPropertySchema().find(key))

	{

		return descriptor;

	}

	if (const auto* descriptor = lineInstructionPropertySchema().find(key))

	{

		return descriptor;

	}

	if (const auto* descriptor = waitInstructionPropertySchema().find(key))

	{

		return descriptor;

	}

	if (const auto* descriptor = setDoInstructionPropertySchema().find(key))

	{

		return descriptor;

	}

	return setAoInstructionPropertySchema().find(key);

}


} // namespace RobotInstruction


