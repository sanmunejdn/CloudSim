#include "RobotInstructionAttribute.h"

#include "RobotInstructionModel.h"
#include "RobotInstructionPropertySchema.h"

#include <array>

namespace
{
void appendRow(nlohmann::json& rows, const char* key, const char* label, bool editable, const std::string& value)
{
	nlohmann::json row;
	row["key"] = key;
	row["label"] = label;
	row["editable"] = editable;
	row["value"] = value;
	rows.push_back(std::move(row));
}

const char* labelForKey(const char* key, const char* fallback)
{
	const property_core::PropertyDescriptor* descriptor = RobotInstruction::findInstructionPropertyDescriptor(key);
	return descriptor ? descriptor->label.c_str() : fallback;
}

bool hasPoseProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasPoseProperty();
}

RobotInstruction::Vec3 getPose(const RobotInstruction::Base& cmd)
{
	return cmd.pose();
}

void setPose(RobotInstruction::Base& cmd, const RobotInstruction::Vec3& pose)
{
	cmd.setPose(pose);
}

bool hasEulerProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasEulerProperty();
}

RobotInstruction::Vec3 getEuler(const RobotInstruction::Base& cmd)
{
	return cmd.eulerDeg();
}

void setEuler(RobotInstruction::Base& cmd, const RobotInstruction::Vec3& value)
{
	cmd.setEulerDeg(value);
}

bool hasSpeedProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasSpeedProperty();
}

double getSpeed(const RobotInstruction::Base& cmd)
{
	return cmd.speed();
}

void setSpeed(RobotInstruction::Base& cmd, const double& value)
{
	cmd.setSpeed(value);
}

bool hasAccelProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasAccelProperty();
}

double getAccel(const RobotInstruction::Base& cmd)
{
	return cmd.accel();
}

void setAccel(RobotInstruction::Base& cmd, const double& value)
{
	cmd.setAccel(value);
}

bool hasBlendRadiusProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasBlendRadiusProperty();
}

double getBlendRadius(const RobotInstruction::Base& cmd)
{
	return cmd.blendRadius();
}

void setBlendRadius(RobotInstruction::Base& cmd, const double& value)
{
	cmd.setBlendRadius(value);
}

bool hasAxisConfigProperty(const RobotInstruction::Base& cmd)
{
	return cmd.hasAxisConfigProperty();
}

std::string getAxisConfig(const RobotInstruction::Base& cmd)
{
	return cmd.axisConfig();
}

void setAxisConfig(RobotInstruction::Base& cmd, const std::string& value)
{
	cmd.setAxisConfig(value);
}
} // namespace

namespace RobotInstruction
{
using DoubleScalarAttributeImpl = property_core::PropertyScalarAttribute<Base, double, AttributeBase>;
using EnumAttributeImpl = property_core::PropertyEnumAttribute<Base, AttributeBase>;

PoseAttribute::PoseAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		hasPoseProperty,
		getPose,
		setPose,
		std::array<const char*, 3>{
			"motion.target.pose.x",
			"motion.target.pose.y",
			"motion.target.pose.z"
		},
		std::array<const char*, 3>{
			labelForKey("motion.target.pose.x", "Target X (mm)"),
			labelForKey("motion.target.pose.y", "Target Y (mm)"),
			labelForKey("motion.target.pose.z", "Target Z (mm)")
		},
		appendRow)
{
}

EulerAttribute::EulerAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		hasEulerProperty,
		getEuler,
		setEuler,
		std::array<const char*, 3>{
			"motion.target.euler.rx",
			"motion.target.euler.ry",
			"motion.target.euler.rz"
		},
		std::array<const char*, 3>{
			labelForKey("motion.target.euler.rx", "Euler RX (deg)"),
			labelForKey("motion.target.euler.ry", "Euler RY (deg)"),
			labelForKey("motion.target.euler.rz", "Euler RZ (deg)")
		},
		appendRow)
{
}

AttributePtr makeScalarDoubleAttribute(
	bool (*hasProperty)(const Base&),
	double (*getter)(const Base&),
	void (*setter)(Base&, const double&),
	const char* key,
	const char* label)
{
	return std::make_shared<DoubleScalarAttributeImpl>(
		hasProperty,
		getter,
		setter,
		key,
		label,
		appendRow);
}

AttributePtr makeEnumAttribute(
	bool (*hasProperty)(const Base&),
	std::string (*getter)(const Base&),
	void (*setter)(Base&, const std::string&),
	const char* key,
	const char* label)
{
	return std::make_shared<EnumAttributeImpl>(
		hasProperty,
		getter,
		setter,
		key,
		label,
		appendRow);
}

AttributePtr makeSpeedAttribute()
{
	return makeScalarDoubleAttribute(
		hasSpeedProperty,
		getSpeed,
		setSpeed,
		"motion.speed",
		labelForKey("motion.speed", "Speed"));
}

AttributePtr makeAccelAttribute()
{
	return makeScalarDoubleAttribute(
		hasAccelProperty,
		getAccel,
		setAccel,
		"motion.acc",
		labelForKey("motion.acc", "Acceleration"));
}

AttributePtr makeAxisConfigAttribute()
{
	return makeEnumAttribute(
		hasAxisConfigProperty,
		getAxisConfig,
		setAxisConfig,
		"motion.axisConfig",
		labelForKey("motion.axisConfig", "Axis Configuration"));
}

AttributePtr makeBlendRadiusAttribute()
{
	return makeScalarDoubleAttribute(
		hasBlendRadiusProperty,
		getBlendRadius,
		setBlendRadius,
		"motion.blendRadius",
		labelForKey("motion.blendRadius", "Blend Radius (mm)"));
}
} // namespace RobotInstruction
