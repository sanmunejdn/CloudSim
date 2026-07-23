/// @file RobotInstructionAttribute.cpp
/// @brief RobotInstructionAttribute 实现

#include "RobotInstructionAttribute.h"

#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionPropertySchema.h"
#include "RobotInstructionTransform.h"

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

bool hasViaPosePropertyFn(const RobotInstruction::Base& cmd)
{
	return cmd.hasViaPoseProperty();
}

RobotInstruction::Vec3 getViaPoseFn(const RobotInstruction::Base& cmd)
{
	return cmd.viaPose();
}

void setViaPoseFn(RobotInstruction::Base& cmd, const RobotInstruction::Vec3& pose)
{
	cmd.setViaPose(pose);
	// 面板改 Via 后丢掉旧 transform，避免规划仍用示教快照
	cmd.eraseExtensionProperty(RobotInstruction::kExtContextViaTransformQuatCsv);
	cmd.eraseExtensionProperty(RobotInstruction::kExtContextViaTransformTransMmCsv);
}

bool hasViaEulerPropertyFn(const RobotInstruction::Base& cmd)
{
	return cmd.hasViaEulerProperty();
}

RobotInstruction::Vec3 getViaEulerFn(const RobotInstruction::Base& cmd)
{
	return cmd.viaEulerDeg();
}

void setViaEulerFn(RobotInstruction::Base& cmd, const RobotInstruction::Vec3& value)
{
	cmd.setViaEulerDeg(value);
	cmd.eraseExtensionProperty(RobotInstruction::kExtContextViaTransformQuatCsv);
	cmd.eraseExtensionProperty(RobotInstruction::kExtContextViaTransformTransMmCsv);
}
} // namespace

namespace RobotInstruction
{
using DoubleScalarAttributeImpl = property_core::PropertyScalarAttribute<Base, double, AttributeBase>;
using EnumAttributeImpl = property_core::PropertyEnumAttribute<Base, AttributeBase>;

PoseAttribute::PoseAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		  hasPoseProperty, getPose, setPose,
		  std::array<const char*, 3>{"motion.target.pose.x", "motion.target.pose.y", "motion.target.pose.z"},
		  std::array<const char*, 3>{labelForKey("motion.target.pose.x", "Target X (mm)"),
									 labelForKey("motion.target.pose.y", "Target Y (mm)"),
									 labelForKey("motion.target.pose.z", "Target Z (mm)")},
		  appendRow)
{
}

EulerAttribute::EulerAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		  hasEulerProperty, getEuler, setEuler,
		  std::array<const char*, 3>{"motion.target.euler.rx", "motion.target.euler.ry", "motion.target.euler.rz"},
		  std::array<const char*, 3>{labelForKey("motion.target.euler.rx", "Euler RX (deg)"),
									 labelForKey("motion.target.euler.ry", "Euler RY (deg)"),
									 labelForKey("motion.target.euler.rz", "Euler RZ (deg)")},
		  appendRow)
{
}

ViaPoseAttribute::ViaPoseAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		  hasViaPosePropertyFn, getViaPoseFn, setViaPoseFn,
		  std::array<const char*, 3>{"motion.via.pose.x", "motion.via.pose.y", "motion.via.pose.z"},
		  std::array<const char*, 3>{labelForKey("motion.via.pose.x", "Via X (mm)"),
									 labelForKey("motion.via.pose.y", "Via Y (mm)"),
									 labelForKey("motion.via.pose.z", "Via Z (mm)")},
		  appendRow)
{
}

ViaEulerAttribute::ViaEulerAttribute()
	: property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>(
		  hasViaEulerPropertyFn, getViaEulerFn, setViaEulerFn,
		  std::array<const char*, 3>{"motion.via.euler.rx", "motion.via.euler.ry", "motion.via.euler.rz"},
		  std::array<const char*, 3>{labelForKey("motion.via.euler.rx", "Via Euler RX (deg)"),
									 labelForKey("motion.via.euler.ry", "Via Euler RY (deg)"),
									 labelForKey("motion.via.euler.rz", "Via Euler RZ (deg)")},
		  appendRow)
{
}

AttributePtr makeScalarDoubleAttribute(bool (*hasProperty)(const Base&), double (*getter)(const Base&),
									   void (*setter)(Base&, const double&), const char* key, const char* label)
{
	return std::make_shared<DoubleScalarAttributeImpl>(hasProperty, getter, setter, key, label, appendRow);
}

AttributePtr makeEnumAttribute(bool (*hasProperty)(const Base&), std::string (*getter)(const Base&),
							   void (*setter)(Base&, const std::string&), const char* key, const char* label,
							   bool (*isValidFn)(const std::string&))
{
	return std::make_shared<EnumAttributeImpl>(hasProperty, getter, setter, key, label, appendRow, isValidFn);
}

AttributePtr makeSpeedAttribute()
{
	return makeScalarDoubleAttribute(hasSpeedProperty, getSpeed, setSpeed, "motion.speed",
									 labelForKey("motion.speed", "Speed"));
}

AttributePtr makeAccelAttribute()
{
	return makeScalarDoubleAttribute(hasAccelProperty, getAccel, setAccel, "motion.acc",
									 labelForKey("motion.acc", "Acceleration"));
}

AttributePtr makeAxisConfigAttribute()
{
	return makeEnumAttribute(hasAxisConfigProperty, getAxisConfig, setAxisConfig, "motion.axisConfig.preset",
							 labelForKey("motion.axisConfig.preset", "Axis config preset"));
}

namespace
{
bool hasMotionAxisCfg(const Base& cmd)
{
	return cmd.hasMotionAxisConfigurationProperty();
}

std::string getPreset(const Base& cmd)
{
	return cmd.motionAxisConfiguration().preset;
}

void setPreset(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	std::string preset;
	if (!motionAxisConfigPresetFromToken(value, preset))
	{
		return;
	}
	applyPresetToConfiguration(preset, cfg);
	cfg.preset = preset;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getElbow(const Base& cmd)
{
	return elbowPostureToToken(cmd.motionAxisConfiguration().elbow);
}

void setElbow(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	ElbowPosture e{};
	if (!elbowPostureFromToken(value, e))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.elbow = e;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getWrist(const Base& cmd)
{
	return wristPostureToToken(cmd.motionAxisConfiguration().wrist);
}

void setWrist(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	WristPosture w{};
	if (!wristPostureFromToken(value, w))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.wrist = w;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getArm(const Base& cmd)
{
	return armPostureToToken(cmd.motionAxisConfiguration().arm);
}

void setArm(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	ArmPosture a{};
	if (!armPostureFromToken(value, a))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.arm = a;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getTurnJ1(const Base& cmd)
{
	return jointTurnToToken(cmd.motionAxisConfiguration().turnJ1);
}

void setTurnJ1(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	int t = kMotionAxisTurnAuto;
	if (!jointTurnFromToken(value, t))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ1 = t;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getTurnJ4(const Base& cmd)
{
	return jointTurnToToken(cmd.motionAxisConfiguration().turnJ4);
}

void setTurnJ4(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	int t = kMotionAxisTurnAuto;
	if (!jointTurnFromToken(value, t))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ4 = t;
	cmd.setMotionAxisConfiguration(cfg);
}

std::string getTurnJ6(const Base& cmd)
{
	return jointTurnToToken(cmd.motionAxisConfiguration().turnJ6);
}

void setTurnJ6(Base& cmd, const std::string& value)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return;
	}
	int t = kMotionAxisTurnAuto;
	if (!jointTurnFromToken(value, t))
	{
		return;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ6 = t;
	cmd.setMotionAxisConfiguration(cfg);
}
} // namespace

std::vector<AttributePtr> makeMotionAxisConfigAttributes()
{
	std::vector<AttributePtr> attrs;
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getPreset, setPreset, "motion.axisConfig.preset",
									  labelForKey("motion.axisConfig.preset", "Axis config preset"),
									  isValidMotionAxisConfigPreset));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getElbow, setElbow, "motion.axisConfig.elbow",
									  labelForKey("motion.axisConfig.elbow", "Elbow posture"),
									  isValidElbowPostureToken));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getWrist, setWrist, "motion.axisConfig.wrist",
									  labelForKey("motion.axisConfig.wrist", "Wrist posture"),
									  isValidWristPostureToken));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getArm, setArm, "motion.axisConfig.arm",
									  labelForKey("motion.axisConfig.arm", "Arm posture"), isValidArmPostureToken));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getTurnJ1, setTurnJ1, "motion.axisConfig.turn.j1",
									  labelForKey("motion.axisConfig.turn.j1", "J1 turn"), isValidMotionAxisTurnToken));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getTurnJ4, setTurnJ4, "motion.axisConfig.turn.j4",
									  labelForKey("motion.axisConfig.turn.j4", "J4 turn"), isValidMotionAxisTurnToken));
	attrs.push_back(makeEnumAttribute(hasMotionAxisCfg, getTurnJ6, setTurnJ6, "motion.axisConfig.turn.j6",
									  labelForKey("motion.axisConfig.turn.j6", "J6 turn"), isValidMotionAxisTurnToken));
	return attrs;
}

AttributePtr makeBlendRadiusAttribute()
{
	return makeScalarDoubleAttribute(hasBlendRadiusProperty, getBlendRadius, setBlendRadius, "motion.blendRadius",
									 labelForKey("motion.blendRadius", "Blend Radius (mm)"));
}
} // namespace RobotInstruction
