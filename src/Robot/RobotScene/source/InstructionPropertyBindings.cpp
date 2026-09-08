/// @file InstructionPropertyBindings.cpp
/// @brief 指令属性 Binding 包：snapshot / schema / apply

#include "InstructionPropertyBinding.h"
#include "InstructionPropertyBindingSelfTest.h"

#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionTransform.h"
#include "RunLogger.h"

#include "../../Data/PropertyCore/inc/PropertyAttributeHelpers.h"
#include "../../Data/PropertyCore/inc/PropertyTypes.h"

#include <memory>
#include <mutex>
#include <unordered_map>

namespace
{
using property_core::PropertyDescriptor;
using property_core::PropertySemanticFlags;
using property_core::PropertyType;
using RobotInstruction::Base;
using RobotInstruction::Condition;
using RobotInstruction::ConditionKind;
using RobotInstruction::InstructionPropertyBinding;
using RobotInstruction::MotionAxisConfiguration;
using RobotInstruction::Type;

constexpr PropertySemanticFlags kMotion = PropertySemanticFlags::AffectsInstructionMotion;

PropertyDescriptor makeDesc(const char* key, const char* label, PropertyType type, bool editable,
							PropertySemanticFlags flags = PropertySemanticFlags::None)
{
	PropertyDescriptor d;
	d.key = key;
	d.label = label;
	d.type = type;
	d.editable = editable;
	d.semanticFlags = flags;
	return d;
}

PropertyDescriptor makeEnumDesc(const char* key, const char* label, const char* defaultValue,
								const std::vector<std::string>& options, bool editable = true,
								PropertySemanticFlags flags = PropertySemanticFlags::None)
{
	PropertyDescriptor d = makeDesc(key, label, PropertyType::Enum, editable, flags);
	d.defaultValue = std::string(defaultValue);
	d.constraints.enumConstraint.options = options;
	d.constraints.enumConstraint.allowCustom = false;
	return d;
}

std::shared_ptr<Base> makeEmptyOfType(const Type type)
{
	switch (type)
	{
	case Type::PTP:
		return std::make_shared<RobotInstruction::PtpInstruction>();
	case Type::LINE:
		return std::make_shared<RobotInstruction::LineInstruction>();
	case Type::ARC:
		return std::make_shared<RobotInstruction::ArcInstruction>();
	case Type::WAIT:
		return std::make_shared<RobotInstruction::WaitInstruction>();
	case Type::IF:
		return std::make_shared<RobotInstruction::IfInstruction>();
	case Type::WHILE:
		return std::make_shared<RobotInstruction::WhileInstruction>();
	case Type::SET_DO:
		return std::make_shared<RobotInstruction::SetDigitalOutputInstruction>();
	case Type::SET_AO:
		return std::make_shared<RobotInstruction::SetAnalogOutputInstruction>();
	case Type::DeviceAxis:
		return std::make_shared<RobotInstruction::DeviceAxisInstruction>();
	case Type::PathPlan:
		return std::make_shared<RobotInstruction::PathPlanInstruction>();
	default:
		return std::make_shared<RobotInstruction::PtpInstruction>();
	}
}

const char* objectTypeIdFor(const Type type)
{
	switch (type)
	{
	case Type::LINE:
		return "robot_instruction.line";
	case Type::ARC:
		return "robot_instruction.arc";
	case Type::WAIT:
		return "robot_instruction.wait";
	case Type::IF:
		return "robot_instruction.if";
	case Type::WHILE:
		return "robot_instruction.while";
	case Type::SET_DO:
		return "robot_instruction.set_do";
	case Type::SET_AO:
		return "robot_instruction.set_ao";
	case Type::DeviceAxis:
		return "robot_instruction.device_axis";
	case Type::PathPlan:
		return "robot_instruction.path_plan";
	case Type::PTP:
	default:
		return "robot_instruction.ptp";
	}
}

bool isActiveHasPose(const Base& cmd)
{
	return cmd.hasPoseProperty();
}
bool isActiveHasEuler(const Base& cmd)
{
	return cmd.hasEulerProperty();
}
bool isActiveHasViaPose(const Base& cmd)
{
	return cmd.hasViaPoseProperty();
}
bool isActiveHasViaEuler(const Base& cmd)
{
	return cmd.hasViaEulerProperty();
}
bool isActiveHasSpeed(const Base& cmd)
{
	return cmd.hasSpeedProperty();
}
bool isActiveHasAccel(const Base& cmd)
{
	return cmd.hasAccelProperty();
}
bool isActiveHasBlend(const Base& cmd)
{
	return cmd.hasBlendRadiusProperty();
}
bool isActiveHasMotionAxisCfg(const Base& cmd)
{
	return cmd.hasMotionAxisConfigurationProperty();
}
bool isActiveHasCondition(const Base& cmd)
{
	return cmd.hasConditionProperty();
}
bool isActiveConditionIo(const Base& cmd)
{
	return cmd.hasConditionProperty() && cmd.condition().kind == ConditionKind::Io;
}
bool isActiveWaitDuration(const Base& cmd)
{
	return cmd.type() == Type::WAIT && cmd.hasDurationProperty();
}
bool isActiveDeviceDuration(const Base& cmd)
{
	return cmd.type() == Type::DeviceAxis && cmd.hasDurationProperty();
}
bool isActiveHasIoSignal(const Base& cmd)
{
	return cmd.hasIoSignalNameProperty();
}
bool isActiveHasIoPort(const Base& cmd)
{
	return cmd.hasIoPortProperty();
}
bool isActiveHasIoDigital(const Base& cmd)
{
	return cmd.hasIoValueProperty();
}
bool isActiveSetAoAnalog(const Base& cmd)
{
	return cmd.type() == Type::SET_AO;
}
bool isActiveHasDeviceAxis(const Base& cmd)
{
	return cmd.hasDeviceAxisProperty();
}
bool isActivePathPlan(const Base& cmd)
{
	return cmd.type() == Type::PathPlan;
}

std::string formatTargetPoseX(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.pose().x);
}
std::string formatTargetPoseY(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.pose().y);
}
std::string formatTargetPoseZ(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.pose().z);
}
std::string formatTargetEulerRx(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.eulerDeg().x);
}
std::string formatTargetEulerRy(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.eulerDeg().y);
}
std::string formatTargetEulerRz(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.eulerDeg().z);
}

bool applyTargetPoseX(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.pose.x", value, err);
}
bool applyTargetPoseY(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.pose.y", value, err);
}
bool applyTargetPoseZ(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.pose.z", value, err);
}
bool applyTargetEulerRx(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.euler.rx", value, err);
}
bool applyTargetEulerRy(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.euler.ry", value, err);
}
bool applyTargetEulerRz(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyTargetDisplayComponent(cmd, "motion.target.euler.rz", value, err);
}

std::string formatViaPoseX(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaPose().x);
}
std::string formatViaPoseY(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaPose().y);
}
std::string formatViaPoseZ(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaPose().z);
}
std::string formatViaEulerRx(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaEulerDeg().x);
}
std::string formatViaEulerRy(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaEulerDeg().y);
}
std::string formatViaEulerRz(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.viaEulerDeg().z);
}

bool applyViaPoseX(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.pose.x", value, err);
}
bool applyViaPoseY(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.pose.y", value, err);
}
bool applyViaPoseZ(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.pose.z", value, err);
}
bool applyViaEulerRx(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.euler.rx", value, err);
}
bool applyViaEulerRy(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.euler.ry", value, err);
}
bool applyViaEulerRz(Base& cmd, const std::string& value, std::string* err)
{
	return RobotInstruction::applyViaDisplayComponent(cmd, "motion.via.euler.rz", value, err);
}

std::string formatSpeed(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.speed());
}
std::string formatAccel(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.accel());
}
std::string formatBlendRadius(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.blendRadius());
}
std::string formatDurationSec(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.durationSec());
}

bool applySpeed(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setSpeed(v);
	return true;
}
bool applyAccel(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setAccel(v);
	return true;
}
bool applyBlendRadius(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setBlendRadius(v);
	return true;
}
bool applyDurationSec(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setDurationSec(v);
	return true;
}

std::string formatAxisPreset(const Base& cmd)
{
	return cmd.motionAxisConfiguration().preset;
}
std::string formatAxisElbow(const Base& cmd)
{
	return RobotInstruction::elbowPostureToToken(cmd.motionAxisConfiguration().elbow);
}
std::string formatAxisWrist(const Base& cmd)
{
	return RobotInstruction::wristPostureToToken(cmd.motionAxisConfiguration().wrist);
}
std::string formatAxisArm(const Base& cmd)
{
	return RobotInstruction::armPostureToToken(cmd.motionAxisConfiguration().arm);
}
std::string formatAxisTurnJ1(const Base& cmd)
{
	return RobotInstruction::jointTurnToToken(cmd.motionAxisConfiguration().turnJ1);
}
std::string formatAxisTurnJ4(const Base& cmd)
{
	return RobotInstruction::jointTurnToToken(cmd.motionAxisConfiguration().turnJ4);
}
std::string formatAxisTurnJ6(const Base& cmd)
{
	return RobotInstruction::jointTurnToToken(cmd.motionAxisConfiguration().turnJ6);
}

bool applyAxisPreset(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	std::string preset;
	if (!RobotInstruction::motionAxisConfigPresetFromToken(value, preset))
	{
		if (err)
		{
			*err = "Invalid axis config preset.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	RobotInstruction::applyPresetToConfiguration(preset, cfg);
	cfg.preset = preset;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisElbow(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	RobotInstruction::ElbowPosture e{};
	if (!RobotInstruction::elbowPostureFromToken(value, e))
	{
		if (err)
		{
			*err = "Invalid elbow posture.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.elbow = e;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisWrist(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	RobotInstruction::WristPosture w{};
	if (!RobotInstruction::wristPostureFromToken(value, w))
	{
		if (err)
		{
			*err = "Invalid wrist posture.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.wrist = w;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisArm(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	RobotInstruction::ArmPosture a{};
	if (!RobotInstruction::armPostureFromToken(value, a))
	{
		if (err)
		{
			*err = "Invalid arm posture.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.preset = "CUSTOM";
	cfg.arm = a;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisTurnJ1(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	int t = RobotInstruction::kMotionAxisTurnAuto;
	if (!RobotInstruction::jointTurnFromToken(value, t))
	{
		if (err)
		{
			*err = "Invalid J1 turn.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ1 = t;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisTurnJ4(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	int t = RobotInstruction::kMotionAxisTurnAuto;
	if (!RobotInstruction::jointTurnFromToken(value, t))
	{
		if (err)
		{
			*err = "Invalid J4 turn.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ4 = t;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}
bool applyAxisTurnJ6(Base& cmd, const std::string& value, std::string* err)
{
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return true;
	}
	int t = RobotInstruction::kMotionAxisTurnAuto;
	if (!RobotInstruction::jointTurnFromToken(value, t))
	{
		if (err)
		{
			*err = "Invalid J6 turn.";
		}
		return false;
	}
	MotionAxisConfiguration cfg = cmd.motionAxisConfiguration();
	cfg.turnJ6 = t;
	cmd.setMotionAxisConfiguration(cfg);
	return true;
}

std::string formatConditionKind(const Base& cmd)
{
	switch (cmd.condition().kind)
	{
	case ConditionKind::Never:
		return "never";
	case ConditionKind::Io:
		return "io";
	case ConditionKind::Compare:
		return "compare";
	case ConditionKind::Always:
	default:
		return "always";
	}
}
std::string formatConditionSignalName(const Base& cmd)
{
	return cmd.condition().signalName;
}
std::string formatConditionPort(const Base& cmd)
{
	return property_core::formatDoubleFixed3(static_cast<double>(cmd.condition().ioPort));
}
std::string formatConditionEquals(const Base& cmd)
{
	return cmd.condition().ioEquals ? "1" : "0";
}

bool applyConditionKind(Base& cmd, const std::string& value, std::string* err)
{
	Condition c = cmd.condition();
	if (value == "never")
	{
		c.kind = ConditionKind::Never;
	}
	else if (value == "io")
	{
		c.kind = ConditionKind::Io;
	}
	else if (value == "compare")
	{
		c.kind = ConditionKind::Compare;
	}
	else if (value == "always")
	{
		c.kind = ConditionKind::Always;
	}
	else
	{
		if (err)
		{
			*err = "Invalid condition kind.";
		}
		return false;
	}
	cmd.setCondition(c);
	return true;
}
bool applyConditionSignalName(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	Condition c = cmd.condition();
	c.signalName = value;
	cmd.setCondition(c);
	return true;
}
bool applyConditionPort(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	Condition c = cmd.condition();
	c.ioPort = static_cast<int>(v);
	cmd.setCondition(c);
	return true;
}
bool applyConditionEquals(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	Condition c = cmd.condition();
	c.ioEquals = (value == "1" || value == "true" || value == "on");
	cmd.setCondition(c);
	return true;
}

std::string formatIoSignalName(const Base& cmd)
{
	return cmd.ioSignalName();
}
std::string formatIoPort(const Base& cmd)
{
	return property_core::formatDoubleFixed3(static_cast<double>(cmd.ioPort()));
}
std::string formatIoDigitalValue(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.ioBoolValue() ? 1.0 : 0.0);
}
std::string formatIoAnalogValue(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.ioAnalogValue());
}

bool applyIoSignalName(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	cmd.setIoSignalName(value);
	return true;
}
bool applyIoPort(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setIoPort(static_cast<int>(v));
	return true;
}
bool applyIoDigitalValue(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setIoBoolValue(v >= 0.5);
	return true;
}
bool applyIoAnalogValue(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setIoAnalogValue(v);
	return true;
}

std::string formatDeviceBackendId(const Base& cmd)
{
	return cmd.deviceBackendId();
}
std::string formatDeviceAxisIndex(const Base& cmd)
{
	return property_core::formatDoubleFixed3(static_cast<double>(cmd.deviceAxisIndex()));
}
std::string formatDeviceTargetQ(const Base& cmd)
{
	return property_core::formatDoubleFixed3(cmd.deviceAxisTargetQ());
}

bool applyDeviceBackendId(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	cmd.setDeviceBackendId(value);
	return true;
}
bool applyDeviceAxisIndex(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setDeviceAxisIndex(static_cast<int>(v));
	return true;
}
bool applyDeviceTargetQ(Base& cmd, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	cmd.setDeviceAxisTargetQ(v);
	return true;
}

std::string formatPlanningPhase(const Base& cmd)
{
	const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	if (!pp)
	{
		return "draft";
	}
	switch (pp->phase())
	{
	case RobotInstruction::PathPlanPhase::RawReady:
		return "raw_ready";
	case RobotInstruction::PathPlanPhase::Applied:
		return "applied";
	default:
		return "draft";
	}
}
std::string formatPlanningOutputGroupId(const Base& cmd)
{
	const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	return pp ? pp->outputGroupId() : std::string();
}
std::string formatPlanningRawTrajectoryKey(const Base& cmd)
{
	const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	return pp ? pp->rawTrajectoryKey() : std::string();
}
std::string formatPlanningPipelineOpCount(const Base& cmd)
{
	const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	return property_core::formatDoubleFixed3(pp ? static_cast<double>(pp->pipeline().size()) : 0.0);
}
std::string formatPlanningRawRevision(const Base& cmd)
{
	const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	return property_core::formatDoubleFixed3(pp ? static_cast<double>(pp->rawRevision()) : 0.0);
}

bool applyPlanningPhase(Base& cmd, const std::string& value, std::string* err)
{
	RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	if (!pp)
	{
		if (err)
		{
			*err = "Not a path plan instruction.";
		}
		return false;
	}
	RobotInstruction::PathPlanPhase phase = RobotInstruction::PathPlanPhase::Draft;
	if (value == "raw_ready" || value == "RawReady")
	{
		phase = RobotInstruction::PathPlanPhase::RawReady;
	}
	else if (value == "applied" || value == "Applied")
	{
		phase = RobotInstruction::PathPlanPhase::Applied;
	}
	else if (value == "draft" || value == "Draft")
	{
		phase = RobotInstruction::PathPlanPhase::Draft;
	}
	else
	{
		if (err)
		{
			*err = "Invalid planning phase.";
		}
		return false;
	}
	pp->setPhase(phase);
	return true;
}
bool applyPlanningOutputGroupId(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	if (!pp)
	{
		return false;
	}
	pp->setOutputGroupId(value);
	return true;
}
bool applyPlanningRawTrajectoryKey(Base& cmd, const std::string& value, std::string* err)
{
	(void)err;
	RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(cmd);
	if (!pp)
	{
		return false;
	}
	pp->setRawTrajectoryKey(value);
	return true;
}
bool applyReadOnly(Base&, const std::string&, std::string* err)
{
	if (err)
	{
		*err = "Property is read-only for this object type.";
	}
	return false;
}

const std::vector<InstructionPropertyBinding>& allCandidateBindings()
{
	static const std::vector<InstructionPropertyBinding> kAll = {
		{makeDesc("motion.target.pose.x", "Target X (mm)", PropertyType::Double, true, kMotion), formatTargetPoseX,
		 applyTargetPoseX, isActiveHasPose},
		{makeDesc("motion.target.pose.y", "Target Y (mm)", PropertyType::Double, true, kMotion), formatTargetPoseY,
		 applyTargetPoseY, isActiveHasPose},
		{makeDesc("motion.target.pose.z", "Target Z (mm)", PropertyType::Double, true, kMotion), formatTargetPoseZ,
		 applyTargetPoseZ, isActiveHasPose},
		{makeDesc("motion.target.euler.rx", "Euler RX (deg)", PropertyType::Double, true, kMotion), formatTargetEulerRx,
		 applyTargetEulerRx, isActiveHasEuler},
		{makeDesc("motion.target.euler.ry", "Euler RY (deg)", PropertyType::Double, true, kMotion), formatTargetEulerRy,
		 applyTargetEulerRy, isActiveHasEuler},
		{makeDesc("motion.target.euler.rz", "Euler RZ (deg)", PropertyType::Double, true, kMotion), formatTargetEulerRz,
		 applyTargetEulerRz, isActiveHasEuler},
		{makeDesc("motion.via.pose.x", "Via X (mm)", PropertyType::Double, true, kMotion), formatViaPoseX, applyViaPoseX,
		 isActiveHasViaPose},
		{makeDesc("motion.via.pose.y", "Via Y (mm)", PropertyType::Double, true, kMotion), formatViaPoseY, applyViaPoseY,
		 isActiveHasViaPose},
		{makeDesc("motion.via.pose.z", "Via Z (mm)", PropertyType::Double, true, kMotion), formatViaPoseZ, applyViaPoseZ,
		 isActiveHasViaPose},
		{makeDesc("motion.via.euler.rx", "Via Euler RX (deg)", PropertyType::Double, true, kMotion), formatViaEulerRx,
		 applyViaEulerRx, isActiveHasViaEuler},
		{makeDesc("motion.via.euler.ry", "Via Euler RY (deg)", PropertyType::Double, true, kMotion), formatViaEulerRy,
		 applyViaEulerRy, isActiveHasViaEuler},
		{makeDesc("motion.via.euler.rz", "Via Euler RZ (deg)", PropertyType::Double, true, kMotion), formatViaEulerRz,
		 applyViaEulerRz, isActiveHasViaEuler},
		{makeDesc("motion.speed", "Speed", PropertyType::Double, true, kMotion), formatSpeed, applySpeed,
		 isActiveHasSpeed},
		{makeDesc("motion.acc", "Acceleration", PropertyType::Double, true, kMotion), formatAccel, applyAccel,
		 isActiveHasAccel},
		{makeDesc("motion.blendRadius", "Blend Radius (mm)", PropertyType::Double, true, kMotion), formatBlendRadius,
		 applyBlendRadius, isActiveHasBlend},
		{makeEnumDesc("motion.axisConfig.preset", "Axis config preset", "AUTO",
					  RobotInstruction::motionAxisConfigPresetTokens(), true, kMotion),
		 formatAxisPreset, applyAxisPreset, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.elbow", "Elbow posture", "AUTO", RobotInstruction::elbowPostureTokens(), true,
					  kMotion),
		 formatAxisElbow, applyAxisElbow, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.wrist", "Wrist posture", "AUTO", RobotInstruction::wristPostureTokens(), true,
					  kMotion),
		 formatAxisWrist, applyAxisWrist, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.arm", "Arm posture", "AUTO", RobotInstruction::armPostureTokens(), true,
					  kMotion),
		 formatAxisArm, applyAxisArm, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.turn.j1", "J1 turn", "AUTO", RobotInstruction::motionAxisTurnTokens(), true,
					  kMotion),
		 formatAxisTurnJ1, applyAxisTurnJ1, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.turn.j4", "J4 turn", "AUTO", RobotInstruction::motionAxisTurnTokens(), true,
					  kMotion),
		 formatAxisTurnJ4, applyAxisTurnJ4, isActiveHasMotionAxisCfg},
		{makeEnumDesc("motion.axisConfig.turn.j6", "J6 turn", "AUTO", RobotInstruction::motionAxisTurnTokens(), true,
					  kMotion),
		 formatAxisTurnJ6, applyAxisTurnJ6, isActiveHasMotionAxisCfg},
		{makeEnumDesc("logic.condition.kind", "Condition kind", "always",
					  {"never", "io", "compare", "always"}),
		 formatConditionKind, applyConditionKind, isActiveHasCondition},
		{makeDesc("logic.condition.signalName", "Signal name", PropertyType::String, true), formatConditionSignalName,
		 applyConditionSignalName, isActiveConditionIo},
		{makeDesc("logic.condition.port", "IO port", PropertyType::Double, true), formatConditionPort,
		 applyConditionPort, isActiveConditionIo},
		{makeEnumDesc("logic.condition.equals", "Equals (0/1)", "1", {"0", "1"}), formatConditionEquals,
		 applyConditionEquals, isActiveConditionIo},
		{makeDesc("logic.wait.durationSec", "Duration/Timeout (s)", PropertyType::Double, true), formatDurationSec,
		 applyDurationSec, isActiveWaitDuration},
		{makeDesc("logic.io.signalName", "Signal name", PropertyType::String, true), formatIoSignalName,
		 applyIoSignalName, isActiveHasIoSignal},
		{makeDesc("logic.io.port", "Port", PropertyType::Double, true), formatIoPort, applyIoPort, isActiveHasIoPort},
		{makeDesc("logic.io.digitalValue", "Value (0/1)", PropertyType::Double, true), formatIoDigitalValue,
		 applyIoDigitalValue, isActiveHasIoDigital},
		{makeDesc("logic.io.analogValue", "Analog value", PropertyType::Double, true), formatIoAnalogValue,
		 applyIoAnalogValue, isActiveSetAoAnalog},
		{makeDesc("logic.device.backendId", "Device id", PropertyType::String, true), formatDeviceBackendId,
		 applyDeviceBackendId, isActiveHasDeviceAxis},
		{makeDesc("logic.device.axisIndex", "Axis index", PropertyType::Double, true), formatDeviceAxisIndex,
		 applyDeviceAxisIndex, isActiveHasDeviceAxis},
		{makeDesc("logic.device.targetQ", "Target q", PropertyType::Double, true), formatDeviceTargetQ,
		 applyDeviceTargetQ, isActiveHasDeviceAxis},
		{makeDesc("logic.device.durationSec", "Duration (s)", PropertyType::Double, true), formatDurationSec,
		 applyDurationSec, isActiveDeviceDuration},
		{makeDesc("planning.phase", "Phase", PropertyType::String, true), formatPlanningPhase, applyPlanningPhase,
		 isActivePathPlan},
		{makeDesc("planning.outputGroupId", "Output group", PropertyType::String, true), formatPlanningOutputGroupId,
		 applyPlanningOutputGroupId, isActivePathPlan},
		{makeDesc("planning.rawTrajectoryKey", "Raw key", PropertyType::String, true), formatPlanningRawTrajectoryKey,
		 applyPlanningRawTrajectoryKey, isActivePathPlan},
		{makeDesc("planning.pipelineOpCount", "Pipeline ops", PropertyType::Double, false), formatPlanningPipelineOpCount,
		 applyReadOnly, isActivePathPlan},
		{makeDesc("planning.rawRevision", "Raw revision", PropertyType::Double, false), formatPlanningRawRevision,
		 applyReadOnly, isActivePathPlan},
	};
	return kAll;
}

bool includeBindingInSchema(const InstructionPropertyBinding& binding, const Type type, const Base& emptyCmd)
{
	if (!binding.isActive || binding.isActive(emptyCmd))
	{
		return true;
	}
	if (type == Type::WAIT || type == Type::IF || type == Type::WHILE)
	{
		const std::string& key = binding.desc.key;
		if (key.rfind("logic.condition.", 0) == 0)
		{
			return true;
		}
	}
	return false;
}

property_core::PropertySchema buildSchemaForType(const Type type)
{
	const std::shared_ptr<Base> emptyCmd = makeEmptyOfType(type);
	property_core::PropertySchema schema;
	schema.objectTypeId = objectTypeIdFor(type);
	schema.schemaVersion = 1;
	for (const InstructionPropertyBinding& binding : allCandidateBindings())
	{
		if (!includeBindingInSchema(binding, type, *emptyCmd))
		{
			continue;
		}
		PropertyDescriptor desc = binding.desc;
		if (type == Type::WAIT && desc.key == "logic.condition.kind")
		{
			desc.label = "Wait mode";
			desc.constraints.enumConstraint.options = {"always", "io"};
			desc.defaultValue = std::string("io");
		}
		schema.descriptors.push_back(std::move(desc));
	}
	return schema;
}

} // namespace

namespace RobotInstruction::instruction_property_binding
{
std::vector<InstructionPropertyBinding> collectBindings(const Base& cmd)
{
	std::vector<InstructionPropertyBinding> out;
	out.reserve(allCandidateBindings().size());
	for (const InstructionPropertyBinding& binding : allCandidateBindings())
	{
		if (binding.isActive && !binding.isActive(cmd))
		{
			continue;
		}
		out.push_back(binding);
	}
	return out;
}

const property_core::PropertySchema& schemaForType(const Type type)
{
	static std::mutex mu;
	static std::unordered_map<int, property_core::PropertySchema> cache;
	{
		std::lock_guard<std::mutex> lock(mu);
		const auto it = cache.find(static_cast<int>(type));
		if (it != cache.end())
		{
			return it->second;
		}
	}
	property_core::PropertySchema built = buildSchemaForType(type);
	{
		std::lock_guard<std::mutex> lock(mu);
		auto inserted = cache.emplace(static_cast<int>(type), std::move(built));
		if (!inserted.second)
		{
			return inserted.first->second;
		}
	}

#ifndef NDEBUG
	static int selfTestState = 0;
	if (selfTestState == 0)
	{
		selfTestState = 1;
		if (!runInstructionPropertyBindingSelfTest(nullptr))
		{
			RunLogger::error("InstructionPropertyBindingSelfTest failed.");
		}
		selfTestState = 2;
	}
#endif

	std::lock_guard<std::mutex> lock(mu);
	return cache.find(static_cast<int>(type))->second;
}

void appendBindingRows(const Base& cmd, nlohmann::json& rows)
{
	for (const InstructionPropertyBinding& binding : collectBindings(cmd))
	{
		if (!binding.formatValue)
		{
			continue;
		}
		nlohmann::json row;
		row["key"] = binding.desc.key;
		row["label"] = binding.desc.label;
		row["editable"] = binding.desc.editable;
		row["value"] = binding.formatValue(cmd);
		rows.push_back(std::move(row));
	}
}

bool applyBindingKey(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg)
{
	for (const InstructionPropertyBinding& binding : allCandidateBindings())
	{
		if (binding.desc.key != key)
		{
			continue;
		}
		if (binding.isActive && !binding.isActive(cmd))
		{
			return false;
		}
		if (!binding.desc.editable || !binding.applyValue)
		{
			if (errMsg)
			{
				*errMsg = "Property is read-only for this object type.";
			}
			return false;
		}
		return binding.applyValue(cmd, value, errMsg);
	}
	return false;
}

} // namespace RobotInstruction::instruction_property_binding
