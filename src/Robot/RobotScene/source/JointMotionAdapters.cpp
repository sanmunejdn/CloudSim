#include "JointMotionAdapters.h"

namespace JointMotionAdapters
{
kinematic_core::JointMotion1D fromCustomDeviceAxisConfig(const CustomDeviceAxisConfig& in)
{
	kinematic_core::JointMotion1D out;
	out.enabled = in.enabled;
	out.name = in.jointName.empty() ? in.displayName : in.jointName;
	out.motionType = in.motionType == CustomDeviceMotionType::Rotate ? kinematic_core::JointMotionType::Revolute
																	 : kinematic_core::JointMotionType::Translate;
	out.lower = in.lower;
	out.upper = in.upper;
	out.home = in.home;
	out.axis[0] = in.axis[0];
	out.axis[1] = in.axis[1];
	out.axis[2] = in.axis[2];
	out.originMm[0] = in.originMm[0];
	out.originMm[1] = in.originMm[1];
	out.originMm[2] = in.originMm[2];
	return out;
}

kinematic_core::JointMotion1D fromRobotExternalAxisConfig(const RobotExternal::RobotExternalAxisConfig& in)
{
	kinematic_core::JointMotion1D out;
	out.enabled = in.enabled;
	out.name = in.jointName.empty() ? in.displayName : in.jointName;
	out.motionType = in.motionType == RobotExternal::RobotExternalMotionType::Rotate
						 ? kinematic_core::JointMotionType::Revolute
						 : kinematic_core::JointMotionType::Translate;
	out.lower = in.lower;
	out.upper = in.upper;
	out.home = in.home;
	out.axis[0] = in.axis[0];
	out.axis[1] = in.axis[1];
	out.axis[2] = in.axis[2];
	out.originMm[0] = in.originMm[0];
	out.originMm[1] = in.originMm[1];
	out.originMm[2] = in.originMm[2];
	return out;
}

} // namespace JointMotionAdapters
