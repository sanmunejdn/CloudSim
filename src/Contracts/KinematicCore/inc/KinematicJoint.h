#ifndef KINEMATICCORE_KINEMATICJOINT_H
#define KINEMATICCORE_KINEMATICJOINT_H

#include "JointMotion1D.h"
#include "kinematic_core_global.h"

namespace kinematic_core
{
enum class JointTransformOrder : int
{
	MotionThenRest = 0,
	RestThenMotion = 1
};

struct KINEMATIC_CORE_API KinematicJoint
{
	int parentLinkIdx = -1;
	int childLinkIdx = -1;
	JointMotion1D motion;
	double parentToChildRest[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	int qIndex = -1;
	JointTransformOrder transformOrder = JointTransformOrder::MotionThenRest;
};

} // namespace kinematic_core

#endif // KINEMATICCORE_KINEMATICJOINT_H
