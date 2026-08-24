#ifndef KINEMATICCORE_AXISDESCRIPTOR_H
#define KINEMATICCORE_AXISDESCRIPTOR_H

#include "JointMotion1D.h"
#include "kinematic_core_global.h"

#include <string>

namespace kinematic_core
{
struct KINEMATIC_CORE_API AxisDescriptor
{
	std::string name;
	int qIndex = -1;
	JointMotionType motionType = JointMotionType::Translate;
	double lower = 0.0;
	double upper = 0.0;
	double home = 0.0;
	bool enabled = true;

	AxisDescriptor();
	~AxisDescriptor();
	AxisDescriptor(const AxisDescriptor&);
	AxisDescriptor(AxisDescriptor&&) noexcept;
	AxisDescriptor& operator=(const AxisDescriptor&);
	AxisDescriptor& operator=(AxisDescriptor&&) noexcept;
};

} // namespace kinematic_core

#endif // KINEMATICCORE_AXISDESCRIPTOR_H
