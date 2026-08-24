#ifndef KINEMATICCORE_IKINEMATICMODEL_H
#define KINEMATICCORE_IKINEMATICMODEL_H

#include "AxisDescriptor.h"
#include "KinematicGraph.h"
#include "kinematic_core_global.h"

#include <array>
#include <vector>

namespace kinematic_core
{
class KINEMATIC_CORE_API IKinematicModel
{
public:
	IKinematicModel();
	virtual ~IKinematicModel();
	virtual const KinematicGraph& graph() const = 0;
	virtual int dofCount() const = 0;
	virtual std::vector<AxisDescriptor> axisDescriptors() const = 0;
	virtual bool forward(const double* q, std::size_t qCount,
						 std::vector<std::array<double, 16>>& linkWorld) const = 0;
	virtual int revision() const { return 0; }
};

} // namespace kinematic_core

#endif // KINEMATICCORE_IKINEMATICMODEL_H
