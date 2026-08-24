#ifndef ROBOTSCENE_EXTERNALAXISKINEMATICMODEL_H
#define ROBOTSCENE_EXTERNALAXISKINEMATICMODEL_H

#include "RobotExternalAxes.h"
#include "robot_scene_global.h"

#include "IKinematicModel.h"
#include "KinematicGraph.h"

#include <array>
#include <memory>
#include <vector>

namespace ExternalAxisKinematicModel
{
class ROBOT_SCENE_API Model : public kinematic_core::IKinematicModel
{
public:
	Model(RobotExternal::RobotExternalAxisConfigSet set, RobotExternal::RobotExternalAttachment attachment);

	const kinematic_core::KinematicGraph& graph() const override { return m_graph; }
	int dofCount() const override { return m_graph.dofCount(); }
	std::vector<kinematic_core::AxisDescriptor> axisDescriptors() const override;
	bool forward(const double* q, std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const override;

private:
	RobotExternal::RobotExternalAxisConfigSet m_set;
	kinematic_core::KinematicGraph m_graph;
	std::vector<int> m_axisIndicesInSet;
};

ROBOT_SCENE_API std::shared_ptr<Model> create(const RobotExternal::RobotExternalAxisConfigSet& set,
											  RobotExternal::RobotExternalAttachment attachment);

} // namespace ExternalAxisKinematicModel

#endif // ROBOTSCENE_EXTERNALAXISKINEMATICMODEL_H
