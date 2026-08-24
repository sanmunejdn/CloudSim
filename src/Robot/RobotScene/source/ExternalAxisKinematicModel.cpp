#include "ExternalAxisKinematicModel.h"

#include "ExternalAxisGraphBuilder.h"

#include "TreeForwardKinematics.h"

namespace ExternalAxisKinematicModel
{
Model::Model(RobotExternal::RobotExternalAxisConfigSet set, const RobotExternal::RobotExternalAttachment attachment)
	: m_set(std::move(set))
{
	ExternalAxisGraphBuilder::buildChain(m_set, attachment, m_graph, m_axisIndicesInSet);
}

std::vector<kinematic_core::AxisDescriptor> Model::axisDescriptors() const
{
	std::vector<kinematic_core::AxisDescriptor> out;
	for (const int idx : m_axisIndicesInSet)
	{
		if (idx < 0 || idx >= static_cast<int>(m_set.axes.size()))
		{
			continue;
		}
		const RobotExternal::RobotExternalAxisConfig& cfg = m_set.axes[static_cast<size_t>(idx)];
		kinematic_core::AxisDescriptor d;
		d.name = cfg.displayName.empty() ? cfg.jointName : cfg.displayName;
		d.qIndex = static_cast<int>(out.size());
		d.motionType = cfg.motionType == RobotExternal::RobotExternalMotionType::Translate
						   ? kinematic_core::JointMotionType::Translate
						   : kinematic_core::JointMotionType::Revolute;
		d.lower = cfg.lower;
		d.upper = cfg.upper;
		d.home = cfg.home;
		d.enabled = cfg.enabled;
		out.push_back(d);
	}
	return out;
}

bool Model::forward(const double* q, const std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const
{
	if (m_graph.links.empty())
	{
		return false;
	}
	linkWorld.assign(m_graph.links.size(), {});
	double base[16];
	for (int i = 0; i < 16; ++i)
	{
		base[i] = (i % 5 == 0) ? 1.0 : 0.0;
	}
	return kinematic_core::forwardKinematicsTree(m_graph, base, q, qCount,
												 reinterpret_cast<double(*)[16]>(linkWorld.data()));
}

std::shared_ptr<Model> create(const RobotExternal::RobotExternalAxisConfigSet& set,
							  const RobotExternal::RobotExternalAttachment attachment)
{
	return std::make_shared<Model>(set, attachment);
}

} // namespace ExternalAxisKinematicModel
