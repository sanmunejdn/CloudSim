#include "UrdfRobotKinematicModel.h"

#include "UrdfRobotLoader.h"

#include "TreeForwardKinematics.h"

namespace UrdfRobotKinematicModel
{
Model::Model(QString urdfPath) : m_urdfPath(std::move(urdfPath))
{
	rebuildGraph(nullptr);
}

bool Model::rebuildGraph(QString* errorMessage)
{
	m_graph = kinematic_core::KinematicGraph{};
	m_jointNames.clear();
	m_lowerRad.clear();
	m_upperRad.clear();
	if (m_urdfPath.isEmpty())
	{
		return false;
	}
	if (!UrdfRobotLoader::buildUrdfKinematicGraph(m_urdfPath, m_graph, errorMessage))
	{
		return false;
	}
	return UrdfRobotLoader::loadRevoluteJointMeta(m_urdfPath, m_jointNames, m_lowerRad, m_upperRad, errorMessage);
}

std::vector<kinematic_core::AxisDescriptor> Model::axisDescriptors() const
{
	std::vector<kinematic_core::AxisDescriptor> out;
	out.reserve(static_cast<std::size_t>(m_jointNames.size()));
	for (int i = 0; i < m_jointNames.size(); ++i)
	{
		kinematic_core::AxisDescriptor d;
		d.name = m_jointNames.at(i).toStdString();
		d.qIndex = i;
		d.motionType = kinematic_core::JointMotionType::Revolute;
		d.lower = i < m_lowerRad.size() ? m_lowerRad.at(i) : -3.14159265358979323846;
		d.upper = i < m_upperRad.size() ? m_upperRad.at(i) : 3.14159265358979323846;
		d.enabled = true;
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

std::shared_ptr<Model> create(const QString& urdfPath)
{
	return std::make_shared<Model>(urdfPath);
}

} // namespace UrdfRobotKinematicModel
