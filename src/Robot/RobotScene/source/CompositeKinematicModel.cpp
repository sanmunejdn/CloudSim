#include "CompositeKinematicModel.h"

#include "ExternalAxisKinematicModel.h"
#include "IRobotSimulationDocument.h"
#include "RobotExternalAxisSceneApply.h"
#include "UrdfRobotKinematicModel.h"
#include "UrdfRobotKinematicModelSink.h"

#include <array>
namespace CompositeKinematicModel
{
void Model::addSegment(std::shared_ptr<kinematic_core::IKinematicModel> segment)
{
	if (segment)
	{
		m_segments.push_back(std::move(segment));
	}
}

const kinematic_core::KinematicGraph& Model::graph() const
{
	m_cachedGraph.links.clear();
	m_cachedGraph.joints.clear();
	m_cachedGraph.rootLinkIdx = 0;
	for (const auto& seg : m_segments)
	{
		const kinematic_core::KinematicGraph& g = seg->graph();
		const int linkOffset = static_cast<int>(m_cachedGraph.links.size());
		for (const kinematic_core::KinematicLink& L : g.links)
		{
			m_cachedGraph.links.push_back(L);
		}
		int qOffset = m_cachedGraph.dofCount();
		for (kinematic_core::KinematicJoint j : g.joints)
		{
			j.parentLinkIdx += linkOffset;
			j.childLinkIdx += linkOffset;
			if (j.qIndex >= 0)
			{
				j.qIndex += qOffset;
			}
			m_cachedGraph.joints.push_back(j);
		}
	}
	return m_cachedGraph;
}

int Model::dofCount() const
{
	int n = 0;
	for (const auto& seg : m_segments)
	{
		n += seg->dofCount();
	}
	return n;
}

std::vector<kinematic_core::AxisDescriptor> Model::axisDescriptors() const
{
	std::vector<kinematic_core::AxisDescriptor> out;
	int qOffset = 0;
	for (const auto& seg : m_segments)
	{
		std::vector<kinematic_core::AxisDescriptor> d = seg->axisDescriptors();
		for (kinematic_core::AxisDescriptor& row : d)
		{
			if (row.qIndex >= 0)
			{
				row.qIndex += qOffset;
			}
			out.push_back(row);
		}
		qOffset += seg->dofCount();
	}
	return out;
}

bool Model::forward(const double* q, const std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const
{
	linkWorld.clear();
	std::size_t qOffset = 0;
	for (const auto& seg : m_segments)
	{
		const int nd = seg->dofCount();
		if (qOffset + static_cast<std::size_t>(nd) > qCount)
		{
			return false;
		}
		std::vector<std::array<double, 16>> segWorld;
		if (!seg->forward(q + qOffset, static_cast<std::size_t>(nd), segWorld))
		{
			return false;
		}
		linkWorld.insert(linkWorld.end(), segWorld.begin(), segWorld.end());
		qOffset += static_cast<std::size_t>(nd);
	}
	return true;
}

int Model::armDofCount() const
{
	if (m_segments.empty())
	{
		return 0;
	}
	return m_segments.front()->dofCount();
}

int Model::externalDofCount() const
{
	int n = 0;
	for (std::size_t i = 1; i < m_segments.size(); ++i)
	{
		n += m_segments[i]->dofCount();
	}
	return n;
}

bool Model::applyToSink(const RobotKinematicApplyContext::Context& ctx, const std::vector<double>& armQ,
						const std::vector<double>* externalFullQ, QVector<double>& aggregatedAnglesRad) const
{
	if (m_segments.empty())
	{
		return false;
	}
	const auto arm = std::dynamic_pointer_cast<UrdfRobotKinematicModel::Model>(m_segments.front());
	if (!arm)
	{
		return false;
	}
	if (!UrdfRobotKinematicModelSink::applyToSink(*arm, ctx, armQ, aggregatedAnglesRad))
	{
		return false;
	}
	if (externalDofCount() <= 0 || !ctx.doc)
	{
		return true;
	}
	std::vector<double> extQ;
	if (externalFullQ && !externalFullQ->empty())
	{
		extQ = *externalFullQ;
	}
	else
	{
		extQ = ctx.doc->robotExternalAxisQ(ctx.instanceIndex);
	}
	return RobotExternalAxisSceneApply::applyExternalAxisQ(ctx.doc, ctx.sink, ctx.instanceIndex, extQ);
}

bool Model::applyArmToSink(const RobotKinematicApplyContext::Context& ctx, const std::vector<double>& localArmQ,
						   QVector<double>& aggregatedAnglesRad) const
{
	return applyToSink(ctx, localArmQ, nullptr, aggregatedAnglesRad);
}

} // namespace CompositeKinematicModel