#include "CustomDeviceKinematicModel.h"

#include "CustomDeviceGraphBuilder.h"
#include "CustomDeviceMat4Layout.h"

#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "TreeForwardKinematics.h"

#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace CustomDeviceKinematicModel
{
namespace
{
void writeSinkWorld(IRobotBackendPoseSink* sink, const std::string& backendId, const BackendMat4& wm)
{
	if (!sink || backendId.empty())
	{
		return;
	}
	cloudsim::core::Mat4 mat{};
	for (int k = 0; k < 16; ++k)
	{
		mat[static_cast<size_t>(k)] = wm.v[k];
	}
	sink->setBackendRootWorldMatrixFromWorld(backendId, mat);
}

/// 连杆 compound 刚体增量：子件无 Follow，须随父几何同乘 Δ=W_new·inv(W_old)
void applyCompoundDeltaToDescendants(BackendDataManager& mgr, IRobotBackendPoseSink* sink,
									 const std::string& linkGeomId, const BackendMat4& delta,
									 const std::unordered_set<std::string>& linkGeomIds)
{
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	visited.insert(linkGeomId);
	queue.push(linkGeomId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		for (const std::string& childId : mgr.childrenOf(cur))
		{
			if (!visited.insert(childId).second)
			{
				continue;
			}
			queue.push(childId);
			if (linkGeomIds.count(childId) != 0)
			{
				continue;
			}
			const auto child = mgr.getData(childId);
			if (!child || !child->hasPoseProperty())
			{
				continue;
			}
			BackendMat4 childNew{};
			if (!backend_mat4_multiply(delta, child->worldMatrix(&mgr), childNew))
			{
				continue;
			}
			child->setWorldMatrix(childNew, &mgr);
			writeSinkWorld(sink, childId, childNew);
		}
	}
}
} // namespace

Model::Model(CustomDeviceBackendData& device) : m_device(device)
{
	rebuildGraph();
}

bool Model::rebuildGraph()
{
	return CustomDeviceGraphBuilder::buildGraph(m_device, m_graph, m_rootLinkIdx);
}

std::vector<kinematic_core::AxisDescriptor> Model::axisDescriptors() const
{
	std::vector<kinematic_core::AxisDescriptor> out;
	for (const kinematic_core::KinematicJoint& j : m_graph.joints)
	{
		if (j.qIndex < 0 || !j.motion.enabled)
		{
			continue;
		}
		kinematic_core::AxisDescriptor d;
		d.name = j.motion.name;
		d.qIndex = j.qIndex;
		d.motionType = j.motion.motionType;
		d.lower = j.motion.lower;
		d.upper = j.motion.upper;
		d.home = j.motion.home;
		d.enabled = j.motion.enabled;
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
	std::vector<std::array<double, 16>> buf(m_graph.links.size());
	double w0[16];
	CustomDeviceMat4Layout::backendMat4ToKinematicCore(m_device.baseWorldW0(), w0);
	if (!kinematic_core::forwardKinematicsTree(m_graph, w0, q, qCount,
											   reinterpret_cast<double(*)[16]>(buf.data())))
	{
		return false;
	}
	linkWorld = std::move(buf);
	return true;
}

bool Model::applyToSink(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
						const std::vector<double>& q, const double w0[16]) const
{
	(void)device;
	if (m_graph.links.empty())
	{
		return false;
	}
	std::vector<std::array<double, 16>> buf(m_graph.links.size());
	if (!kinematic_core::forwardKinematicsTree(m_graph, w0, q.data(), q.size(),
											   reinterpret_cast<double(*)[16]>(buf.data())))
	{
		return false;
	}

	std::unordered_set<std::string> linkGeomIds;
	linkGeomIds.reserve(m_graph.links.size());
	for (const kinematic_core::KinematicLink& L : m_graph.links)
	{
		if (!L.payloadKey.empty())
		{
			linkGeomIds.insert(L.payloadKey);
		}
	}

	for (size_t i = 0; i < m_graph.links.size(); ++i)
	{
		const kinematic_core::KinematicLink& L = m_graph.links[i];
		if (L.payloadKey.empty())
		{
			continue;
		}
		const auto geom = mgr ? mgr->getData(L.payloadKey) : nullptr;
		if (!geom || !geom->hasPoseProperty())
		{
			continue;
		}
		const BackendMat4 wOld = geom->worldMatrix(mgr);
		const BackendMat4 wNew = CustomDeviceMat4Layout::kinematicCoreToBackendMat4(buf[i].data());
		geom->setWorldMatrix(wNew, mgr);
		writeSinkWorld(sink, L.payloadKey, wNew);

		if (!mgr)
		{
			continue;
		}
		BackendMat4 invOld{};
		BackendMat4 delta{};
		if (!backend_mat4_invert_rigid(wOld, invOld) || !backend_mat4_multiply(wNew, invOld, delta))
		{
			continue;
		}
		applyCompoundDeltaToDescendants(*mgr, sink, L.payloadKey, delta, linkGeomIds);
	}
	return true;
}

std::shared_ptr<Model> create(CustomDeviceBackendData& device)
{
	return std::make_shared<Model>(device);
}

bool forwardLinkWorldById(const CustomDeviceBackendData& device, BackendDataManager* mgr, const std::vector<double>& q,
						  std::unordered_map<std::string, std::array<double, 16>>& worldByLink)
{
	worldByLink.clear();
	kinematic_core::KinematicGraph graph;
	int rootIdx = 0;
	if (!CustomDeviceGraphBuilder::buildGraph(device, graph, rootIdx))
	{
		return false;
	}
	std::vector<std::array<double, 16>> buf(graph.links.size());
	double w0[16];
	const BackendMat4 w0Mat = mgr ? device.worldMatrix(mgr) : device.baseWorldW0();
	CustomDeviceMat4Layout::backendMat4ToKinematicCore(w0Mat, w0);
	if (!kinematic_core::forwardKinematicsTree(graph, w0, q.data(), q.size(),
											   reinterpret_cast<double(*)[16]>(buf.data())))
	{
		return false;
	}
	for (size_t i = 0; i < graph.links.size(); ++i)
	{
		std::array<double, 16> wf{};
		CustomDeviceMat4Layout::kinematicCoreToOsgBackend(buf[i].data(), wf.data());
		worldByLink[graph.links[i].id] = wf;
	}
	return !worldByLink.empty();
}

} // namespace CustomDeviceKinematicModel
