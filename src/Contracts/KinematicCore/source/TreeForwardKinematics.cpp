#include "TreeForwardKinematics.h"

#include "JointMotionEval.h"
#include "Mat4Ops.h"

#include <array>
#include <cstring>
#include <queue>
#include <unordered_map>

namespace kinematic_core
{
namespace
{
void mulRest(const double parentWorld[16], const double motion[16], const double rest[16],
			 kinematic_core::JointTransformOrder order, double childWorld[16])
{
	if (order == kinematic_core::JointTransformOrder::RestThenMotion)
	{
		double parentRest[16];
		mat4MulColumnMajor16(parentWorld, rest, parentRest);
		mat4MulColumnMajor16(parentRest, motion, childWorld);
		return;
	}
	double parentMotion[16];
	mat4MulColumnMajor16(parentWorld, motion, parentMotion);
	mat4MulColumnMajor16(parentMotion, rest, childWorld);
}
} // namespace

bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16], const double* q,
						   const std::size_t qCount, double linkWorld[][16])
{
	std::string err;
	if (!graph.validateTree(&err))
	{
		return false;
	}
	const int nLinks = static_cast<int>(graph.links.size());

	std::unordered_map<int, std::array<double, 16>> worldByLink;
	for (int i = 0; i < nLinks; ++i)
	{
		worldByLink[i].fill(0.0);
		worldByLink[i][0] = worldByLink[i][5] = worldByLink[i][10] = worldByLink[i][15] = 1.0;
	}

	{
		double rootW[16];
		mat4MulColumnMajor16(baseWorld, graph.links[static_cast<size_t>(graph.rootLinkIdx)].restInBase, rootW);
		std::memcpy(worldByLink[graph.rootLinkIdx].data(), rootW, sizeof(rootW));
	}

	std::queue<int> queue;
	std::unordered_map<int, bool> visited;
	queue.push(graph.rootLinkIdx);
	visited[graph.rootLinkIdx] = true;

	while (!queue.empty())
	{
		const int parentIdx = queue.front();
		queue.pop();
		for (const KinematicJoint& j : graph.joints)
		{
			if (j.parentLinkIdx != parentIdx || visited.count(j.childLinkIdx))
			{
				continue;
			}
			double qj = j.motion.home;
			if (j.qIndex >= 0 && static_cast<std::size_t>(j.qIndex) < qCount)
			{
				qj = q[static_cast<size_t>(j.qIndex)];
			}
			double motion[16];
			evaluateJointMotion1D(j.motion, qj, motion);
			double childW[16];
			mulRest(worldByLink[parentIdx].data(), motion, j.parentToChildRest, j.transformOrder, childW);
			std::memcpy(worldByLink[j.childLinkIdx].data(), childW, sizeof(childW));
			visited[j.childLinkIdx] = true;
			queue.push(j.childLinkIdx);
		}
	}

	for (int i = 0; i < nLinks; ++i)
	{
		std::memcpy(linkWorld[i], worldByLink[i].data(), 16 * sizeof(double));
	}
	return true;
}

bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16], const double* q,
						   const std::size_t qCount, std::vector<double>& flatLinkWorld16)
{
	const int nLinks = static_cast<int>(graph.links.size());
	if (nLinks <= 0)
	{
		return false;
	}
	std::vector<std::array<double, 16>> buf(static_cast<size_t>(nLinks));
	if (!forwardKinematicsTree(graph, baseWorld, q, qCount, reinterpret_cast<double(*)[16]>(buf.data())))
	{
		return false;
	}
	flatLinkWorld16.resize(static_cast<size_t>(nLinks) * 16);
	for (int i = 0; i < nLinks; ++i)
	{
		std::memcpy(flatLinkWorld16.data() + static_cast<size_t>(i) * 16, buf[static_cast<size_t>(i)].data(),
					16 * sizeof(double));
	}
	return true;
}

} // namespace kinematic_core
