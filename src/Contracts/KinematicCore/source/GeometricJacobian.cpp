#include "GeometricJacobian.h"

#include "JointMotionEval.h"
#include "Mat4Ops.h"
#include "TreeForwardKinematics.h"

#include <array>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace kinematic_core
{
namespace
{
void extractRotation(const double m[16], double R[9])
{
	R[0] = m[0];
	R[1] = m[1];
	R[2] = m[2];
	R[3] = m[4];
	R[4] = m[5];
	R[5] = m[6];
	R[6] = m[8];
	R[7] = m[9];
	R[8] = m[10];
}

void mulRVec3(const double R[9], const double v[3], double out[3])
{
	out[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
	out[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
	out[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

void cross3(const double a[3], const double b[3], double out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize3(double v[3], const double eps)
{
	const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (n <= eps)
	{
		return;
	}
	const double inv = 1.0 / n;
	v[0] *= inv;
	v[1] *= inv;
	v[2] *= inv;
}

/// URDF 等 RestThenMotion 链：轴与枢轴在 parent*Rest（或 MotionThenRest 时 parent*Motion）坐标系，而非 parent link 原点
void jointAxisOriginInWorld(const KinematicJoint& j, const double parentW[16], const double* q,
							const std::size_t qCount, double axisW[3], double originW[3])
{
	double jointFrameW[16];
	if (j.transformOrder == JointTransformOrder::RestThenMotion)
	{
		mat4MulColumnMajor16(parentW, j.parentToChildRest, jointFrameW);
	}
	else
	{
		double qj = j.motion.home;
		if (j.qIndex >= 0 && static_cast<std::size_t>(j.qIndex) < qCount)
		{
			qj = q[static_cast<std::size_t>(j.qIndex)];
		}
		double motion[16];
		evaluateJointMotion1D(j.motion, qj, motion);
		mat4MulColumnMajor16(parentW, motion, jointFrameW);
	}
	double R[9];
	extractRotation(jointFrameW, R);
	axisW[0] = j.motion.axis[0];
	axisW[1] = j.motion.axis[1];
	axisW[2] = j.motion.axis[2];
	mulRVec3(R, axisW, axisW);
	originW[0] = jointFrameW[12];
	originW[1] = jointFrameW[13];
	originW[2] = jointFrameW[14];
}

bool jointAffectsTarget(const KinematicGraph& graph, int jointIdx, int targetLinkIdx)
{
	if (jointIdx < 0 || jointIdx >= static_cast<int>(graph.joints.size()))
	{
		return false;
	}
	const int child = graph.joints[static_cast<size_t>(jointIdx)].childLinkIdx;
	std::unordered_map<int, int> parentOf;
	for (const KinematicJoint& j : graph.joints)
	{
		parentOf[j.childLinkIdx] = j.parentLinkIdx;
	}
	int cur = targetLinkIdx;
	while (cur >= 0)
	{
		if (cur == child)
		{
			return true;
		}
		const auto it = parentOf.find(cur);
		if (it == parentOf.end())
		{
			break;
		}
		cur = it->second;
	}
	return false;
}

bool fillPositionJacobianFromLinkWorld(const KinematicGraph& graph, const double* q, const std::size_t qCount,
									   const int targetLinkIdx,
									   const std::vector<std::array<double, 16>>& linkWorld,
									   std::vector<double>& J_3xn, const JacobianOptions& opt)
{
	if (targetLinkIdx < 0 || targetLinkIdx >= static_cast<int>(graph.links.size()) ||
		linkWorld.size() != graph.links.size())
	{
		return false;
	}
	const int n = graph.dofCount();
	if (n <= 0)
	{
		return false;
	}
	const double* targetW = linkWorld[static_cast<size_t>(targetLinkIdx)].data();
	double pTarget[3] = {targetW[12], targetW[13], targetW[14]};

	J_3xn.assign(static_cast<size_t>(3 * n), 0.0);
	for (size_t ji = 0; ji < graph.joints.size(); ++ji)
	{
		const KinematicJoint& j = graph.joints[ji];
		if (j.qIndex < 0 || !j.motion.enabled || j.qIndex >= n)
		{
			continue;
		}
		if (!jointAffectsTarget(graph, static_cast<int>(ji), targetLinkIdx))
		{
			continue;
		}
		if (j.parentLinkIdx < 0 || j.parentLinkIdx >= static_cast<int>(linkWorld.size()))
		{
			continue;
		}
		const int col = j.qIndex;
		const double* parentW = linkWorld[static_cast<size_t>(j.parentLinkIdx)].data();
		double axisW[3];
		double originW[3];
		jointAxisOriginInWorld(j, parentW, q, qCount, axisW, originW);
		normalize3(axisW, opt.axisEps);

		if (j.motion.motionType == JointMotionType::Translate)
		{
			J_3xn[static_cast<size_t>(0 * n + col)] = axisW[0];
			J_3xn[static_cast<size_t>(1 * n + col)] = axisW[1];
			J_3xn[static_cast<size_t>(2 * n + col)] = axisW[2];
		}
		else
		{
			double r[3] = {pTarget[0] - originW[0], pTarget[1] - originW[1], pTarget[2] - originW[2]};
			double colVec[3];
			cross3(axisW, r, colVec);
			J_3xn[static_cast<size_t>(0 * n + col)] = colVec[0];
			J_3xn[static_cast<size_t>(1 * n + col)] = colVec[1];
			J_3xn[static_cast<size_t>(2 * n + col)] = colVec[2];
		}
	}
	return true;
}

bool fillOrientationRowsFromLinkWorld(const KinematicGraph& graph, const double* q, const std::size_t qCount,
									  const int targetLinkIdx,
									  const std::vector<std::array<double, 16>>& linkWorld,
									  std::vector<double>& J_6xn, const JacobianOptions& opt)
{
	const int n = graph.dofCount();
	if (n <= 0 || static_cast<int>(J_6xn.size()) < 3 * n || linkWorld.size() != graph.links.size())
	{
		return false;
	}
	J_6xn.resize(static_cast<size_t>(6 * n));
	const double w = opt.orientationWeight;
	for (size_t ji = 0; ji < graph.joints.size(); ++ji)
	{
		const KinematicJoint& j = graph.joints[ji];
		if (j.qIndex < 0 || !j.motion.enabled || j.qIndex >= n)
		{
			continue;
		}
		if (!jointAffectsTarget(graph, static_cast<int>(ji), targetLinkIdx))
		{
			continue;
		}
		if (j.parentLinkIdx < 0 || j.parentLinkIdx >= static_cast<int>(linkWorld.size()))
		{
			continue;
		}
		const int col = j.qIndex;
		const double* parentW = linkWorld[static_cast<size_t>(j.parentLinkIdx)].data();
		double axisW[3];
		double originWUnused[3];
		jointAxisOriginInWorld(j, parentW, q, qCount, axisW, originWUnused);
		normalize3(axisW, opt.axisEps);
		if (j.motion.motionType == JointMotionType::Translate)
		{
			J_6xn[static_cast<size_t>(3 * n + col)] = 0.0;
			J_6xn[static_cast<size_t>(4 * n + col)] = 0.0;
			J_6xn[static_cast<size_t>(5 * n + col)] = 0.0;
		}
		else
		{
			J_6xn[static_cast<size_t>(3 * n + col)] = axisW[0] * w;
			J_6xn[static_cast<size_t>(4 * n + col)] = axisW[1] * w;
			J_6xn[static_cast<size_t>(5 * n + col)] = axisW[2] * w;
		}
	}
	return true;
}
} // namespace

bool computePositionJacobianFromLinkWorld(const KinematicGraph& graph, const double* q, const std::size_t qCount,
										  const int targetLinkIdx,
										  const std::vector<std::array<double, 16>>& linkWorld,
										  std::vector<double>& J_3xn, const JacobianOptions& opt)
{
	return fillPositionJacobianFromLinkWorld(graph, q, qCount, targetLinkIdx, linkWorld, J_3xn, opt);
}

bool computePoseJacobianFromLinkWorld(const KinematicGraph& graph, const double* q, const std::size_t qCount,
									  const int targetLinkIdx,
									  const std::vector<std::array<double, 16>>& linkWorld,
									  std::vector<double>& J_6xn, const JacobianOptions& opt)
{
	if (!fillPositionJacobianFromLinkWorld(graph, q, qCount, targetLinkIdx, linkWorld, J_6xn, opt))
	{
		return false;
	}
	return fillOrientationRowsFromLinkWorld(graph, q, qCount, targetLinkIdx, linkWorld, J_6xn, opt);
}

bool computePositionJacobian(const KinematicGraph& graph, const double baseWorld[16], const double* q,
							 const std::size_t qCount, const int targetLinkIdx, std::vector<double>& J_3xn,
							 const JacobianOptions& opt)
{
	if (targetLinkIdx < 0 || targetLinkIdx >= static_cast<int>(graph.links.size()))
	{
		return false;
	}
	std::vector<std::array<double, 16>> linkWorld(graph.links.size());
	if (!forwardKinematicsTree(graph, baseWorld, q, qCount, reinterpret_cast<double(*)[16]>(linkWorld.data())))
	{
		return false;
	}
	return fillPositionJacobianFromLinkWorld(graph, q, qCount, targetLinkIdx, linkWorld, J_3xn, opt);
}

bool computePoseJacobian(const KinematicGraph& graph, const double baseWorld[16], const double* q,
						 const std::size_t qCount, const int targetLinkIdx, std::vector<double>& J_6xn,
						 const JacobianOptions& opt)
{
	if (targetLinkIdx < 0 || targetLinkIdx >= static_cast<int>(graph.links.size()))
	{
		return false;
	}
	std::vector<std::array<double, 16>> linkWorld(graph.links.size());
	if (!forwardKinematicsTree(graph, baseWorld, q, qCount, reinterpret_cast<double(*)[16]>(linkWorld.data())))
	{
		return false;
	}
	return computePoseJacobianFromLinkWorld(graph, q, qCount, targetLinkIdx, linkWorld, J_6xn, opt);
}

} // namespace kinematic_core
