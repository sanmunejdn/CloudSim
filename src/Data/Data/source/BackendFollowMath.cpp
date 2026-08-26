/// @file BackendFollowMath.cpp
/// @brief Follow 矩阵数学

#include "BackendFollowMath.h"

#include "BackendDataBase.h"

#include <Adapters.h>
#include <BackendWorldPose.h>
#include <RigidTransform.h>

#include <cmath>

namespace
{
engine::RigidTransform rigidFromBackendMat4(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = m.v[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

BackendMat4 backendMat4FromRigid(const engine::RigidTransform& rt)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(rt);
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}
} // namespace

BackendMat4 BackendMat4::identity()
{
	return backendMat4FromRigid(engine::RigidTransform::identity());
}

BackendMat4 BackendMat4::translate(double tx, double ty, double tz)
{
	// 必须走 Adapters：BackendMat4 与 OSG 同序，平移不在 Eigen 的 v[12..14]
	return backendMat4FromRigid(engine::RigidTransform::fromTranslationEulerDeg(tx, ty, tz, 0.0, 0.0, 0.0));
}

BackendMat4 BackendMat4::rotateEulerDeg(double exDeg, double eyDeg, double ezDeg)
{
	return backendMat4FromRigid(engine::RigidTransform::fromTranslationEulerDeg(0.0, 0.0, 0.0, exDeg, eyDeg, ezDeg));
}

bool backend_mat4_multiply(const BackendMat4& a, const BackendMat4& b, BackendMat4& out)
{
	out = backendMat4FromRigid(rigidFromBackendMat4(a).composeColumn(rigidFromBackendMat4(b)));
	return true;
}

bool backend_mat4_invert_rigid(const BackendMat4& m, BackendMat4& out)
{
	out = backendMat4FromRigid(rigidFromBackendMat4(m).inverse());
	return true;
}

bool backend_mat4_is_nearly_rigid(const BackendMat4& m, const double absEps)
{
	const auto colLen = [&](const int c) -> double
	{
		const double x = m.v[c * 4 + 0];
		const double y = m.v[c * 4 + 1];
		const double z = m.v[c * 4 + 2];
		return std::sqrt(x * x + y * y + z * z);
	};
	const auto dotCols = [&](const int a, const int b) -> double
	{
		return m.v[a * 4 + 0] * m.v[b * 4 + 0] + m.v[a * 4 + 1] * m.v[b * 4 + 1] +
			   m.v[a * 4 + 2] * m.v[b * 4 + 2];
	};
	for (int c = 0; c < 3; ++c)
	{
		if (std::abs(colLen(c) - 1.0) > absEps)
		{
			return false;
		}
	}
	if (std::abs(dotCols(0, 1)) > absEps || std::abs(dotCols(0, 2)) > absEps || std::abs(dotCols(1, 2)) > absEps)
	{
		return false;
	}
	if (std::abs(m.v[3]) > absEps || std::abs(m.v[7]) > absEps || std::abs(m.v[11]) > absEps ||
		std::abs(m.v[15] - 1.0) > absEps)
	{
		return false;
	}
	// P3-1: 镜像矩阵（det=-1，列仍单位正交）需额外排除——叉积方向检查
	// dot(cross(c0,c1),c2) > 0 保证右手系
	const double cx = m.v[0 * 4 + 1] * m.v[1 * 4 + 2] - m.v[0 * 4 + 2] * m.v[1 * 4 + 1];
	const double cy = m.v[0 * 4 + 2] * m.v[1 * 4 + 0] - m.v[0 * 4 + 0] * m.v[1 * 4 + 2];
	const double cz = m.v[0 * 4 + 0] * m.v[1 * 4 + 1] - m.v[0 * 4 + 1] * m.v[1 * 4 + 0];
	const double det3 = cx * m.v[2 * 4 + 0] + cy * m.v[2 * 4 + 1] + cz * m.v[2 * 4 + 2];
	if (det3 <= absEps)
	{
		return false;
	}
	return true;
}

BackendMat4 backend_world_mat_from_pose(const BackendVec3& pose, const BackendVec3& rotationEulerDeg)
{
	const engine::RigidTransform rt = engine::rigidTransformFromBackendPoseEuler(
		pose.x, pose.y, pose.z, rotationEulerDeg.x, rotationEulerDeg.y, rotationEulerDeg.z);
	return backendMat4FromRigid(rt);
}

void backend_pose_euler_from_world_mat(const BackendMat4& world, BackendVec3& outPose, BackendVec3& outEulerDeg)
{
	const engine::RigidTransform rt = rigidFromBackendMat4(world);
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	engine::backendPoseEulerFromRigidTransform(rt, px, py, pz, ex, ey, ez);
	outPose.x = px;
	outPose.y = py;
	outPose.z = pz;
	outEulerDeg.x = ex;
	outEulerDeg.y = ey;
	outEulerDeg.z = ez;
}

void backend_trans_euler_from_rigid_mat(const BackendMat4& m, BackendVec3& outTrans, BackendVec3& outEulerDeg)
{
	backend_pose_euler_from_world_mat(m, outTrans, outEulerDeg);
}

bool backend_mat4_nearly_equal(const BackendMat4& a, const BackendMat4& b, double absEps)
{
	for (int i = 0; i < 16; ++i)
	{
		if (std::abs(a.v[i] - b.v[i]) > absEps)
		{
			return false;
		}
	}
	return true;
}

BackendMat4 backend_world_mat_replace_translation(const BackendMat4& world, const BackendVec3& pose)
{
	engine::RigidTransform rt = rigidFromBackendMat4(world);
	rt.setTranslationMm(Eigen::Vector3d(pose.x, pose.y, pose.z));
	return backendMat4FromRigid(rt);
}
