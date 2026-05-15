#pragma once

#include "data_global.h"

#include <cmath>

struct BackendVec3;

/// Column-major 4x4 (same storage order as \c osg::Matrixd): transforms column vectors \(p' = M p\).
struct DATA_EXPORT BackendMat4
{
	double v[16]{};

	static BackendMat4 identity();
	static BackendMat4 translate(double tx, double ty, double tz);
	/// Same rotation convention as \c backendvisual_math::eulerDegToQuat: R = Rz*Ry*Rx (degrees).
	static BackendMat4 rotateEulerDeg(double exDeg, double eyDeg, double ezDeg);
};

DATA_EXPORT bool backend_mat4_multiply(const BackendMat4& a, const BackendMat4& b, BackendMat4& out);
/// Rigid transform inverse; returns false if not invertible.
DATA_EXPORT bool backend_mat4_invert_rigid(const BackendMat4& m, BackendMat4& out);

/// Outer-branch world matrix: T(center+pose) * R(rotationEulerDeg) (matches PointCloud/Mesh backend visual).
DATA_EXPORT BackendMat4 backend_world_mat_from_pose(
	const BackendVec3& modelCenter, const BackendVec3& pose, const BackendVec3& rotationEulerDeg);

/// From world matrix extract backend pose offset and Euler degrees (deg) for the same outer-branch convention.
DATA_EXPORT void backend_pose_euler_from_world_mat(
	const BackendMat4& world, const BackendVec3& modelCenter, BackendVec3& outPose, BackendVec3& outEulerDeg);

/// Rigid matrix as T(trans) * R(euler) with no model-center term: extracts translation and Euler (deg).
DATA_EXPORT void backend_trans_euler_from_rigid_mat(const BackendMat4& m, BackendVec3& outTrans, BackendVec3& outEulerDeg);
