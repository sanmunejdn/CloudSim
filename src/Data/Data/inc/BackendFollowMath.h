#pragma once

#include "data_global.h"

#include <cmath>

struct BackendVec3;

/// 列主序 4×4（与 osg::Matrixd 同序）；新链路优先 engine::RigidTransform
struct DATA_EXPORT BackendMat4
{
	double v[16]{};

	static BackendMat4 identity();
	static BackendMat4 translate(double tx, double ty, double tz);
	/// 旋转约定 R=Rz*Ry*Rx（度），与 BackendVisualMath 一致
	static BackendMat4 rotateEulerDeg(double exDeg, double eyDeg, double ezDeg);
};

DATA_EXPORT bool backend_mat4_multiply(const BackendMat4& a, const BackendMat4& b, BackendMat4& out);
/// 刚体逆变换；不可逆返回 false
DATA_EXPORT bool backend_mat4_invert_rigid(const BackendMat4& m, BackendMat4& out);

/// 世界矩阵 T(pose)*R(euler)；geometry 存世界坐标，pose 为相对 identity 的刚体偏移
DATA_EXPORT BackendMat4 backend_world_mat_from_pose(const BackendVec3& pose, const BackendVec3& rotationEulerDeg);

/// 从世界矩阵反解 pose 平移与欧拉角（度）
DATA_EXPORT void backend_pose_euler_from_world_mat(const BackendMat4& world, BackendVec3& outPose, BackendVec3& outEulerDeg);

/// 无模型中心项的 T(trans)*R(euler)，提取平移与欧拉角
DATA_EXPORT void backend_trans_euler_from_rigid_mat(const BackendMat4& m, BackendVec3& outTrans, BackendVec3& outEulerDeg);
