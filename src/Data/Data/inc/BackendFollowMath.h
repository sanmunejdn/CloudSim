#ifndef DATA_BACKENDFOLLOWMATH_H
#define DATA_BACKENDFOLLOWMATH_H

/// @file BackendFollowMath.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 列主序 4×4（与 osg::Matrixd 同序）；新链路优先 engine::RigidTransform

#include "data_global.h"

#include <cmath>

struct BackendVec3;

/// 列主序 4×4（与 osg::Matrixd 同序，经 Adapters↔RigidTransform）；禁止按 Eigen 直读 v[12..14]
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

/// 世界矩阵：pose=模型原点世界坐标；权威实现 engine::rigidTransformFromBackendPoseEuler
DATA_EXPORT BackendMat4 backend_world_mat_from_pose(const BackendVec3& pose, const BackendVec3& rotationEulerDeg);

/// 从世界矩阵反解 pose 平移与欧拉角（度）
DATA_EXPORT void backend_pose_euler_from_world_mat(const BackendMat4& world, BackendVec3& outPose,
												   BackendVec3& outEulerDeg);

/// v2 别名：composeWorldMatrix / decomposeWorldMatrix（UI 分解视图，非独立存储）
inline BackendMat4 composeWorldMatrix(const BackendVec3& pose, const BackendVec3& rotationEulerDeg)
{
	return backend_world_mat_from_pose(pose, rotationEulerDeg);
}
inline void decomposeWorldMatrix(const BackendMat4& world, BackendVec3& outPose, BackendVec3& outEulerDeg)
{
	backend_pose_euler_from_world_mat(world, outPose, outEulerDeg);
}

/// 无模型中心项的 T(trans)*R(euler)，提取平移与欧拉角
DATA_EXPORT void backend_trans_euler_from_rigid_mat(const BackendMat4& m, BackendVec3& outTrans,
													BackendVec3& outEulerDeg);

DATA_EXPORT bool backend_mat4_nearly_equal(const BackendMat4& a, const BackendMat4& b, double absEps = 1e-7);

/// 只换平移，旋转保持原矩阵（pose.x 提交不得欧拉重写姿态）
DATA_EXPORT BackendMat4 backend_world_mat_replace_translation(const BackendMat4& world, const BackendVec3& pose);

#endif // DATA_BACKENDFOLLOWMATH_H
