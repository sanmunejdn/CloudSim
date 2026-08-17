#ifndef ROBOTKINEMATICS_SERIALLINKKINEMATICS_H
#define ROBOTKINEMATICS_SERIALLINKKINEMATICS_H

/// @file SerialLinkKinematics.h
/// @brief 单节改进 DH（Craig）：FK + 位置 DLS IK（legacy：仅无 URDF 时回退；生产有 URDF 走 RobotUrdf）

#include "robot_kinematics_global.h"

#include <cstddef>
#include <vector>

namespace robot_kinematics
{
/// 单节改进 DH（Craig《机器人学导论》）：\n
/// 旋转：θ_i = thetaOffset + q[jointIndex]；棱柱：d_i = d + q[jointIndex]\n
/// jointIndex < 0 表示该节无关节变量
/// @deprecated 有 URDF 时勿新建 DH 行；请用 UrdfRobotLoader / UrdfNumericalIk
struct DhRow
{
	double a = 0.0;
	double alpha = 0.0;
	double d = 0.0;
	double thetaOffset = 0.0;
	int jointIndex = -1;
	/// true：变量进 d（mm）；false：变量进 θ（rad）
	bool isPrismatic = false;
};

/// 正运动学：末端坐标系相对基座的 4×4 齐次矩阵（列主序）
ROBOT_KINEMATICS_API bool fkSerialDh(const std::vector<DhRow>& rows, const std::vector<double>& q,
									 double T_end4x4_colMajor[16]);

/// 末端位置（齐次矩阵第四列前三个分量）
ROBOT_KINEMATICS_API bool endEffectorPosition(const std::vector<DhRow>& rows, const std::vector<double>& q,
											  double posOut[3]);

/// 数值逆运动学（仅位置）：阻尼最小二乘，使末端接近 targetPos。\n
/// qInOut 长度须等于关节数；初值作为迭代起点。\n
/// 成功返回 true（位置误差小于 positionTolerance）；iterationsUsed 可选
ROBOT_KINEMATICS_API bool ikPositionDampedLeastSquares(const std::vector<DhRow>& rows, const double targetPos[3],
													   std::vector<double>& qInOut, int maxIterations,
													   double positionTolerance, double lambdaDamping,
													   int* iterationsUsed = nullptr);

/// 解析位置雅可比 3×n（列主序按行存储：J[r*n+c]）；供单测对比有限差分
ROBOT_KINEMATICS_API bool positionJacobianAnalytic(const std::vector<DhRow>& rows, const std::vector<double>& q,
												   std::vector<double>& J_3xn);

/// 由 DH 行推导关节数量（jointIndex 最大值 + 1）
ROBOT_KINEMATICS_API std::size_t jointCountFromDhRows(const std::vector<DhRow>& rows);

} // namespace robot_kinematics

#endif // ROBOTKINEMATICS_SERIALLINKKINEMATICS_H
