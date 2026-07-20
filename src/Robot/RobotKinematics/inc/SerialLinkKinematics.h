#ifndef ROBOTKINEMATICS_SERIALLINKKINEMATICS_H
#define ROBOTKINEMATICS_SERIALLINKKINEMATICS_H

/// @file SerialLinkKinematics.h
/// @brief 单节改进 DH（Craig《机器人学导论》）：\n

#include "robot_kinematics_global.h"

#include <cstddef>
#include <vector>

namespace robot_kinematics
{
/// 单节改进 DH（Craig《机器人学导论》）：\n
/// 其中 θ_i = thetaOffset + q[jointIndex]（revolute），jointIndex < 0 表示该节无关节变量（θ 仅用 thetaOffset）
struct DhRow
{
	double a = 0.0;
	double alpha = 0.0;
	double d = 0.0;
	double thetaOffset = 0.0;
	int jointIndex = -1;
};

/// 正运动学：末端坐标系相对基座的 4×4 齐次矩阵（列主序，与 OpenGL/OSG 一致：column * 4 + row）
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

/// 由 DH 行推导关节数量（jointIndex 最大值 + 1）
ROBOT_KINEMATICS_API std::size_t jointCountFromDhRows(const std::vector<DhRow>& rows);

} // namespace robot_kinematics

#endif // ROBOTKINEMATICS_SERIALLINKKINEMATICS_H
