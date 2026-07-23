#ifndef ROBOTKINEMATICS_CIRCULARARCGEOMETRY_H
#define ROBOTKINEMATICS_CIRCULARARCGEOMETRY_H

/// @file CircularArcGeometry.h
/// @brief 三点定圆与按弦长弧采样（mm，与 URDF/Qt 无关）

#include "robot_kinematics_global.h"

#include <cstddef>
#include <vector>

namespace robot_kinematics
{
struct Circle3Fit
{
	double center[3] = {0.0, 0.0, 0.0};
	double normal[3] = {0.0, 0.0, 1.0};
	double uAxis[3] = {1.0, 0.0, 0.0};
	double vAxis[3] = {0.0, 1.0, 0.0};
	double radius = 0.0;
	double theta0 = 0.0;
	double thetaVia = 0.0;
	double thetaEnd = 0.0;
};

/// 三点定圆；共线或半径过小返回 false。theta 相对 uAxis，且从 theta0 经 thetaVia 到 thetaEnd 单调
ROBOT_KINEMATICS_API bool fitCircle3Points(const double p0[3], const double p1[3], const double p2[3], Circle3Fit& out,
										  double minRadiusMm = 1e-3);

/// 按弦长采样弧（不含起点、含终点与 Via）；samples 钳位到 [minSamples, maxSamples]
/// outU01 可选，与每个采样点一一对应的弧参数 u∈(0,1]
ROBOT_KINEMATICS_API bool sampleArcByChord(const Circle3Fit& fit, double chordMm, int minSamples, int maxSamples,
										  std::vector<double>& outPositionsXyzFlat,
										  std::vector<double>* outU01 = nullptr);

/// 弧上参数 u∈[0,1]（经 Via）对应位置
ROBOT_KINEMATICS_API void pointOnArc(const Circle3Fit& fit, double u01, double outPos[3]);

/// Start→End 经 Via 的弧长（mm）
ROBOT_KINEMATICS_API double arcLengthMm(const Circle3Fit& fit);

} // namespace robot_kinematics

#endif // ROBOTKINEMATICS_CIRCULARARCGEOMETRY_H
