#pragma once

#include "robot_urdf_global.h"

#include <QVector>

/// 单条指令：在 durationSec 内将某一旋转关节转动 angleDeg（度）
struct ROBOT_URDF_API RobotSimulationCommand
{
	int jointIndex = 0; ///< 与当前文档 revolute 关节列表下标一致
	double angleDeg = 45.0;
	double durationSec = 2.0;
};

namespace RobotSimulation
{
inline constexpr double kPi = 3.14159265358979323846;
/// 与 MainWindow 中 QTimer 周期一致；后续可改为可配置
inline constexpr int kPlaybackTimerIntervalMs = 16;
} // namespace RobotSimulation
