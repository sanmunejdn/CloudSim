#ifndef ROBOTSCENE_IKRANKANDFILTER_H
#define ROBOTSCENE_IKRANKANDFILTER_H

/// @file IkRankAndFilter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轴配置硬门 / 腕连续 / Rank 比较与软罚

#include "robot_scene_global.h"

#include "IkCandidate.h"
#include "IkSolveTypes.h"
#include "RobotInstructionAxisConfiguration.h"

#include <QVector>
#include <functional>
#include <string>
#include <vector>

namespace RobotInstruction
{
ROBOT_SCENE_API int elbowSignFromQ(const std::vector<double>& q, int elbowIdx);

ROBOT_SCENE_API int softCfgMismatchScore(const std::vector<double>& q, const std::vector<std::string>& jointNames,
										 const std::vector<double>& classRef, const MotionAxisConfiguration& cfg,
										 int elbowOv, int wristOv);

ROBOT_SCENE_API double softLimitProximityPenalty(const std::vector<double>& q, const QVector<double>& lower,
												 const QVector<double>& upper);

/// 硬 class / 腕连续过滤后取最优；空池写 failKind/message
ROBOT_SCENE_API IkSolveResult rankAndSelectIkCandidates(std::vector<IkCandidate>& converged, const IkSelectionPolicy& policy,
														const MotionAxisConfiguration& cfg,
														const std::vector<std::string>& jointNames,
														const std::vector<double>& classRef, bool sphericalWrist,
														int elbowOverride, int wristOverride, bool budgetExhausted,
														const std::string& lastSeedFail,
														const std::function<double(const std::vector<double>&)>& manipFn);

ROBOT_SCENE_API bool isIkCandidateBetter(const IkCandidate& c, const IkCandidate& best, IkRankMode rankMode);

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKRANKANDFILTER_H
