#ifndef ROBOTSCENE_ROBOTJOINTWRAP_H
#define ROBOTSCENE_ROBOTJOINTWRAP_H

/// @file RobotJointWrap.h
/// @brief 回转关节折到种子最近圈；越限则失败（桌面/网页共用）

#include "robot_scene_global.h"

#include "RobotInstructionController.h"

#include <QVector>
#include <string>
#include <vector>

ROBOT_SCENE_API bool normalizeJointRevolutionsToReference(std::vector<double>& q, const std::vector<double>& ref,
														  const QVector<double>& lowerRad,
														  const QVector<double>& upperRad);

ROBOT_SCENE_API std::string describeJointNormalizeFailure(const std::vector<double>& q, const std::vector<double>& ref,
														  const QVector<double>& lowerRad,
														  const QVector<double>& upperRad);

ROBOT_SCENE_API void alignTrajectoryAfterTargetNormalize(RobotInstruction::PlanResult& plan,
														 const std::vector<double>& targetsBeforeNormalize);

/// 折圈写入 plan；失败则 ok=false 并填中文 summary
ROBOT_SCENE_API bool applyJointWrapToPlan(RobotInstruction::PlanResult& plan, const std::vector<double>& seedRef,
										  const QVector<double>& lowerRad, const QVector<double>& upperRad);

#endif
