#ifndef ROBOTSCENE_IKCANDIDATEPIPELINE_H
#define ROBOTSCENE_IKCANDIDATEPIPELINE_H

/// @file IkCandidatePipeline.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Seed→Solver→Filter→Rank→Budget；行为对齐原 Controller 编排

#include "robot_scene_global.h"

#include "IkCandidate.h"
#include "IkSolveTypes.h"
#include "RobotInstructionAxisConfiguration.h"
#include "UrdfIkMachineProfile.h"
#include "UrdfNumericalIk.h"

#include <QVector>
#include <functional>
#include <string>
#include <vector>

namespace RobotInstruction
{
/// 单种子臂位姿求解（由 Controller 注入，可含外轴联立）
using IkArmPoseSolveFn = std::function<std::vector<double>(
	const std::vector<double>& seed, bool preferNumericalOnly, bool raceAnalyticalAndNumerical,
	double seedAttractionWeight, std::string* failReason)>;

using IkManipulabilityFn = std::function<double(const std::vector<double>& q)>;
using IkCollisionCostFn = std::function<double(const std::vector<double>& q)>;

struct ROBOT_SCENE_API IkAnalyticalPoseInput
{
	std::string urdfPath;
	std::string ikLinkName;
	UrdfRobotLoader::UrdfPoseIkTarget poseTarget{};
	bool valid = false;
};

struct ROBOT_SCENE_API IkPipelineContext
{
	std::vector<double> qSeed;
	std::vector<double> classRef;
	MotionAxisConfiguration cfg{};
	IkSelectionPolicy policy{};
	std::vector<std::string> jointNames;
	int elbowOverride = -1;
	int wristOverride = -1;
	bool sphericalWrist = false;
	UrdfRobotLoader::UrdfIkMachineProfile machineProfile{};
	bool externalContextActive = false;
	bool dragProfile = false;
	QVector<double> limitLower;
	QVector<double> limitUpper;
	IkAnalyticalPoseInput analyticalInput{};
	IkArmPoseSolveFn armSolve;
	IkManipulabilityFn manipFn;
	IkCollisionCostFn collisionFn;
};

ROBOT_SCENE_API IkSolveResult runIkCandidatePipeline(const IkPipelineContext& ctx);

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKCANDIDATEPIPELINE_H
