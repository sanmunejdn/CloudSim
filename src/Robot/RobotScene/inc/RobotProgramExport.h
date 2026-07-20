#ifndef ROBOTSCENE_ROBOTPROGRAMEXPORT_H
#define ROBOTSCENE_ROBOTPROGRAMEXPORT_H

/// @file RobotProgramExport.h
/// @brief Build export rows from motion instructions and matching plan results (same order as collectMotionInstructions).

#include "robot_scene_global.h"

#include "RobotInstructionController.h"
#include "RobotInstructionModel.h"

#include <string>
#include <vector>

namespace RobotProgramExport
{
struct ROBOT_SCENE_API MotionPointExport
{
	int pointIndex = 0;
	std::string type;
	double posBaseMm[3]{0.0, 0.0, 0.0};
	double eulerBaseDeg[3]{0.0, 0.0, 0.0};
	std::string toolFrameId;
	std::string userFrameId;
	std::vector<double> jointRad;
	bool ikOk = false;
	std::string ikError;
};

struct ROBOT_SCENE_API RobotProgramExportResult
{
	std::string robotSceneBackendId;
	std::string urdfPath;
	std::vector<MotionPointExport> points;
};

/// Build export rows from motion instructions and matching plan results (same order as collectMotionInstructions).
ROBOT_SCENE_API bool buildExportResult(const std::vector<const RobotInstruction::Base*>& motions,
									   const std::vector<RobotInstruction::PlanResult>& plans,
									   const std::string& robotSceneBackendId, const std::string& urdfPath,
									   RobotProgramExportResult& out, std::string* errMsg = nullptr);

ROBOT_SCENE_API bool writeExportResultToJson(const RobotProgramExportResult& result, std::string& outJson,
											 std::string* errMsg = nullptr);

ROBOT_SCENE_API bool writeExportResultToCsv(const RobotProgramExportResult& result, std::string& outCsv,
											std::string* errMsg = nullptr);

} // namespace RobotProgramExport

#endif // ROBOTSCENE_ROBOTPROGRAMEXPORT_H
