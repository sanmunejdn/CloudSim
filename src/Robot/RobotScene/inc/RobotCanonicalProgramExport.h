#pragma once

#include "RobotInstructionController.h"
#include "RobotInstructionModel.h"
#include "RobotProgramCatalog.h"
#include "RobotCoordinateFrames.h"
#include "robot_scene_global.h"

#include <json.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace RobotCanonicalExport
{

inline constexpr const char* kFormatId = "cloudsim.program_export";
inline constexpr int kSchemaVersion = 1;

enum class ROBOT_SCENE_API CanonicalExportLayout
{
	NestedTree = 0,
	FlatMotion
};

struct ROBOT_SCENE_API InstructionRuntimeResolveContext
{
	int robotInstanceIndex = 0;
	std::string robotSceneBackendId;
	std::string urdfPath;
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames = nullptr;
	const RobotInstruction::Controller* instructionController = nullptr;
};

struct ROBOT_SCENE_API FlatMotionRef
{
	int flatIndex = 0;
	std::string instructionId;
	std::vector<size_t> programStepPath;
	int pointIndex = 0;
};

struct ROBOT_SCENE_API CanonicalProgramExportV1
{
	std::string exportedAtUtc;
	std::string programId;
	std::string programName;
	CanonicalExportLayout layout = CanonicalExportLayout::NestedTree;
	int robotInstanceIndex = 0;
	std::string robotSceneBackendId;
	std::string urdfPath;
	RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
	nlohmann::json instructions = nlohmann::json::array();
	std::vector<FlatMotionRef> flatMotionSequence;
};

ROBOT_SCENE_API bool buildCanonicalExportV1(
	const RobotInstruction::RobotProgram& program,
	const InstructionRuntimeResolveContext& ctx,
	CanonicalExportLayout layout,
	bool includePathPlanMetadata,
	const std::vector<RobotInstruction::PlanResult>* motionPlansInDfsOrder,
	CanonicalProgramExportV1& out,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool writeCanonicalExportV1ToJson(
	const CanonicalProgramExportV1& doc,
	std::string& outJson,
	std::string* errMsg = nullptr);

} // namespace RobotCanonicalExport
