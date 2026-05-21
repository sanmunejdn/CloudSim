#pragma once

#include "robot_scene_global.h"

#include "BackendFollowMath.h"

#include <RigidTransform.h>

#include <json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace RobotCoordinate
{

/// Extension keys on motion instructions (PTP/LINE).
inline constexpr const char* kExtMotionToolFrameId = "motion.tool.frameId";
inline constexpr const char* kExtMotionUserFrameId = "motion.user.frameId";
inline constexpr const char* kExtMotionTargetFrame = "motion.target.frame";
inline constexpr const char* kExtContextToolFrameMat4 = "context.toolFrameMat4";
inline constexpr const char* kExtContextCapturedTcpLinkName = "context.capturedTcpLinkName";

struct ROBOT_SCENE_API RobotRigidFrame
{
	double positionMm[3]{ 0.0, 0.0, 0.0 };
	double eulerDeg[3]{ 0.0, 0.0, 0.0 };
};

struct ROBOT_SCENE_API RobotToolFrame
{
	std::string id;
	std::string name;
	/// T_flange_tool：相对 URDF 法兰连杆坐标系；空 flangeLinkName 时用 RobotCoordinateFrameSet::flangeLinkName。
	RobotRigidFrame T_flange_tool;
	std::string flangeLinkName;
};

struct ROBOT_SCENE_API RobotUserFrame
{
	std::string id;
	std::string name;
	RobotRigidFrame T_base_user;
};

struct ROBOT_SCENE_API RobotCoordinateFrameSet
{
	std::string flangeLinkName;
	std::vector<RobotToolFrame> toolFrames;
	std::string activeToolFrameId;
	std::vector<RobotUserFrame> userFrames;
	std::string activeUserFrameId;
	bool showToolFrameInScene = true;
	bool showUserFramesInScene = true;
};

ROBOT_SCENE_API std::string makeToolFrameId();
/// Next id unique within \p set (bumps global counter from existing \c TFR_* ids).
ROBOT_SCENE_API std::string allocateUniqueToolFrameId(const RobotCoordinateFrameSet& set);
/// Reassign duplicate/empty tool ids after JSON load or legacy projects.
ROBOT_SCENE_API void ensureUniqueToolFrameIds(RobotCoordinateFrameSet& set);
ROBOT_SCENE_API std::string makeUserFrameId();

ROBOT_SCENE_API RobotRigidFrame identityRigidFrame();
/// Legacy JSON/UI; prefer \ref rigidTransformFromFrame for new code.
ROBOT_SCENE_API BackendMat4 frameToMat4(const RobotRigidFrame& frame);
ROBOT_SCENE_API RobotRigidFrame mat4ToFrame(const BackendMat4& m);

ROBOT_SCENE_API engine::RigidTransform rigidTransformFromFrame(const RobotRigidFrame& frame);
ROBOT_SCENE_API RobotRigidFrame frameFromRigidTransform(const engine::RigidTransform& t);
ROBOT_SCENE_API engine::RigidTransform rigidTransformFromBackendMat4(const BackendMat4& m);
ROBOT_SCENE_API BackendMat4 backendMat4FromRigidTransform(const engine::RigidTransform& t);

ROBOT_SCENE_API engine::RigidTransform targetRigidTransformFromPose(
	double px,
	double py,
	double pz,
	double ex,
	double ey,
	double ez);

/// Instruction pose/euler in robot base: T_base_target (tool frame origin for this point's tool).
/// If T_flange_tool is identity, target coincides with flange; otherwise tool tip/origin.
/// Legacy pose+euler path; prefer \ref targetRigidTransformFromPose and instruction quat extensions.
ROBOT_SCENE_API BackendMat4 targetInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez);
inline BackendMat4 toolOriginInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez)
{
	return targetInBaseFromPose(px, py, pz, ex, ey, ez);
}

/// Legacy name; same as targetInBaseFromPose.
ROBOT_SCENE_API BackendMat4 tcpInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez);

ROBOT_SCENE_API void poseEulerFromTargetInBase(const BackendMat4& T_base_target, double outPos[3], double outEulerDeg[3]);
/// Legacy name.
ROBOT_SCENE_API void poseEulerFromTcpInBase(const BackendMat4& T_base_tcp, double outPos[3], double outEulerDeg[3]);

/// Pre-IK transform (sole tool entry): T_base_flange = T_base_target * inv(T_flange_tool).
ROBOT_SCENE_API BackendMat4 flangeTargetFromToolOriginInBase(
	const BackendMat4& T_base_target,
	const BackendMat4& T_flange_tool);

/// Legacy name.
ROBOT_SCENE_API BackendMat4 flangeTargetFromBaseTcpAndTool(const BackendMat4& T_base_tcp, const BackendMat4& T_flange_tool);

/// Teach/FK check: T_base_target = T_base_flange * T_flange_tool.
ROBOT_SCENE_API BackendMat4 targetInBaseFromFlange(const BackendMat4& T_base_flange, const BackendMat4& T_flange_tool);
ROBOT_SCENE_API BackendMat4 tcpInBaseFromUserTcp(const BackendMat4& T_base_user, const BackendMat4& T_user_tcp);
ROBOT_SCENE_API BackendMat4 tcpInUserFromBaseTcp(const BackendMat4& T_base_user, const BackendMat4& T_base_tcp);

ROBOT_SCENE_API std::string encodeMat4Csv(const BackendMat4& m);
ROBOT_SCENE_API bool parseMat4Csv(const std::string& csv, BackendMat4& out);

ROBOT_SCENE_API std::string effectiveFlangeLinkName(const RobotCoordinateFrameSet& set, const RobotToolFrame& tool);
ROBOT_SCENE_API const RobotToolFrame* findToolFrameById(const RobotCoordinateFrameSet& set, const std::string& id);
ROBOT_SCENE_API const RobotToolFrame* activeToolFrame(const RobotCoordinateFrameSet& set);
ROBOT_SCENE_API const RobotUserFrame* findUserFrameById(const RobotCoordinateFrameSet& set, const std::string& id);
ROBOT_SCENE_API const RobotUserFrame* activeUserFrame(const RobotCoordinateFrameSet& set);

ROBOT_SCENE_API RobotCoordinateFrameSet makeDefaultFrameSet(const std::string& defaultFlangeLinkName);

/// Resolve tool for instruction extensions: motion.tool.frameId, then frozen context.toolFrameMat4, then active.
ROBOT_SCENE_API const RobotToolFrame* resolveToolFrameForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext);

ROBOT_SCENE_API BackendMat4 toolMat4ForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext);

/// Resolve user frame: motion.user.frameId, else active user frame.
ROBOT_SCENE_API const RobotUserFrame* resolveUserFrameForExtension(
	const RobotCoordinateFrameSet& set,
	const std::unordered_map<std::string, std::string>& ext);

/// True when property panel / display should show TCP in user frame (motion.target.frame = user or legacy active_user).
ROBOT_SCENE_API bool instructionTargetDisplayUsesUserFrame(
	const std::unordered_map<std::string, std::string>& ext);

ROBOT_SCENE_API void writeCoordinateFrameSetToJson(const RobotCoordinateFrameSet& set, nlohmann::json& out);
ROBOT_SCENE_API bool readCoordinateFrameSetFromJson(const nlohmann::json& in, RobotCoordinateFrameSet& out);

} // namespace RobotCoordinate
