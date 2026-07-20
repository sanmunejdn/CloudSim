#ifndef ROBOTSCENE_ROBOTCOORDINATEFRAMES_H
#define ROBOTSCENE_ROBOTCOORDINATEFRAMES_H

/// @file RobotCoordinateFrames.h
/// @brief 运动指令扩展键（PTP/LINE）

#include "robot_scene_global.h"

#include "BackendFollowMath.h"

#include <string>
#include <unordered_map>
#include <vector>

#include <RigidTransform.h>
#include <json.hpp>

namespace RobotCoordinate
{
/// 运动指令扩展键（PTP/LINE）
inline constexpr const char* kExtMotionToolFrameId = "motion.tool.frameId";
inline constexpr const char* kExtMotionUserFrameId = "motion.user.frameId";
inline constexpr const char* kExtMotionTargetFrame = "motion.target.frame";
inline constexpr const char* kExtContextToolFrameMat4 = "context.toolFrameMat4";
inline constexpr const char* kExtContextCapturedTcpLinkName = "context.capturedTcpLinkName";

struct ROBOT_SCENE_API RobotRigidFrame
{
	double positionMm[3]{0.0, 0.0, 0.0};
	double eulerDeg[3]{0.0, 0.0, 0.0};
};

struct ROBOT_SCENE_API RobotToolFrame
{
	std::string id;
	std::string name;
	/// T_flange_tool；空 flangeLinkName 时用 RobotCoordinateFrameSet::flangeLinkName
	RobotRigidFrame T_flange_tool;
	std::string flangeLinkName;
	bool showInScene = true;
};

struct ROBOT_SCENE_API RobotUserFrame
{
	std::string id;
	std::string name;
	RobotRigidFrame T_base_user;
	bool showInScene = true;
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
/// 在 set 内分配唯一 TFR_* id
ROBOT_SCENE_API std::string allocateUniqueToolFrameId(const RobotCoordinateFrameSet& set);
/// JSON/旧工程加载后去重 tool id
ROBOT_SCENE_API void ensureUniqueToolFrameIds(RobotCoordinateFrameSet& set);
ROBOT_SCENE_API std::string makeUserFrameId();

ROBOT_SCENE_API RobotRigidFrame identityRigidFrame();
/// 旧 JSON/UI；新代码用 rigidTransformFromFrame
ROBOT_SCENE_API BackendMat4 frameToMat4(const RobotRigidFrame& frame);
ROBOT_SCENE_API RobotRigidFrame mat4ToFrame(const BackendMat4& m);

ROBOT_SCENE_API engine::RigidTransform rigidTransformFromFrame(const RobotRigidFrame& frame);
ROBOT_SCENE_API RobotRigidFrame frameFromRigidTransform(const engine::RigidTransform& t);
ROBOT_SCENE_API engine::RigidTransform rigidTransformFromBackendMat4(const BackendMat4& m);
ROBOT_SCENE_API BackendMat4 backendMat4FromRigidTransform(const engine::RigidTransform& t);

ROBOT_SCENE_API engine::RigidTransform targetRigidTransformFromPose(double px, double py, double pz, double ex,
																	double ey, double ez);

/// 基座下目标位姿 T_base_target；T_flange_tool 为单位时即法兰
ROBOT_SCENE_API BackendMat4 targetInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez);
inline BackendMat4 toolOriginInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez)
{
	return targetInBaseFromPose(px, py, pz, ex, ey, ez);
}

ROBOT_SCENE_API BackendMat4 tcpInBaseFromPose(double px, double py, double pz, double ex, double ey, double ez);

ROBOT_SCENE_API void poseEulerFromTargetInBase(const BackendMat4& T_base_target, double outPos[3],
											   double outEulerDeg[3]);
ROBOT_SCENE_API void poseEulerFromTcpInBase(const BackendMat4& T_base_tcp, double outPos[3], double outEulerDeg[3]);

/// IK 前：T_base_flange = T_base_target * inv(T_flange_tool)
ROBOT_SCENE_API BackendMat4 flangeTargetFromToolOriginInBase(const BackendMat4& T_base_target,
															 const BackendMat4& T_flange_tool);

ROBOT_SCENE_API BackendMat4 flangeTargetFromBaseTcpAndTool(const BackendMat4& T_base_tcp,
														   const BackendMat4& T_flange_tool);

/// 示教/FK：T_base_target = T_base_flange * T_flange_tool
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

/// motion.tool.frameId → context.toolFrameMat4 → active
ROBOT_SCENE_API const RobotToolFrame*
resolveToolFrameForExtension(const RobotCoordinateFrameSet& set,
							 const std::unordered_map<std::string, std::string>& ext);

ROBOT_SCENE_API BackendMat4 toolMat4ForExtension(const RobotCoordinateFrameSet& set,
												 const std::unordered_map<std::string, std::string>& ext);

ROBOT_SCENE_API const RobotUserFrame*
resolveUserFrameForExtension(const RobotCoordinateFrameSet& set,
							 const std::unordered_map<std::string, std::string>& ext);

/// motion.target.frame=user 时在属性面板显示用户系 TCP
ROBOT_SCENE_API bool instructionTargetDisplayUsesUserFrame(const std::unordered_map<std::string, std::string>& ext);

ROBOT_SCENE_API void writeCoordinateFrameSetToJson(const RobotCoordinateFrameSet& set, nlohmann::json& out);
ROBOT_SCENE_API bool readCoordinateFrameSetFromJson(const nlohmann::json& in, RobotCoordinateFrameSet& out);

} // namespace RobotCoordinate

#endif // ROBOTSCENE_ROBOTCOORDINATEFRAMES_H
