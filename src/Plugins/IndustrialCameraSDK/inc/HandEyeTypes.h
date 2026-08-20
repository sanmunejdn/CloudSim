#ifndef INDUSTRIALCAMERASDK_HANDEYETYPES_H
#define INDUSTRIALCAMERASDK_HANDEYETYPES_H

/// @file HandEyeTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 手眼标定类型与 Ensemble API

#include "CameraTypes.h"

#include <string>
#include <vector>

namespace industrial_camera
{

enum class HandEyeMountMode
{
	EyeInHand = 0,
	EyeToHand = 1
};

enum class HandEyeMethod
{
	Tsai = 0,
	Park,
	Horaud,
	Andreff,
	Daniilidis,
	MechOfficial, // 梅卡官方（不参与 Eigen 循环）
	Count
};

struct HandEyeSample
{
	Mat4 T_base_flange{}; // 机器人
	Mat4 T_cam_board{};	  // 标定板在相机系
};

struct HandEyeMethodScore
{
	HandEyeMethod method = HandEyeMethod::Tsai;
	std::string name;
	bool ok = false;
	double rotErrRad = 0.0;
	double transErrMm = 0.0;
	double score = 0.0;
	Mat4 T{};
	std::string error;
};

struct HandEyeSolveParams
{
	HandEyeMountMode mode = HandEyeMountMode::EyeInHand;
	double weightRot = 1.0;
	double weightTrans = 1.0;
	double workspaceScaleMm = 1000.0;
	double minRelRotDeg = 5.0;	 // 过小相对转角丢弃
	double maxRelRotDeg = 175.0; // 近 π 丢弃
};

struct HandEyeResult
{
	bool ok = false;
	HandEyeMethod bestMethod = HandEyeMethod::Tsai;
	std::string bestMethodName;
	Mat4 T_best{};
	int inlierMotionPairs = 0;
	std::vector<HandEyeMethodScore> scores;
	std::string error;
};

INDUSTRIAL_CAMERA_SDK_EXPORT const char* handEyeMethodName(HandEyeMethod m);
INDUSTRIAL_CAMERA_SDK_EXPORT HandEyeResult solveHandEyeEnsemble(const std::vector<HandEyeSample>& samples,
															   const HandEyeSolveParams& params);

/// 将额外候选（如 MechOfficial）按同一残差规则并入结果并可能刷新最优
INDUSTRIAL_CAMERA_SDK_EXPORT void mergeHandEyeCandidate(HandEyeResult& inout,
														const HandEyeMethodScore& candidate,
														const std::vector<HandEyeSample>& samples,
														const HandEyeSolveParams& params);

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_HANDEYETYPES_H
