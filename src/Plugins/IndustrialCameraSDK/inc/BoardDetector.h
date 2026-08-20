#ifndef INDUSTRIALCAMERASDK_BOARDDETECTOR_H
#define INDUSTRIALCAMERASDK_BOARDDETECTOR_H

/// @file BoardDetector.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 标定板检测（OpenCV 棋盘格/ArUco；无宏时返回明确错误）

#include "CameraTypes.h"

#include <string>

namespace industrial_camera
{

enum class BoardType
{
	Chessboard = 0,
	Aruco = 1
};

struct BoardDetectParams
{
	BoardType type = BoardType::Chessboard;
	int cornersX = 9;  // 内角点列
	int cornersY = 6;  // 内角点行
	double squareSizeMm = 20.0;
	int arucoDictId = 0; // DICT_4X4_50
	float arucoMarkerLengthMm = 40.f;
};

struct BoardDetectResult
{
	bool ok = false;
	Mat4 T_cam_board{};
	std::string error;
};

INDUSTRIAL_CAMERA_SDK_EXPORT bool openCvAvailable();
INDUSTRIAL_CAMERA_SDK_EXPORT BoardDetectResult detectBoardPose(const CameraFrame2D& image,
															   const CameraIntrinsics& K,
															   const BoardDetectParams& params);

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_BOARDDETECTOR_H
