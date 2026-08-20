#ifndef INDUSTRIALCAMERAPLUGIN_CAMERARESOURCESTORE_H
#define INDUSTRIALCAMERAPLUGIN_CAMERARESOURCESTORE_H

/// @file CameraResourceStore.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief resource/industrial_camera 落盘

#include "CameraTypes.h"
#include "HandEyeTypes.h"

#include <QString>

namespace industrial_camera_ui
{

QString industrialCameraRoot();
bool ensureIndustrialCameraRoot(QString* err);

/// 返回会话目录绝对路径
QString saveCaptureSession(const industrial_camera::CameraDeviceInfo& info,
						   const industrial_camera::CameraFrame2D* color,
						   const industrial_camera::CameraFrame3D* cloud,
						   const industrial_camera::CameraIntrinsics* intrinsics,
						   QString* err);

QString saveCalibrationSession(industrial_camera::HandEyeMountMode mode,
							   const industrial_camera::HandEyeResult& result,
							   const industrial_camera::CameraIntrinsics* intrinsicsUsed,
							   QString* err);

bool saveDeviceIntrinsics(const industrial_camera::CameraDeviceInfo& info,
						  const industrial_camera::CameraIntrinsics& K,
						  QString* err);

} // namespace industrial_camera_ui

#endif // INDUSTRIALCAMERAPLUGIN_CAMERARESOURCESTORE_H
