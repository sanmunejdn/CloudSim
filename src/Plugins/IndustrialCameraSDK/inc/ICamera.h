#ifndef INDUSTRIALCAMERASDK_ICAMERA_H
#define INDUSTRIALCAMERASDK_ICAMERA_H

/// @file ICamera.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 通用工业相机接口

#include "CameraTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace industrial_camera
{

class INDUSTRIAL_CAMERA_SDK_EXPORT ICamera
{
public:
	virtual ~ICamera() = default;

	virtual CameraBrand brand() const = 0;
	virtual bool connect(const CameraConnectParams& params) = 0;
	virtual void disconnect() = 0;
	virtual bool isConnected() const = 0;

	virtual bool startGrab() = 0;
	virtual void stopGrab() = 0;

	/// opt3d 可为 nullptr；2D-only 相机忽略
	virtual bool grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int timeoutMs) = 0;
	virtual bool getIntrinsics(CameraIntrinsics& out) const = 0;
	virtual CameraDeviceInfo deviceInfo() const = 0;
	virtual std::string lastError() const = 0;
};

class INDUSTRIAL_CAMERA_SDK_EXPORT ICameraFactory
{
public:
	virtual ~ICameraFactory() = default;
	virtual std::vector<CameraDeviceInfo> enumerate(CameraBrand brand) = 0;
	virtual std::unique_ptr<ICamera> create(CameraBrand brand) = 0;
	virtual std::string lastError() const = 0;
};

INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<ICameraFactory> createCameraFactory();
INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<ICamera> createHikMvsCamera();
INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<ICamera> createHikMv3dCamera();
INDUSTRIAL_CAMERA_SDK_EXPORT std::unique_ptr<ICamera> createMechEyeCamera();
INDUSTRIAL_CAMERA_SDK_EXPORT bool hikMvsSdkAvailable();
INDUSTRIAL_CAMERA_SDK_EXPORT bool hikMv3dSdkAvailable();
INDUSTRIAL_CAMERA_SDK_EXPORT bool mechEyeSdkAvailable();

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_ICAMERA_H
