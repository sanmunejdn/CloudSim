#ifndef INDUSTRIALCAMERASDK_HIKMV3DCAMERA_H
#define INDUSTRIALCAMERASDK_HIKMV3DCAMERA_H

/// @file HikMv3dCamera.h
/// @brief 海康 Mv3dRgbd 3D 适配

#include "ICamera.h"

namespace industrial_camera
{

class HikMv3dCamera final : public ICamera
{
public:
	CameraBrand brand() const override { return CameraBrand::Hikvision; }
	bool connect(const CameraConnectParams& params) override;
	void disconnect() override;
	bool isConnected() const override { return connected_; }
	bool startGrab() override;
	void stopGrab() override;
	bool grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int timeoutMs) override;
	bool getIntrinsics(CameraIntrinsics& out) const override;
	CameraDeviceInfo deviceInfo() const override { return info_; }
	std::string lastError() const override { return lastError_; }

	static bool sdkAvailable();
	static std::vector<CameraDeviceInfo> enumerateDevices();

private:
	bool connected_ = false;
	bool grabbing_ = false;
	CameraDeviceInfo info_{};
	std::string lastError_;
	void* handle_ = nullptr;
};

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_HIKMV3DCAMERA_H
