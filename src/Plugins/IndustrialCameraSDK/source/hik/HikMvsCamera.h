#ifndef INDUSTRIALCAMERASDK_HIKMVSCAMERA_H
#define INDUSTRIALCAMERASDK_HIKMVSCAMERA_H

/// @file HikMvsCamera.h
/// @brief 海康 2D MVS 适配（无 SDK 时枚举空、连接失败并提示）

#include "ICamera.h"

namespace industrial_camera
{

class HikMvsCamera final : public ICamera
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
	void* handle_ = nullptr; // MV_CC handle，真 SDK 时使用
};

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_HIKMVSCAMERA_H
