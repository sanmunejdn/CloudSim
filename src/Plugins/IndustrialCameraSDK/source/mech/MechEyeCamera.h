#ifndef INDUSTRIALCAMERASDK_MECHEYECAMERA_H
#define INDUSTRIALCAMERASDK_MECHEYECAMERA_H

/// @file MechEyeCamera.h
/// @brief 梅卡 Mech-Eye 适配（宏 CLOUDSIM_HAS_MECH_EYE 启用真机）

#include "ICamera.h"

namespace industrial_camera
{

class MechEyeCamera final : public ICamera
{
public:
	CameraBrand brand() const override { return CameraBrand::MechMind; }
	bool connect(const CameraConnectParams& params) override;
	void disconnect() override;
	bool isConnected() const override { return connected_; }
	bool startGrab() override;
	void stopGrab() override;
	bool grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int timeoutMs) override;
	bool getIntrinsics(CameraIntrinsics& out) const override;
	CameraDeviceInfo deviceInfo() const override { return info_; }
	std::string lastError() const override { return lastError_; }

	/// 供官方手眼会话取 mmind::eye::Camera*
	void* nativeHandle() const { return camera_; }

	static bool sdkAvailable();
	static std::vector<CameraDeviceInfo> enumerateDevices();

private:
	bool connected_ = false;
	bool grabbing_ = false;
	CameraDeviceInfo info_{};
	std::string lastError_;
	void* camera_ = nullptr; // mmind::eye::Camera*
};

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_MECHEYECAMERA_H
