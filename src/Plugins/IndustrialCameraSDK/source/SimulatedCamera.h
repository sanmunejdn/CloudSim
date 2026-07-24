#ifndef INDUSTRIALCAMERASDK_SIMULATEDCAMERA_H
#define INDUSTRIALCAMERASDK_SIMULATEDCAMERA_H

/// @file SimulatedCamera.h
/// @brief 无硬件时的模拟相机（调试 UI/落盘）

#include "ICamera.h"

namespace industrial_camera
{

class SimulatedCamera final : public ICamera
{
public:
	CameraBrand brand() const override { return CameraBrand::Simulated; }
	bool connect(const CameraConnectParams& params) override;
	void disconnect() override;
	bool isConnected() const override { return connected_; }
	bool startGrab() override;
	void stopGrab() override;
	bool grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int timeoutMs) override;
	bool getIntrinsics(CameraIntrinsics& out) const override;
	CameraDeviceInfo deviceInfo() const override { return info_; }
	std::string lastError() const override { return lastError_; }

	static std::vector<CameraDeviceInfo> enumerateDevices();

private:
	bool connected_ = false;
	bool grabbing_ = false;
	bool with3d_ = true;
	CameraDeviceInfo info_{};
	std::string lastError_;
	int frameIndex_ = 0;
};

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_SIMULATEDCAMERA_H
