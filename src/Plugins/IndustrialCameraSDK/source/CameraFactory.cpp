/// @file CameraFactory.cpp
/// @brief 按品牌创建/枚举相机

#include "ICamera.h"

#include "SimulatedCamera.h"
#include "hik/HikMv3dCamera.h"
#include "hik/HikMvsCamera.h"
#include "mech/MechEyeCamera.h"

namespace industrial_camera
{
namespace
{

class CameraFactory final : public ICameraFactory
{
public:
	std::vector<CameraDeviceInfo> enumerate(CameraBrand brand) override
	{
		lastError_.clear();
		switch (brand)
		{
		case CameraBrand::Hikvision:
		{
			auto a = HikMvsCamera::enumerateDevices();
			auto b = HikMv3dCamera::enumerateDevices();
			a.insert(a.end(), b.begin(), b.end());
			if (a.empty() && !HikMvsCamera::sdkAvailable() && !HikMv3dCamera::sdkAvailable())
				lastError_ = "海康 MVS/Mv3dRgbd SDK 未就绪，枚举为空。可改用模拟相机。";
			return a;
		}
		case CameraBrand::MechMind:
		{
			auto list = MechEyeCamera::enumerateDevices();
			if (list.empty() && !MechEyeCamera::sdkAvailable())
				lastError_ = "Mech-Eye SDK 未就绪（CLOUDSIM_HAS_MECH_EYE）。可改用模拟相机。";
			return list;
		}
		case CameraBrand::Simulated:
			return SimulatedCamera::enumerateDevices();
		default:
			lastError_ = "不支持的品牌";
			return {};
		}
	}

	std::unique_ptr<ICamera> create(CameraBrand brand) override
	{
		lastError_.clear();
		switch (brand)
		{
		case CameraBrand::Hikvision:
			return std::make_unique<HikMvsCamera>();
		case CameraBrand::MechMind:
			return std::make_unique<MechEyeCamera>();
		case CameraBrand::Simulated:
			return std::make_unique<SimulatedCamera>();
		default:
			lastError_ = "不支持的品牌";
			return nullptr;
		}
	}

	std::string lastError() const override { return lastError_; }

private:
	std::string lastError_;
};

} // namespace

std::unique_ptr<ICameraFactory> createCameraFactory()
{
	return std::make_unique<CameraFactory>();
}

bool hikMvsSdkAvailable()
{
	return HikMvsCamera::sdkAvailable();
}

bool hikMv3dSdkAvailable()
{
	return HikMv3dCamera::sdkAvailable();
}

bool mechEyeSdkAvailable()
{
	return MechEyeCamera::sdkAvailable();
}

std::unique_ptr<ICamera> createHikMv3dCamera()
{
	return std::make_unique<HikMv3dCamera>();
}

std::unique_ptr<ICamera> createHikMvsCamera()
{
	return std::make_unique<HikMvsCamera>();
}

std::unique_ptr<ICamera> createMechEyeCamera()
{
	return std::make_unique<MechEyeCamera>();
}

} // namespace industrial_camera
