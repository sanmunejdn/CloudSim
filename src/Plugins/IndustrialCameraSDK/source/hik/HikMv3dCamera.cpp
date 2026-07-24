/// @file HikMv3dCamera.cpp
/// @brief 海康 Mv3dRgbd：定义 CLOUDSIM_HAS_HIK_MV3D 后启用

#include "HikMv3dCamera.h"

namespace industrial_camera
{

bool HikMv3dCamera::sdkAvailable()
{
#if defined(CLOUDSIM_HAS_HIK_MV3D)
	return true;
#else
	return false;
#endif
}

std::vector<CameraDeviceInfo> HikMv3dCamera::enumerateDevices()
{
	std::vector<CameraDeviceInfo> out;
#if defined(CLOUDSIM_HAS_HIK_MV3D)
	// 真 SDK：调用 Mv3dRgbd 枚举，填 ip/serial/CapDepth|CapPointCloud
#else
	(void)out;
#endif
	return out;
}

bool HikMv3dCamera::connect(const CameraConnectParams& params)
{
	lastError_.clear();
	disconnect();
#if !defined(CLOUDSIM_HAS_HIK_MV3D)
	(void)params;
	lastError_ = "未编译/未安装海康 Mv3dRgbd SDK。请放到 bin/SDK/Hikrobot-Mv3dRgbd 并定义 CLOUDSIM_HAS_HIK_MV3D。"
				 "调试可用「模拟」品牌（SIM-001 含点云）。";
	return false;
#else
	(void)params;
	lastError_ = "Mv3dRgbd 真机路径待链入厂商头文件";
	return false;
#endif
}

void HikMv3dCamera::disconnect()
{
	stopGrab();
	connected_ = false;
	handle_ = nullptr;
}

bool HikMv3dCamera::startGrab()
{
	if (!connected_)
	{
		lastError_ = "未连接";
		return false;
	}
	grabbing_ = true;
	return true;
}

void HikMv3dCamera::stopGrab()
{
	grabbing_ = false;
}

bool HikMv3dCamera::grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int /*timeoutMs*/)
{
	(void)out2d;
	(void)opt3d;
	lastError_ = "无 Mv3dRgbd SDK";
	return false;
}

bool HikMv3dCamera::getIntrinsics(CameraIntrinsics& out) const
{
	out = {};
	return connected_;
}

} // namespace industrial_camera
