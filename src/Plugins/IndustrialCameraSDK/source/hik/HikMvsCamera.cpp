/// @file HikMvsCamera.cpp
/// @brief 海康 MVS：定义 CLOUDSIM_HAS_HIK_MVS 且安装头库后启用真机路径

#include "HikMvsCamera.h"

#if defined(CLOUDSIM_HAS_HIK_MVS)
#include "MvCameraControl.h"
#endif

namespace industrial_camera
{

bool HikMvsCamera::sdkAvailable()
{
#if defined(CLOUDSIM_HAS_HIK_MVS)
	return true;
#else
	return false;
#endif
}

std::vector<CameraDeviceInfo> HikMvsCamera::enumerateDevices()
{
	std::vector<CameraDeviceInfo> out;
#if defined(CLOUDSIM_HAS_HIK_MVS)
	MV_CC_DEVICE_INFO_LIST list{};
	const int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
	if (nRet != MV_OK)
		return out;
	for (unsigned i = 0; i < list.nDeviceNum; ++i)
	{
		MV_CC_DEVICE_INFO* di = list.pDeviceInfo[i];
		if (!di)
			continue;
		CameraDeviceInfo info;
		info.brand = CameraBrand::Hikvision;
		info.capabilities = CapImage2D | CapIntrinsics;
		if (di->nTLayerType == MV_GIGE_DEVICE)
		{
			info.transport = CameraTransport::GigE;
			info.model = reinterpret_cast<char*>(di->SpecialInfo.stGigEInfo.chModelName);
			info.serial = reinterpret_cast<char*>(di->SpecialInfo.stGigEInfo.chSerialNumber);
			unsigned ip = di->SpecialInfo.stGigEInfo.nCurrentIp;
			char buf[32];
			std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff,
						  ip & 0xff);
			info.ip = buf;
		}
		else
		{
			info.transport = CameraTransport::USB;
			info.model = reinterpret_cast<char*>(di->SpecialInfo.stUsb3VInfo.chModelName);
			info.serial = reinterpret_cast<char*>(di->SpecialInfo.stUsb3VInfo.chSerialNumber);
		}
		out.push_back(info);
	}
#endif
	return out;
}

bool HikMvsCamera::connect(const CameraConnectParams& params)
{
	lastError_.clear();
	disconnect();
#if !defined(CLOUDSIM_HAS_HIK_MVS)
	lastError_ = "未编译/未安装海康 MVS SDK。请将 SDK 放到 bin/SDK/Hikrobot-MVS 并定义 CLOUDSIM_HAS_HIK_MVS。"
				 "调试可用「模拟」品牌。";
	(void)params;
	return false;
#else
	MV_CC_DEVICE_INFO_LIST list{};
	if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK)
	{
		lastError_ = "MV_CC_EnumDevices 失败";
		return false;
	}
	MV_CC_DEVICE_INFO* selected = nullptr;
	for (unsigned i = 0; i < list.nDeviceNum; ++i)
	{
		MV_CC_DEVICE_INFO* di = list.pDeviceInfo[i];
		if (!di)
			continue;
		std::string serial;
		std::string ip;
		if (di->nTLayerType == MV_GIGE_DEVICE)
		{
			serial = reinterpret_cast<char*>(di->SpecialInfo.stGigEInfo.chSerialNumber);
			unsigned u = di->SpecialInfo.stGigEInfo.nCurrentIp;
			char buf[32];
			std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (u >> 24) & 0xff, (u >> 16) & 0xff, (u >> 8) & 0xff, u & 0xff);
			ip = buf;
		}
		else
		{
			serial = reinterpret_cast<char*>(di->SpecialInfo.stUsb3VInfo.chSerialNumber);
		}
		if (!params.serial.empty() && serial == params.serial)
		{
			selected = di;
			break;
		}
		if (!params.ip.empty() && ip == params.ip)
		{
			selected = di;
			break;
		}
	}
	if (!selected && !params.ip.empty())
	{
		// 按 IP 创建句柄
		int nRet = MV_CC_CreateHandleByIP(&handle_, params.ip.c_str());
		if (nRet != MV_OK)
		{
			lastError_ = "按 IP 创建句柄失败";
			handle_ = nullptr;
			return false;
		}
	}
	else
	{
		if (!selected)
		{
			lastError_ = "未找到匹配设备（检查 IP/序列号）";
			return false;
		}
		if (MV_CC_CreateHandle(&handle_, selected) != MV_OK)
		{
			lastError_ = "CreateHandle 失败";
			handle_ = nullptr;
			return false;
		}
	}
	if (MV_CC_OpenDevice(handle_) != MV_OK)
	{
		lastError_ = "OpenDevice 失败";
		MV_CC_DestroyHandle(handle_);
		handle_ = nullptr;
		return false;
	}
	info_.brand = CameraBrand::Hikvision;
	info_.ip = params.ip;
	info_.serial = params.serial;
	info_.transport = params.ip.empty() ? CameraTransport::USB : CameraTransport::GigE;
	info_.capabilities = CapImage2D | CapIntrinsics;
	info_.model = "HikMVS";
	connected_ = true;
	return true;
#endif
}

void HikMvsCamera::disconnect()
{
	stopGrab();
#if defined(CLOUDSIM_HAS_HIK_MVS)
	if (handle_)
	{
		MV_CC_CloseDevice(handle_);
		MV_CC_DestroyHandle(handle_);
		handle_ = nullptr;
	}
#endif
	connected_ = false;
}

bool HikMvsCamera::startGrab()
{
#if !defined(CLOUDSIM_HAS_HIK_MVS)
	lastError_ = "无 MVS SDK";
	return false;
#else
	if (!connected_ || !handle_)
	{
		lastError_ = "未连接";
		return false;
	}
	MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
	if (MV_CC_StartGrabbing(handle_) != MV_OK)
	{
		lastError_ = "StartGrabbing 失败";
		return false;
	}
	grabbing_ = true;
	return true;
#endif
}

void HikMvsCamera::stopGrab()
{
#if defined(CLOUDSIM_HAS_HIK_MVS)
	if (handle_ && grabbing_)
		MV_CC_StopGrabbing(handle_);
#endif
	grabbing_ = false;
}

bool HikMvsCamera::grabOne(CameraFrame2D& out2d, CameraFrame3D* /*opt3d*/, int timeoutMs)
{
#if !defined(CLOUDSIM_HAS_HIK_MVS)
	(void)out2d;
	(void)timeoutMs;
	lastError_ = "无 MVS SDK";
	return false;
#else
	if (!grabbing_ && !startGrab())
		return false;
	MV_FRAME_OUT frame{};
	if (MV_CC_GetImageBuffer(handle_, &frame, timeoutMs) != MV_OK)
	{
		lastError_ = "取帧超时/失败";
		return false;
	}
	out2d.width = static_cast<int>(frame.stFrameInfo.nWidth);
	out2d.height = static_cast<int>(frame.stFrameInfo.nHeight);
	out2d.pixelFormat = PixelFormat::Mono8;
	out2d.bytes.assign(frame.pBufAddr, frame.pBufAddr + frame.stFrameInfo.nFrameLen);
	MV_CC_FreeImageBuffer(handle_, &frame);
	return true;
#endif
}

bool HikMvsCamera::getIntrinsics(CameraIntrinsics& out) const
{
	out = {};
	if (!connected_)
		return false;
	// MVS 无统一内参 API；留空由标定板流程估计或用户导入
	out.width = 0;
	out.height = 0;
	return true;
}

} // namespace industrial_camera
