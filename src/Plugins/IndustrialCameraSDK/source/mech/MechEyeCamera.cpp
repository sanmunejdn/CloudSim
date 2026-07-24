/// @file MechEyeCamera.cpp
/// @brief 梅卡 Mech-Eye：定义 CLOUDSIM_HAS_MECH_EYE 后链真 SDK

#include "MechEyeCamera.h"

#include <cmath>
#include <cstring>

#if defined(CLOUDSIM_HAS_MECH_EYE)
#include <area_scan_3d_camera/Camera.h>
#include <area_scan_3d_camera/Frame2DAnd3D.h>
#include <area_scan_3d_camera/api_util.h>
using mmind::eye::Camera;
using mmind::eye::CameraInfo;
using mmind::eye::CameraStatus;
using mmind::eye::ErrorStatus;
using mmind::eye::Frame2DAnd3D;
#endif

namespace industrial_camera
{

bool MechEyeCamera::sdkAvailable()
{
#if defined(CLOUDSIM_HAS_MECH_EYE)
	return true;
#else
	return false;
#endif
}

std::vector<CameraDeviceInfo> MechEyeCamera::enumerateDevices()
{
	std::vector<CameraDeviceInfo> out;
#if defined(CLOUDSIM_HAS_MECH_EYE)
	std::vector<CameraInfo> infos = Camera::discoverCameras();
	for (const auto& ci : infos)
	{
		CameraDeviceInfo d;
		d.brand = CameraBrand::MechMind;
		d.model = ci.model;
		d.serial = ci.serialNumber;
		d.ip = ci.ipAddress;
		d.transport = CameraTransport::GigE;
		d.capabilities = CapImage2D | CapDepth | CapPointCloud | CapIntrinsics;
		out.push_back(d);
	}
#endif
	return out;
}

bool MechEyeCamera::connect(const CameraConnectParams& params)
{
	lastError_.clear();
	disconnect();
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	(void)params;
	lastError_ = "未编译/未安装 Mech-Eye SDK。请放到 bin/SDK/MechEye 并定义 CLOUDSIM_HAS_MECH_EYE。";
	return false;
#else
	auto* cam = new Camera();
	ErrorStatus st;
	if (!params.ip.empty())
		st = cam->connect(params.ip);
	else if (!params.serial.empty())
	{
		std::vector<CameraInfo> infos = Camera::discoverCameras();
		bool found = false;
		for (const auto& ci : infos)
		{
			if (ci.serialNumber == params.serial)
			{
				st = cam->connect(ci);
				found = true;
				break;
			}
		}
		if (!found)
		{
			delete cam;
			lastError_ = "未找到序列号对应的 Mech-Eye";
			return false;
		}
	}
	else
	{
		delete cam;
		lastError_ = "请提供 IP 或序列号";
		return false;
	}
	if (!st.isOK())
	{
		lastError_ = st.errorDescription;
		delete cam;
		return false;
	}
	camera_ = cam;
	CameraInfo info;
	cam->getCameraInfo(info);
	info_.brand = CameraBrand::MechMind;
	info_.model = info.model;
	info_.serial = info.serialNumber;
	info_.ip = info.ipAddress.empty() ? params.ip : info.ipAddress;
	info_.transport = CameraTransport::GigE;
	info_.capabilities = CapImage2D | CapDepth | CapPointCloud | CapIntrinsics;
	connected_ = true;
	return true;
#endif
}

void MechEyeCamera::disconnect()
{
	stopGrab();
#if defined(CLOUDSIM_HAS_MECH_EYE)
	if (camera_)
	{
		static_cast<Camera*>(camera_)->disconnect();
		delete static_cast<Camera*>(camera_);
		camera_ = nullptr;
	}
#endif
	connected_ = false;
}

bool MechEyeCamera::startGrab()
{
	if (!connected_)
	{
		lastError_ = "未连接";
		return false;
	}
	grabbing_ = true;
	return true;
}

void MechEyeCamera::stopGrab()
{
	grabbing_ = false;
}

bool MechEyeCamera::grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int /*timeoutMs*/)
{
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	(void)out2d;
	(void)opt3d;
	lastError_ = "无 Mech-Eye SDK";
	return false;
#else
	if (!connected_ || !camera_)
	{
		lastError_ = "未连接";
		return false;
	}
	if (!grabbing_)
		startGrab();
	auto* cam = static_cast<Camera*>(camera_);
	Frame2DAnd3D frame;
	const ErrorStatus st = cam->capture2DAnd3D(frame);
	if (!st.isOK())
	{
		lastError_ = st.errorDescription;
		return false;
	}
	const auto color = frame.frame2D().getColorImage();
	out2d.width = static_cast<int>(color.width());
	out2d.height = static_cast<int>(color.height());
	out2d.pixelFormat = PixelFormat::Bgr8;
	out2d.bytes.resize(static_cast<size_t>(out2d.width * out2d.height * 3));
	const auto* src = color.data();
	if (src)
		std::memcpy(out2d.bytes.data(), src, out2d.bytes.size());

	if (opt3d)
	{
		const auto untextured = frame.frame3D().getUntexturedPointCloud();
		opt3d->width = static_cast<int>(untextured.width());
		opt3d->height = static_cast<int>(untextured.height());
		opt3d->points.clear();
		const size_t n = untextured.width() * untextured.height();
		opt3d->points.reserve(n);
		for (size_t i = 0; i < n; ++i)
		{
			const auto& p = untextured[i];
			if (!std::isfinite(p.z) || p.z <= 0.f)
				continue;
			CameraPoint3f pt;
			pt.x = p.x;
			pt.y = p.y;
			pt.z = p.z;
			opt3d->points.push_back(pt);
		}
	}
	return true;
#endif
}

bool MechEyeCamera::getIntrinsics(CameraIntrinsics& out) const
{
	out = {};
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	return false;
#else
	if (!connected_ || !camera_)
		return false;
	mmind::eye::CameraIntrinsics mechK;
	if (!static_cast<Camera*>(camera_)->getCameraIntrinsics(mechK).isOK())
		return false;
	out.fx = mechK.texture.cameraMatrix.fx;
	out.fy = mechK.texture.cameraMatrix.fy;
	out.cx = mechK.texture.cameraMatrix.cx;
	out.cy = mechK.texture.cameraMatrix.cy;
	return true;
#endif
}

} // namespace industrial_camera
