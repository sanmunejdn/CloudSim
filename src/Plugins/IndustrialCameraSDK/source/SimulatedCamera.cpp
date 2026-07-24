/// @file SimulatedCamera.cpp
/// @brief 模拟相机：合成 2D/点云，IP 可手填

#include "SimulatedCamera.h"

#include <chrono>
#include <cmath>

namespace industrial_camera
{

std::vector<CameraDeviceInfo> SimulatedCamera::enumerateDevices()
{
	CameraDeviceInfo a;
	a.brand = CameraBrand::Simulated;
	a.model = "SimCam-2D3D";
	a.serial = "SIM-001";
	a.ip = "127.0.0.1";
	a.transport = CameraTransport::GigE;
	a.capabilities = CapImage2D | CapDepth | CapPointCloud | CapIntrinsics;

	CameraDeviceInfo b = a;
	b.serial = "SIM-002";
	b.ip = "127.0.0.2";
	b.model = "SimCam-2D";
	b.capabilities = CapImage2D | CapIntrinsics;
	return {a, b};
}

bool SimulatedCamera::connect(const CameraConnectParams& params)
{
	lastError_.clear();
	info_.brand = CameraBrand::Simulated;
	info_.model = "SimCam-2D3D";
	info_.serial = params.serial.empty() ? "SIM-001" : params.serial;
	info_.ip = params.ip.empty() ? "127.0.0.1" : params.ip;
	info_.transport = CameraTransport::GigE;
	with3d_ = (info_.serial != "SIM-002");
	info_.capabilities = CapImage2D | CapIntrinsics | (with3d_ ? (CapDepth | CapPointCloud) : 0u);
	connected_ = true;
	return true;
}

void SimulatedCamera::disconnect()
{
	stopGrab();
	connected_ = false;
}

bool SimulatedCamera::startGrab()
{
	if (!connected_)
	{
		lastError_ = "未连接";
		return false;
	}
	grabbing_ = true;
	return true;
}

void SimulatedCamera::stopGrab()
{
	grabbing_ = false;
}

bool SimulatedCamera::grabOne(CameraFrame2D& out2d, CameraFrame3D* opt3d, int /*timeoutMs*/)
{
	if (!connected_)
	{
		lastError_ = "未连接";
		return false;
	}
	if (!grabbing_ && !startGrab())
		return false;

	const int w = 640;
	const int h = 480;
	out2d.width = w;
	out2d.height = h;
	out2d.pixelFormat = PixelFormat::Bgr8;
	out2d.bytes.resize(static_cast<size_t>(w * h * 3));
	++frameIndex_;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const size_t i = static_cast<size_t>((y * w + x) * 3);
			out2d.bytes[i + 0] = static_cast<std::uint8_t>((x + frameIndex_) % 256);
			out2d.bytes[i + 1] = static_cast<std::uint8_t>((y + frameIndex_ * 3) % 256);
			out2d.bytes[i + 2] = static_cast<std::uint8_t>(128);
			// 棋盘格条纹，便于标定板检测占位预览
			if (((x / 40) + (y / 40)) % 2 == 0)
			{
				out2d.bytes[i + 0] = 240;
				out2d.bytes[i + 1] = 240;
				out2d.bytes[i + 2] = 240;
			}
		}
	}
	out2d.timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
							std::chrono::steady_clock::now().time_since_epoch())
							.count();

	if (opt3d && with3d_)
	{
		opt3d->width = w;
		opt3d->height = h;
		opt3d->depthMm.assign(static_cast<size_t>(w * h), 0.f);
		opt3d->points.clear();
		opt3d->points.reserve(static_cast<size_t>(w * h / 16));
		for (int y = 0; y < h; y += 4)
		{
			for (int x = 0; x < w; x += 4)
			{
				const float z = 800.f + 50.f * std::sin(0.02f * x + 0.01f * frameIndex_);
				opt3d->depthMm[static_cast<size_t>(y * w + x)] = z;
				CameraPoint3f pt;
				pt.x = (x - w * 0.5f) * z / 600.f;
				pt.y = (y - h * 0.5f) * z / 600.f;
				pt.z = z;
				pt.r = out2d.bytes[static_cast<size_t>((y * w + x) * 3 + 2)];
				pt.g = out2d.bytes[static_cast<size_t>((y * w + x) * 3 + 1)];
				pt.b = out2d.bytes[static_cast<size_t>((y * w + x) * 3 + 0)];
				opt3d->points.push_back(pt);
			}
		}
		opt3d->timestampNs = out2d.timestampNs;
	}
	return true;
}

bool SimulatedCamera::getIntrinsics(CameraIntrinsics& out) const
{
	out.fx = 600.0;
	out.fy = 600.0;
	out.cx = 320.0;
	out.cy = 240.0;
	out.dist = {{0, 0, 0, 0, 0}};
	out.width = 640;
	out.height = 480;
	return true;
}

} // namespace industrial_camera
