#ifndef INDUSTRIALCAMERASDK_CAMERATYPES_H
#define INDUSTRIALCAMERASDK_CAMERATYPES_H

/// @file CameraTypes.h
/// @brief 工业相机公共类型（含 GigE IP）

#include "industrial_camera_sdk_global.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace industrial_camera
{

enum class CameraBrand
{
	Unknown = 0,
	Hikvision = 1,
	Simulated = 2,
	MechMind = 3 // 二期
};

enum class CameraTransport
{
	Unknown = 0,
	GigE = 1,
	USB = 2
};

enum CameraCapability : unsigned
{
	CapNone = 0,
	CapImage2D = 1u << 0,
	CapDepth = 1u << 1,
	CapPointCloud = 1u << 2,
	CapIntrinsics = 1u << 3
};

enum class PixelFormat
{
	Unknown = 0,
	Mono8,
	Bgr8,
	Rgb8
};

struct CameraDeviceInfo
{
	CameraBrand brand = CameraBrand::Unknown;
	std::string model;
	std::string serial;
	std::string ip; // GigE 必填展示；USB 可空
	CameraTransport transport = CameraTransport::Unknown;
	unsigned capabilities = CapNone;
};

struct CameraConnectParams
{
	CameraBrand brand = CameraBrand::Unknown;
	std::string serial;
	std::string ip;
	int timeoutMs = 5000;
};

struct CameraIntrinsics
{
	double fx = 0.0;
	double fy = 0.0;
	double cx = 0.0;
	double cy = 0.0;
	std::array<double, 5> dist{{0, 0, 0, 0, 0}};
	int width = 0;
	int height = 0;
};

struct CameraFrame2D
{
	int width = 0;
	int height = 0;
	PixelFormat pixelFormat = PixelFormat::Unknown;
	std::vector<std::uint8_t> bytes;
	std::int64_t timestampNs = 0;
};

struct CameraPoint3f
{
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
	std::uint8_t r = 128;
	std::uint8_t g = 128;
	std::uint8_t b = 128;
};

struct CameraFrame3D
{
	int width = 0;
	int height = 0;
	std::vector<float> depthMm; // 可选，与宽高对齐
	std::vector<CameraPoint3f> points;
	std::int64_t timestampNs = 0;
};

/// 列主序 4x4，平移 mm
using Mat4 = std::array<double, 16>;

struct Pose6d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double rxDeg = 0.0; // ZYX 内旋，度
	double ryDeg = 0.0;
	double rzDeg = 0.0;
};

INDUSTRIAL_CAMERA_SDK_EXPORT const char* brandToString(CameraBrand b);
INDUSTRIAL_CAMERA_SDK_EXPORT CameraBrand brandFromString(const std::string& s);
INDUSTRIAL_CAMERA_SDK_EXPORT Mat4 pose6dToMat4(const Pose6d& p);
INDUSTRIAL_CAMERA_SDK_EXPORT Pose6d mat4ToPose6d(const Mat4& m);
INDUSTRIAL_CAMERA_SDK_EXPORT bool writePlyAscii(const std::string& pathUtf8, const CameraFrame3D& frame, std::string* err);

} // namespace industrial_camera

#endif // INDUSTRIALCAMERASDK_CAMERATYPES_H
