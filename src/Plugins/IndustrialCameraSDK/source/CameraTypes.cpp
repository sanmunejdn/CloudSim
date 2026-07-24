/// @file CameraTypes.cpp
/// @brief 位姿与 PLY 工具

#include "CameraTypes.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace industrial_camera
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double d)
{
	return d * kPi / 180.0;
}

double rad2deg(double r)
{
	return r * 180.0 / kPi;
}
} // namespace

const char* brandToString(CameraBrand b)
{
	switch (b)
	{
	case CameraBrand::Hikvision:
		return "Hikvision";
	case CameraBrand::Simulated:
		return "Simulated";
	case CameraBrand::MechMind:
		return "MechMind";
	default:
		return "Unknown";
	}
}

CameraBrand brandFromString(const std::string& s)
{
	if (s == "Hikvision" || s == "hik" || s == "海康")
		return CameraBrand::Hikvision;
	if (s == "Simulated" || s == "sim" || s == "模拟")
		return CameraBrand::Simulated;
	if (s == "MechMind" || s == "mech")
		return CameraBrand::MechMind;
	return CameraBrand::Unknown;
}

Mat4 pose6dToMat4(const Pose6d& p)
{
	const double rx = deg2rad(p.rxDeg);
	const double ry = deg2rad(p.ryDeg);
	const double rz = deg2rad(p.rzDeg);
	const double cx = std::cos(rx), sx = std::sin(rx);
	const double cy = std::cos(ry), sy = std::sin(ry);
	const double cz = std::cos(rz), sz = std::sin(rz);
	// R = Rz * Ry * Rx
	Mat4 m{};
	m[0] = cz * cy;
	m[4] = cz * sy * sx - sz * cx;
	m[8] = cz * sy * cx + sz * sx;
	m[12] = p.x;
	m[1] = sz * cy;
	m[5] = sz * sy * sx + cz * cx;
	m[9] = sz * sy * cx - cz * sx;
	m[13] = p.y;
	m[2] = -sy;
	m[6] = cy * sx;
	m[10] = cy * cx;
	m[14] = p.z;
	m[3] = 0.0;
	m[7] = 0.0;
	m[11] = 0.0;
	m[15] = 1.0;
	return m;
}

Pose6d mat4ToPose6d(const Mat4& m)
{
	Pose6d p;
	p.x = m[12];
	p.y = m[13];
	p.z = m[14];
	const double sy = -m[2];
	const double cy = std::sqrt(m[0] * m[0] + m[1] * m[1]);
	if (cy > 1e-8)
	{
		p.rxDeg = rad2deg(std::atan2(m[6], m[10]));
		p.ryDeg = rad2deg(std::atan2(sy, cy));
		p.rzDeg = rad2deg(std::atan2(m[1], m[0]));
	}
	else
	{
		p.rxDeg = rad2deg(std::atan2(-m[9], m[5]));
		p.ryDeg = rad2deg(std::atan2(sy, cy));
		p.rzDeg = 0.0;
	}
	return p;
}

bool writePlyAscii(const std::string& pathUtf8, const CameraFrame3D& frame, std::string* err)
{
	std::ofstream ofs(pathUtf8, std::ios::binary);
	if (!ofs)
	{
		if (err)
			*err = "无法写入 PLY: " + pathUtf8;
		return false;
	}
	ofs << "ply\nformat ascii 1.0\n";
	ofs << "element vertex " << frame.points.size() << "\n";
	ofs << "property float x\nproperty float y\nproperty float z\n";
	ofs << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
	ofs << "end_header\n";
	for (const auto& pt : frame.points)
	{
		ofs << pt.x << ' ' << pt.y << ' ' << pt.z << ' ' << int(pt.r) << ' ' << int(pt.g) << ' ' << int(pt.b) << '\n';
	}
	return true;
}

} // namespace industrial_camera
