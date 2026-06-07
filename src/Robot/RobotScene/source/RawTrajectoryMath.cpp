#include "RawTrajectoryMath.h"

#include <cmath>

namespace RobotInstruction
{

constexpr double rawTrajectoryPi()
{
	return 3.14159265358979323846;
}

Vec3 normalizeVec(const Vec3& v)
{
	const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (len < 1e-12)
	{
		return Vec3{0.0, 0.0, 1.0};
	}
	return Vec3{v.x / len, v.y / len, v.z / len};
}

Vec3 crossVec(const Vec3& a, const Vec3& b)
{
	return Vec3{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
}

Vec3 eulerFromFrame(const Vec3& zAxis, const Vec3& xHint)
{
	const Vec3 z = normalizeVec(zAxis);
	Vec3 x = xHint;
	const double dot = x.x * z.x + x.y * z.y + x.z * z.z;
	x.x -= dot * z.x;
	x.y -= dot * z.y;
	x.z -= dot * z.z;
	x = normalizeVec(x);
	const Vec3 y = crossVec(z, x);
	(void)y;
	const double sy = std::sqrt(x.x * x.x + x.y * x.y);
	const bool singular = sy < 1e-6;
	double yaw = 0.0;
	double pitch = 0.0;
	double roll = 0.0;
	if (!singular)
	{
		yaw = std::atan2(x.y, x.x) * 180.0 / rawTrajectoryPi();
		pitch = std::atan2(-x.z, sy) * 180.0 / rawTrajectoryPi();
		roll = std::atan2(z.x * x.y - z.y * x.x, z.x * x.x + z.y * x.y) * 180.0 / rawTrajectoryPi();
	}
	else
	{
		yaw = std::atan2(-z.y, z.x) * 180.0 / rawTrajectoryPi();
		pitch = std::atan2(-x.z, sy) * 180.0 / rawTrajectoryPi();
	}
	return Vec3{roll, pitch, yaw};
}

void resampleTrajectory(RawTrajectory& traj, double stepMm)
{
	if (traj.points.size() < 2U || stepMm <= 0.0)
	{
		return;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < traj.points.size(); ++i)
	{
		const auto& a = traj.points[i - 1U].poseMm;
		const auto& b = traj.points[i].poseMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
		segLen.push_back(len);
		total += len;
	}
	if (total < stepMm)
	{
		return;
	}
	const int n = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	std::vector<TrajectoryPoint> out;
	out.reserve(static_cast<std::size_t>(n));
	for (int s = 0; s < n; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(n - 1) * total;
		double acc = 0.0;
		std::size_t seg = 0;
		while (seg < segLen.size() && acc + segLen[seg] < t - 1e-9)
		{
			acc += segLen[seg];
			++seg;
		}
		const double local = (seg < segLen.size() && segLen[seg] > 1e-12) ? (t - acc) / segLen[seg] : 0.0;
		const auto& p0 = traj.points[seg].poseMm;
		const auto& p1 = traj.points[std::min(seg + 1U, traj.points.size() - 1U)].poseMm;
		TrajectoryPoint tp = traj.points[seg];
		tp.poseMm.x = p0.x + (p1.x - p0.x) * local;
		tp.poseMm.y = p0.y + (p1.y - p0.y) * local;
		tp.poseMm.z = p0.z + (p1.z - p0.z) * local;
		out.push_back(tp);
	}
	traj.points = std::move(out);
}

} // namespace RobotInstruction
