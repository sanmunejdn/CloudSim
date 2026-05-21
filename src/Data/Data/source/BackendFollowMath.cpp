#include "BackendFollowMath.h"
#include "BackendDataBase.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstring>

namespace
{
Eigen::Isometry3d isometryFromBackendMat4(const BackendMat4& m)
{
	Eigen::Matrix4d e = Eigen::Matrix4d::Identity();
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			e(row, col) = m.v[col * 4 + row];
		}
	}
	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	iso.matrix() = e;
	return iso;
}

BackendMat4 backendMat4FromIsometry(const Eigen::Isometry3d& iso)
{
	BackendMat4 out{};
	const Eigen::Matrix4d e = iso.matrix();
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			out.v[col * 4 + row] = e(row, col);
		}
	}
	return out;
}
} // namespace

BackendMat4 BackendMat4::identity()
{
	BackendMat4 r{};
	r.v[0] = r.v[5] = r.v[10] = r.v[15] = 1.0;
	return r;
}

BackendMat4 BackendMat4::translate(double tx, double ty, double tz)
{
	BackendMat4 r = identity();
	r.v[12] = tx;
	r.v[13] = ty;
	r.v[14] = tz;
	return r;
}

BackendMat4 BackendMat4::rotateEulerDeg(double exDeg, double eyDeg, double ezDeg)
{
	const double ex = exDeg * (3.14159265358979323846 / 180.0);
	const double ey = eyDeg * (3.14159265358979323846 / 180.0);
	const double ez = ezDeg * (3.14159265358979323846 / 180.0);
	const double cx = std::cos(ex);
	const double sx = std::sin(ex);
	const double cy = std::cos(ey);
	const double sy = std::sin(ey);
	const double cz = std::cos(ez);
	const double sz = std::sin(ez);
	// R = Rz(ez)*Ry(ey)*Rx(ex) — matches BackendVisualMath / OSG branch.
	BackendMat4 r = identity();
	r.v[0] = cy * cz;
	r.v[1] = cy * sz;
	r.v[2] = -sy;
	r.v[4] = cz * sx * sy - cx * sz;
	r.v[5] = cx * cz + sx * sy * sz;
	r.v[6] = cy * sx;
	r.v[8] = sx * sz + cx * cz * sy;
	r.v[9] = cx * sy * sz - cz * sx;
	r.v[10] = cx * cy;
	r.v[3] = r.v[7] = r.v[11] = 0.0;
	r.v[12] = r.v[13] = r.v[14] = 0.0;
	r.v[15] = 1.0;
	return r;
}

bool backend_mat4_multiply(const BackendMat4& a, const BackendMat4& b, BackendMat4& out)
{
	out = backendMat4FromIsometry(isometryFromBackendMat4(a) * isometryFromBackendMat4(b));
	return true;
}

bool backend_mat4_invert_rigid(const BackendMat4& m, BackendMat4& out)
{
	// R stored column-major: col j at m[j*4+0..2]. Inverse = R^T and t' = -R^T * t.
	const double tx = m.v[12];
	const double ty = m.v[13];
	const double tz = m.v[14];
	out = BackendMat4::identity();
	out.v[0] = m.v[0];
	out.v[4] = m.v[1];
	out.v[8] = m.v[2];
	out.v[1] = m.v[4];
	out.v[5] = m.v[5];
	out.v[9] = m.v[6];
	out.v[2] = m.v[8];
	out.v[6] = m.v[9];
	out.v[10] = m.v[10];
	out.v[12] = -(m.v[0] * tx + m.v[1] * ty + m.v[2] * tz);
	out.v[13] = -(m.v[4] * tx + m.v[5] * ty + m.v[6] * tz);
	out.v[14] = -(m.v[8] * tx + m.v[9] * ty + m.v[10] * tz);
	out.v[15] = 1.0;
	return true;
}

BackendMat4 backend_world_mat_from_pose(
	const BackendVec3& modelCenter, const BackendVec3& pose, const BackendVec3& rotationEulerDeg)
{
	const BackendMat4 t = BackendMat4::translate(modelCenter.x + pose.x, modelCenter.y + pose.y, modelCenter.z + pose.z);
	const BackendMat4 r = BackendMat4::rotateEulerDeg(rotationEulerDeg.x, rotationEulerDeg.y, rotationEulerDeg.z);
	BackendMat4 out{};
	backend_mat4_multiply(t, r, out);
	return out;
}

namespace
{
constexpr double kPi = 3.14159265358979323846;
} // namespace

static void mat3_to_euler_deg(const BackendMat4& w, double& outX, double& outY, double& outZ)
{
	const double r00 = w.v[0];
	const double r10 = w.v[1];
	const double r20 = w.v[2];
	const double r01 = w.v[4];
	const double r11 = w.v[5];
	const double r21 = w.v[6];
	const double r02 = w.v[8];
	const double r12 = w.v[9];
	const double r22 = w.v[10];
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	if (r20 < -0.999999)
	{
		y = kPi * 0.5;
		x = std::atan2(r01, r02);
		z = 0.0;
	}
	else if (r20 > 0.999999)
	{
		y = -kPi * 0.5;
		x = std::atan2(-r01, -r02);
		z = 0.0;
	}
	else
	{
		y = std::asin(-r20);
		x = std::atan2(r21, r22);
		z = std::atan2(r10, r00);
	}
	outX = x * (180.0 / kPi);
	outY = y * (180.0 / kPi);
	outZ = z * (180.0 / kPi);
}

void backend_pose_euler_from_world_mat(
	const BackendMat4& world, const BackendVec3& modelCenter, BackendVec3& outPose, BackendVec3& outEulerDeg)
{
	const double tx = world.v[12];
	const double ty = world.v[13];
	const double tz = world.v[14];
	outPose.x = tx - modelCenter.x;
	outPose.y = ty - modelCenter.y;
	outPose.z = tz - modelCenter.z;
	mat3_to_euler_deg(world, outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
}

void backend_trans_euler_from_rigid_mat(const BackendMat4& m, BackendVec3& outTrans, BackendVec3& outEulerDeg)
{
	outTrans.x = m.v[12];
	outTrans.y = m.v[13];
	outTrans.z = m.v[14];
	mat3_to_euler_deg(m, outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
}
