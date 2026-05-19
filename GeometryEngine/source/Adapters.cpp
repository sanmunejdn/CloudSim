#include "Adapters.h"

#include <cmath>

namespace engine
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

void normalizeQuat(osg::Quat& q)
{
	const double n2 = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
	if (n2 <= 1e-24)
	{
		q.set(0.0, 0.0, 0.0, 1.0);
		return;
	}
	const double inv = 1.0 / std::sqrt(n2);
	q.set(q.x() * inv, q.y() * inv, q.z() * inv, q.w() * inv);
}

} // namespace

RigidTransform rigidTransformFromOsg(const osg::Matrixd& m)
{
	// OSG scene/URDF: row vectors v' = v * M. Eigen Isometry: p' = R*p + t (column).
	// Same rigid transform => R_eigen = R_osg^T, t_eigen = OSG bottom row (m(3,0..2)).
	Eigen::Matrix3d Rrow;
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			Rrow(r, c) = m(r, c);
		}
	}
	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	iso.linear() = Rrow.transpose();
	iso.translation() = Eigen::Vector3d(m(3, 0), m(3, 1), m(3, 2));
	return RigidTransform::fromIsometry(iso);
}

osg::Matrixd osgMatrixFromRigidTransform(const RigidTransform& t)
{
	const Eigen::Matrix3d Rcol = t.isometry().linear();
	const Eigen::Vector3d tr = t.isometry().translation();
	osg::Matrixd o;
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			o(r, c) = Rcol(c, r);
		}
	}
	o(3, 0) = tr.x();
	o(3, 1) = tr.y();
	o(3, 2) = tr.z();
	o(0, 3) = 0.0;
	o(1, 3) = 0.0;
	o(2, 3) = 0.0;
	o(3, 3) = 1.0;
	return o;
}

RigidTransform rigidTransformFromColMajor(const ColMajorMat4& m)
{
	// BackendMat4::v uses OpenGL column-major: index = col * 4 + row.
	osg::Matrixd o;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			o(r, c) = m[static_cast<size_t>(c * 4 + r)];
		}
	}
	return rigidTransformFromOsg(o);
}

ColMajorMat4 colMajorFromRigidTransform(const RigidTransform& t)
{
	const osg::Matrixd o = osgMatrixFromRigidTransform(t);
	ColMajorMat4 out{};
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			out[static_cast<size_t>(c * 4 + r)] = o(r, c);
		}
	}
	return out;
}

osg::Quat eulerDegToQuat(double exDeg, double eyDeg, double ezDeg)
{
	const double rx = exDeg * (kPi / 180.0);
	const double ry = eyDeg * (kPi / 180.0);
	const double rz = ezDeg * (kPi / 180.0);
	const double cx = std::cos(rx);
	const double sx = std::sin(rx);
	const double cy = std::cos(ry);
	const double sy = std::sin(ry);
	const double cz = std::cos(rz);
	const double sz = std::sin(rz);
	osg::Matrixd m;
	m(0, 0) = cy * cz;
	m(0, 1) = cz * sx * sy - cx * sz;
	m(0, 2) = sx * sz + cx * cz * sy;
	m(1, 0) = cy * sz;
	m(1, 1) = cx * cz + sx * sy * sz;
	m(1, 2) = cx * sy * sz - cz * sx;
	m(2, 0) = -sy;
	m(2, 1) = cy * sx;
	m(2, 2) = cx * cy;
	m(0, 3) = 0.0;
	m(1, 3) = 0.0;
	m(2, 3) = 0.0;
	m(3, 0) = 0.0;
	m(3, 1) = 0.0;
	m(3, 2) = 0.0;
	m(3, 3) = 1.0;
	osg::Quat q = m.getRotate();
	normalizeQuat(q);
	return q;
}

void quatToEulerDeg(const osg::Quat& qIn, double& exDeg, double& eyDeg, double& ezDeg)
{
	osg::Quat q = qIn;
	normalizeQuat(q);
	osg::Matrixd m;
	m.makeRotate(q);
	const double r00 = m(0, 0);
	const double r10 = m(1, 0);
	const double r20 = m(2, 0);
	const double r01 = m(0, 1);
	const double r11 = m(1, 1);
	const double r21 = m(2, 1);
	const double r02 = m(0, 2);
	const double r12 = m(1, 2);
	const double r22 = m(2, 2);
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
	exDeg = x * (180.0 / kPi);
	eyDeg = y * (180.0 / kPi);
	ezDeg = z * (180.0 / kPi);
}

osg::Vec3f quatToEulerDegVec3f(const osg::Quat& q)
{
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	quatToEulerDeg(q, ex, ey, ez);
	return osg::Vec3f(static_cast<float>(ex), static_cast<float>(ey), static_cast<float>(ez));
}

} // namespace engine
