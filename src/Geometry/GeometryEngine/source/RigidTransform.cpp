/// @file RigidTransform.cpp
/// @brief RigidTransform 实现

#include "RigidTransform.h"

#include "Adapters.h"

#include <cmath>

namespace engine
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond eulerDegToEigenQuat(double exDeg, double eyDeg, double ezDeg)
{
	const osg::Quat q = eulerDegToQuat(exDeg, eyDeg, ezDeg);
	return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

void eigenQuatToEulerDeg(const Eigen::Quaterniond& q, double& ex, double& ey, double& ez)
{
	const osg::Quat oq(q.x(), q.y(), q.z(), q.w());
	quatToEulerDeg(oq, ex, ey, ez);
}

} // namespace

RigidTransform::RigidTransform()
{
	m_iso.setIdentity();
}

RigidTransform RigidTransform::identity()
{
	return RigidTransform{};
}

RigidTransform RigidTransform::fromIsometry(const Eigen::Isometry3d& iso)
{
	RigidTransform out;
	out.m_iso = iso;
	return out;
}

RigidTransform RigidTransform::fromTranslationQuat(const Eigen::Vector3d& translationMm,
												   const Eigen::Quaterniond& rotation)
{
	RigidTransform out;
	out.m_iso = Eigen::Isometry3d::Identity();
	out.m_iso.linear() = rotation.normalized().toRotationMatrix();
	out.m_iso.translation() = translationMm;
	return out;
}

RigidTransform RigidTransform::fromTranslationEulerDeg(double pxMm, double pyMm, double pzMm, double exDeg,
													   double eyDeg, double ezDeg)
{
	return fromTranslationQuat(Eigen::Vector3d(pxMm, pyMm, pzMm), eulerDegToEigenQuat(exDeg, eyDeg, ezDeg));
}

Eigen::Vector3d RigidTransform::translationMm() const
{
	return m_iso.translation();
}

void RigidTransform::setTranslationMm(const Eigen::Vector3d& t)
{
	m_iso.translation() = t;
}

Eigen::Quaterniond RigidTransform::rotation() const
{
	return Eigen::Quaterniond(m_iso.linear());
}

void RigidTransform::setRotation(const Eigen::Quaterniond& q)
{
	m_iso.linear() = q.normalized().toRotationMatrix();
}

RigidTransform RigidTransform::composeScene(const RigidTransform& child) const
{
	const osg::Matrixd parentOsg = osgMatrixFromRigidTransform(*this);
	const osg::Matrixd childOsg = osgMatrixFromRigidTransform(child);
	return rigidTransformFromOsg(parentOsg * childOsg);
}

RigidTransform RigidTransform::composeColumn(const RigidTransform& right) const
{
	return fromIsometry(m_iso * right.isometry());
}

RigidTransform RigidTransform::inverse() const
{
	return fromIsometry(m_iso.inverse());
}

double RigidTransform::translationErrorMm(const RigidTransform& other) const
{
	return (translationMm() - other.translationMm()).norm();
}

double RigidTransform::rotationErrorDeg(const RigidTransform& other) const
{
	const Eigen::Quaterniond qa = rotation().normalized();
	const Eigen::Quaterniond qb = other.rotation().normalized();
	const double dot = std::abs(qa.dot(qb));
	const double clamped = std::min(1.0, std::max(-1.0, dot));
	return 2.0 * std::acos(clamped) * 180.0 / kPi;
}

void RigidTransform::translationMm(double& x, double& y, double& z) const
{
	const Eigen::Vector3d t = translationMm();
	x = t.x();
	y = t.y();
	z = t.z();
}

void RigidTransform::eulerDegForDisplay(double& ex, double& ey, double& ez) const
{
	eigenQuatToEulerDeg(rotation(), ex, ey, ez);
}

} // namespace engine
