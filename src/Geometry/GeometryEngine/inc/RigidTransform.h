#pragma once

#include "geometry_engine_global.h"

#include <Eigen/Geometry>

#include <array>

namespace engine
{

/// 基座/场景链刚体变换，平移 mm
class GEOMETRY_ENGINE_API RigidTransform
{
public:
	RigidTransform();

	static RigidTransform identity();
	static RigidTransform fromIsometry(const Eigen::Isometry3d& iso);
	static RigidTransform fromTranslationQuat(const Eigen::Vector3d& translationMm, const Eigen::Quaterniond& rotation);
	static RigidTransform fromTranslationEulerDeg(
		double pxMm,
		double pyMm,
		double pzMm,
		double exDeg,
		double eyDeg,
		double ezDeg);

	const Eigen::Isometry3d& isometry() const { return m_iso; }
	Eigen::Isometry3d& isometry() { return m_iso; }

	Eigen::Vector3d translationMm() const;
	void setTranslationMm(const Eigen::Vector3d& t);

	Eigen::Quaterniond rotation() const;
	void setRotation(const Eigen::Quaterniond& q);

	/// OSG/URDF 行向量链：this * child
	RigidTransform composeScene(const RigidTransform& child) const;

	/// 列向量/BackendMat4 链：out = this * right
	RigidTransform composeColumn(const RigidTransform& right) const;

	RigidTransform inverse() const;

	double translationErrorMm(const RigidTransform& other) const;
	double rotationErrorDeg(const RigidTransform& other) const;

	void translationMm(double& x, double& y, double& z) const;
	void eulerDegForDisplay(double& ex, double& ey, double& ez) const;

private:
	Eigen::Isometry3d m_iso;
};

} // namespace engine
