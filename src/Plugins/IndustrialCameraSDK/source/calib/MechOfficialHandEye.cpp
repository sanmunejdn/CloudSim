/// @file MechOfficialHandEye.cpp
/// @brief 梅卡 HandEyeCalibration（对齐 SDK 2.5.4 嵌套类型）

#include "MechOfficialHandEye.h"

#include "mech/MechEyeCamera.h"

#include <Eigen/Geometry>

#if defined(CLOUDSIM_HAS_MECH_EYE)
#include <area_scan_3d_camera/Camera.h>
#include <area_scan_3d_camera/HandEyeCalibration.h>
using mmind::eye::Camera;
using mmind::eye::Color2DImage;
using mmind::eye::ErrorStatus;
using mmind::eye::HandEyeCalibration;
#endif

namespace industrial_camera
{

#if defined(CLOUDSIM_HAS_MECH_EYE)
namespace
{
struct MechOfficialImpl
{
	HandEyeCalibration calib;
	Camera* cam = nullptr;
};

HandEyeCalibration::Transformation pose6dToMech(const Pose6d& p)
{
	const Mat4 m = pose6dToMat4(p);
	Eigen::Matrix3d R;
	R << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];
	const Eigen::Quaterniond q(R);
	HandEyeCalibration::Transformation t;
	t.x = p.x;
	t.y = p.y;
	t.z = p.z;
	t.qW = q.w();
	t.qX = q.x();
	t.qY = q.y();
	t.qZ = q.z();
	return t;
}

Mat4 mechToMat4(const HandEyeCalibration::Transformation& t)
{
	const Eigen::Quaterniond q(t.qW, t.qX, t.qY, t.qZ);
	const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
	Mat4 m{};
	m[0] = R(0, 0);
	m[4] = R(0, 1);
	m[8] = R(0, 2);
	m[1] = R(1, 0);
	m[5] = R(1, 1);
	m[9] = R(1, 2);
	m[2] = R(2, 0);
	m[6] = R(2, 1);
	m[10] = R(2, 2);
	m[12] = t.x;
	m[13] = t.y;
	m[14] = t.z;
	m[15] = 1.0;
	return m;
}
} // namespace
#endif

MechOfficialHandEyeSession::MechOfficialHandEyeSession() = default;

MechOfficialHandEyeSession::~MechOfficialHandEyeSession()
{
	reset();
}

void MechOfficialHandEyeSession::reset()
{
#if defined(CLOUDSIM_HAS_MECH_EYE)
	delete static_cast<MechOfficialImpl*>(impl_);
#endif
	impl_ = nullptr;
	sampleCount_ = 0;
}

bool MechOfficialHandEyeSession::begin(ICamera* camera, HandEyeMountMode mode, std::string* err)
{
	reset();
	mode_ = mode;
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	(void)camera;
	if (err)
		*err = "无 Mech-Eye SDK，无法启动官方手眼会话";
	return false;
#else
	auto* mechCam = dynamic_cast<MechEyeCamera*>(camera);
	if (!mechCam || !mechCam->isConnected() || !mechCam->nativeHandle())
	{
		if (err)
			*err = "官方手眼需要已连接的 MechEyeCamera";
		return false;
	}
	auto* impl = new MechOfficialImpl();
	impl->cam = static_cast<Camera*>(mechCam->nativeHandle());
	const auto mount = (mode == HandEyeMountMode::EyeInHand)
						   ? HandEyeCalibration::CameraMountingMode::EyeInHand
						   : HandEyeCalibration::CameraMountingMode::EyeToHand;
	// 板型号可后续做成 UI 配置；默认 CGB_20
	const ErrorStatus st =
		impl->calib.initializeCalibration(*impl->cam, mount, HandEyeCalibration::CalibrationBoardModel::CGB_20);
	if (!st.isOK())
	{
		if (err)
			*err = st.errorDescription;
		delete impl;
		return false;
	}
	impl_ = impl;
	return true;
#endif
}

bool MechOfficialHandEyeSession::addPoseAndDetect(const Pose6d& robotPoseMmDeg, std::string* err)
{
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	(void)robotPoseMmDeg;
	if (err)
		*err = "无 Mech-Eye SDK";
	return false;
#else
	if (!impl_)
	{
		if (err)
			*err = "会话未 begin";
		return false;
	}
	auto* impl = static_cast<MechOfficialImpl*>(impl_);
	const auto pose = pose6dToMech(robotPoseMmDeg);
	Color2DImage color;
	const ErrorStatus st = impl->calib.addPoseAndDetect(*impl->cam, pose, color);
	if (!st.isOK())
	{
		if (err)
			*err = st.errorDescription;
		return false;
	}
	++sampleCount_;
	return true;
#endif
}

HandEyeMethodScore MechOfficialHandEyeSession::calculate(std::string* err)
{
	HandEyeMethodScore s;
	s.method = HandEyeMethod::MechOfficial;
	s.name = "MechOfficial";
#if !defined(CLOUDSIM_HAS_MECH_EYE)
	s.error = "无 Mech-Eye SDK";
	if (err)
		*err = s.error;
	return s;
#else
	if (!impl_ || sampleCount_ < 3)
	{
		s.error = "官方样本不足（建议≥6）";
		if (err)
			*err = s.error;
		return s;
	}
	auto* impl = static_cast<MechOfficialImpl*>(impl_);
	HandEyeCalibration::Transformation camToBase;
	const ErrorStatus st = impl->calib.calculateExtrinsics(*impl->cam, camToBase);
	if (!st.isOK())
	{
		s.error = st.errorDescription;
		if (err)
			*err = s.error;
		return s;
	}
	s.T = mechToMat4(camToBase);
	s.ok = true;
	s.score = 0.0;
	return s;
#endif
}

} // namespace industrial_camera
