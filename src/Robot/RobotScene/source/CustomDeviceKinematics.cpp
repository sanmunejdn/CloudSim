/// @file CustomDeviceKinematics.cpp
/// @brief CustomDeviceKinematics 实现

#include "CustomDeviceKinematics.h"

#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"

#include "CoreTypes.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace CustomDeviceKinematics
{
namespace
{
bool approxMat4Equal(const double a[16], const double b[16], const double eps = 1e-3)
{
	for (int i = 0; i < 16; ++i)
	{
		if (std::abs(a[i] - b[i]) > eps)
		{
			return false;
		}
	}
	return true;
}

void copyMat4(const double in[16], double out[16])
{
	std::memcpy(out, in, sizeof(double) * 16);
}

void backendMat4ToArray(const BackendMat4& m, double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = m.v[i];
	}
}

BackendMat4 arrayToBackendMat4(const double in[16])
{
	BackendMat4 m = BackendMat4::identity();
	for (int i = 0; i < 16; ++i)
	{
		m.v[i] = in[i];
	}
	return m;
}
} // namespace

RobotExternal::RobotExternalAxisConfig toExternalAxisConfig(const CustomDeviceAxisConfig& in)
{
	RobotExternal::RobotExternalAxisConfig out;
	out.enabled = in.enabled;
	out.displayName = in.displayName;
	out.jointName = in.jointName;
	out.motionType = in.motionType == CustomDeviceMotionType::Rotate ? RobotExternal::RobotExternalMotionType::Rotate
																	 : RobotExternal::RobotExternalMotionType::Translate;
	out.kind = out.motionType == RobotExternal::RobotExternalMotionType::Rotate
				   ? RobotExternal::RobotExternalAxisKind::Turntable
				   : RobotExternal::RobotExternalAxisKind::LinearRail;
	out.isPrismatic = out.motionType == RobotExternal::RobotExternalMotionType::Translate;
	out.attachment = RobotExternal::RobotExternalAttachment::RobotBase;
	out.lower = in.lower;
	out.upper = in.upper;
	out.home = in.home;
	out.axis[0] = in.axis[0];
	out.axis[1] = in.axis[1];
	out.axis[2] = in.axis[2];
	out.originMm[0] = in.originMm[0];
	out.originMm[1] = in.originMm[1];
	out.originMm[2] = in.originMm[2];
	return out;
}

RobotExternal::RobotExternalAxisConfigSet toExternalAxisConfigSet(const CustomDeviceAxisConfigSet& in)
{
	RobotExternal::RobotExternalAxisConfigSet out;
	out.axes.reserve(in.axes.size());
	for (const CustomDeviceAxisConfig& a : in.axes)
	{
		out.axes.push_back(toExternalAxisConfig(a));
	}
	return out;
}

void composeWorldFromBase(const double w0ColumnMajor[16], const CustomDeviceAxisConfigSet& axes,
						  const std::vector<double>& qValues, double outColumnMajor[16])
{
	copyMat4(w0ColumnMajor, outColumnMajor);
	for (size_t i = 0; i < axes.axes.size(); ++i)
	{
		const CustomDeviceAxisConfig& cfg = axes.axes[i];
		if (!cfg.enabled)
		{
			continue;
		}
		const double q = i < qValues.size() ? qValues[i] : cfg.home;
		const RobotExternal::RobotExternalAxisConfig ext = toExternalAxisConfig(cfg);
		double motion[16];
		RobotExternal::makeAxisMotionColumnMajor(ext, q, motion);
		double next[16];
		RobotExternal::mat4MulColumnMajor16(outColumnMajor, motion, next);
		copyMat4(next, outColumnMajor);
	}
}

void unbakeBaseFromWorld(const double wEffColumnMajor[16], const CustomDeviceAxisConfigSet& axes,
						 const std::vector<double>& qValues, double outW0ColumnMajor[16])
{
	copyMat4(wEffColumnMajor, outW0ColumnMajor);
	for (int i = static_cast<int>(axes.axes.size()) - 1; i >= 0; --i)
	{
		const CustomDeviceAxisConfig& cfg = axes.axes[static_cast<size_t>(i)];
		if (!cfg.enabled)
		{
			continue;
		}
		const double q = static_cast<size_t>(i) < qValues.size() ? qValues[static_cast<size_t>(i)] : cfg.home;
		const RobotExternal::RobotExternalAxisConfig ext = toExternalAxisConfig(cfg);
		double motion[16];
		RobotExternal::makeAxisMotionColumnMajor(ext, q, motion);
		double invMotion[16];
		if (!RobotExternal::mat4InvertRigidColumnMajor(motion, invMotion))
		{
			continue;
		}
		double next[16];
		RobotExternal::mat4MulColumnMajor16(outW0ColumnMajor, invMotion, next);
		copyMat4(next, outW0ColumnMajor);
	}
}

bool applyQ(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
			const std::vector<double>* qOverride)
{
	device.ensureQSize();
	std::vector<double> q = qOverride ? *qOverride : device.qValues();
	if (q.size() < device.axes().axes.size())
	{
		q.resize(device.axes().axes.size(), 0.0);
	}
	for (size_t i = 0; i < device.axes().axes.size() && i < q.size(); ++i)
	{
		q[i] = std::clamp(q[i], device.axes().axes[i].lower, device.axes().axes[i].upper);
	}

	double w0[16];
	backendMat4ToArray(device.baseWorldW0(), w0);
	double expected[16];
	composeWorldFromBase(w0, device.axes(), device.qValues(), expected);
	const BackendMat4 cur = device.worldMatrix(mgr);
	double curArr[16];
	backendMat4ToArray(cur, curArr);
	if (!approxMat4Equal(curArr, expected))
	{
		unbakeBaseFromWorld(curArr, device.axes(), device.qValues(), w0);
		device.setBaseWorldW0(arrayToBackendMat4(w0));
	}

	double wEff[16];
	composeWorldFromBase(w0, device.axes(), q, wEff);
	device.setQValues(q);
	device.setWorldMatrix(arrayToBackendMat4(wEff), mgr);

	if (sink)
	{
		cloudsim::core::Mat4 mat{};
		for (int i = 0; i < 16; ++i)
		{
			mat[static_cast<size_t>(i)] = wEff[i];
		}
		sink->setBackendRootWorldMatrixFromWorld(device.id(), mat);
	}
	return true;
}

bool worldPointToDeviceLocalMm(const BackendMat4& w0, const double worldX, const double worldY, const double worldZ,
							   double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	double w0a[16];
	backendMat4ToArray(w0, w0a);
	double inv[16];
	if (!RobotExternal::mat4InvertRigidColumnMajor(w0a, inv))
	{
		return false;
	}
	outLocal[0] = inv[0] * worldX + inv[4] * worldY + inv[8] * worldZ + inv[12];
	outLocal[1] = inv[1] * worldX + inv[5] * worldY + inv[9] * worldZ + inv[13];
	outLocal[2] = inv[2] * worldX + inv[6] * worldY + inv[10] * worldZ + inv[14];
	return true;
}

bool worldDirectionToDeviceLocal(const BackendMat4& w0, const double worldDx, const double worldDy, const double worldDz,
								 double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	double w0a[16];
	backendMat4ToArray(w0, w0a);
	double inv[16];
	if (!RobotExternal::mat4InvertRigidColumnMajor(w0a, inv))
	{
		return false;
	}
	outLocal[0] = inv[0] * worldDx + inv[4] * worldDy + inv[8] * worldDz;
	outLocal[1] = inv[1] * worldDx + inv[5] * worldDy + inv[9] * worldDz;
	outLocal[2] = inv[2] * worldDx + inv[6] * worldDy + inv[10] * worldDz;
	const double n = std::sqrt(outLocal[0] * outLocal[0] + outLocal[1] * outLocal[1] + outLocal[2] * outLocal[2]);
	if (n < 1e-12)
	{
		return false;
	}
	outLocal[0] /= n;
	outLocal[1] /= n;
	outLocal[2] /= n;
	return true;
}

} // namespace CustomDeviceKinematics
