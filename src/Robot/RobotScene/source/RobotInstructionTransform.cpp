/// @file RobotInstructionTransform.cpp
/// @brief 指令变换

#include "RobotInstructionTransform.h"

#include "RobotExternalAxes.h"

#include <cmath>
#include <sstream>

namespace RobotInstruction
{
namespace
{
std::string encodeQuatCsv(const Eigen::Quaterniond& q)
{
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	oss << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
	return oss.str();
}

bool parseQuatCsv(const std::string& csv, Eigen::Quaterniond& out)
{
	std::stringstream ss(csv);
	std::string token;
	double v[4]{};
	int idx = 0;
	while (std::getline(ss, token, ',') && idx < 4)
	{
		if (token.empty())
		{
			continue;
		}
		try
		{
			v[idx++] = std::stod(token);
		}
		catch (...)
		{
			return false;
		}
	}
	if (idx != 4)
	{
		return false;
	}
	for (int i = 0; i < 4; ++i)
	{
		if (!std::isfinite(v[i]))
		{
			return false;
		}
	}
	Eigen::Quaterniond q(v[3], v[0], v[1], v[2]);
	// 零范数 normalized() 会出 NaN 旋转；脏 CSV / 手改工程必须在读侧拒掉
	if (q.norm() < 1e-9)
	{
		return false;
	}
	out = q.normalized();
	return true;
}

std::string encodeTransCsv(const Eigen::Vector3d& t)
{
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	oss << t.x() << ',' << t.y() << ',' << t.z();
	return oss.str();
}

bool parseTransCsv(const std::string& csv, Eigen::Vector3d& out)
{
	std::stringstream ss(csv);
	std::string token;
	double v[3]{};
	int idx = 0;
	while (std::getline(ss, token, ',') && idx < 3)
	{
		if (token.empty())
		{
			continue;
		}
		try
		{
			v[idx++] = std::stod(token);
		}
		catch (...)
		{
			return false;
		}
	}
	if (idx != 3)
	{
		return false;
	}
	out = Eigen::Vector3d(v[0], v[1], v[2]);
	return true;
}

} // namespace

void writeTargetTransformToInstruction(Base& cmd, const engine::RigidTransform& targetInBase)
{
	if (!cmd.hasPoseProperty())
	{
		return;
	}
	const Eigen::Quaterniond q = targetInBase.rotation().normalized();
	const Eigen::Vector3d t = targetInBase.translationMm();
	cmd.setExtensionProperty(kExtContextTargetTransformQuatCsv, encodeQuatCsv(q));
	cmd.setExtensionProperty(kExtContextTargetTransformTransMmCsv, encodeTransCsv(t));
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	targetInBase.translationMm(px, py, pz);
	targetInBase.eulerDegForDisplay(ex, ey, ez);
	cmd.setPose(Vec3{px, py, pz});
	if (cmd.hasEulerProperty())
	{
		cmd.setEulerDeg(Vec3{ex, ey, ez});
	}
}

bool readTargetTransformFromInstruction(const Base& cmd, engine::RigidTransform& outTargetInBase)
{
	if (!cmd.hasPoseProperty())
	{
		return false;
	}
	const auto& ext = cmd.extensionProperties();
	const auto itQ = ext.find(kExtContextTargetTransformQuatCsv);
	const auto itT = ext.find(kExtContextTargetTransformTransMmCsv);
	if (itQ != ext.end() && itT != ext.end() && !itQ->second.empty() && !itT->second.empty())
	{
		Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
		Eigen::Vector3d t = Eigen::Vector3d::Zero();
		if (parseQuatCsv(itQ->second, q) && parseTransCsv(itT->second, t))
		{
			// 与示教拖动一致：界面 euler 为真源，context 四元数仅作落盘
			if (cmd.hasEulerProperty())
			{
				const Vec3 p = cmd.pose();
				const Vec3 e = cmd.eulerDeg();
				outTargetInBase =
					engine::RigidTransform::fromTranslationEulerDeg(p.x, p.y, p.z, e.x, e.y, e.z);
				return true;
			}
			outTargetInBase = engine::RigidTransform::fromTranslationQuat(t, q);
			return true;
		}
	}
	const Vec3 p = cmd.pose();
	if (cmd.hasEulerProperty())
	{
		const Vec3 e = cmd.eulerDeg();
		outTargetInBase = engine::RigidTransform::fromTranslationEulerDeg(p.x, p.y, p.z, e.x, e.y, e.z);
	}
	else
	{
		outTargetInBase =
			engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d(p.x, p.y, p.z), Eigen::Quaterniond::Identity());
	}
	return true;
}

bool applyTargetDisplayComponent(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg)
{
	if (!cmd.hasPoseProperty())
	{
		if (errMsg)
		{
			*errMsg = "instruction has no pose property";
		}
		return false;
	}
	double parsed = 0.0;
	try
	{
		size_t idx = 0;
		parsed = std::stod(value, &idx);
		if (idx != value.size())
		{
			if (errMsg)
			{
				*errMsg = "invalid number";
			}
			return false;
		}
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "invalid number";
		}
		return false;
	}
	engine::RigidTransform t{};
	if (!readTargetTransformFromInstruction(cmd, t))
	{
		if (errMsg)
		{
			*errMsg = "failed to read target transform";
		}
		return false;
	}
	Eigen::Vector3d trans = t.translationMm();
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	t.eulerDegForDisplay(ex, ey, ez);
	if (key == "motion.target.pose.x")
	{
		trans.x() = parsed;
	}
	else if (key == "motion.target.pose.y")
	{
		trans.y() = parsed;
	}
	else if (key == "motion.target.pose.z")
	{
		trans.z() = parsed;
	}
	else if (key == "motion.target.euler.rx")
	{
		ex = parsed;
	}
	else if (key == "motion.target.euler.ry")
	{
		ey = parsed;
	}
	else if (key == "motion.target.euler.rz")
	{
		ez = parsed;
	}
	else
	{
		if (errMsg)
		{
			*errMsg = "unknown target display key";
		}
		return false;
	}
	// 平移分量保留原旋转；欧拉分量用显示欧拉重建（与面板语义一致）
	if (key == "motion.target.pose.x" || key == "motion.target.pose.y" || key == "motion.target.pose.z")
	{
		writeTargetTransformToInstruction(
			cmd, engine::RigidTransform::fromTranslationQuat(trans, t.rotation().normalized()));
	}
	else
	{
		writeTargetTransformToInstruction(
			cmd, engine::RigidTransform::fromTranslationEulerDeg(trans.x(), trans.y(), trans.z(), ex, ey, ez));
	}
	return true;
}

void writeViaTransformToInstruction(Base& cmd, const engine::RigidTransform& viaInBase)
{
	if (!cmd.hasViaPoseProperty())
	{
		return;
	}
	const Eigen::Quaterniond q = viaInBase.rotation().normalized();
	const Eigen::Vector3d t = viaInBase.translationMm();
	cmd.setExtensionProperty(kExtContextViaTransformQuatCsv, encodeQuatCsv(q));
	cmd.setExtensionProperty(kExtContextViaTransformTransMmCsv, encodeTransCsv(t));
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	viaInBase.translationMm(px, py, pz);
	viaInBase.eulerDegForDisplay(ex, ey, ez);
	cmd.setViaPose(Vec3{px, py, pz});
	if (cmd.hasViaEulerProperty())
	{
		cmd.setViaEulerDeg(Vec3{ex, ey, ez});
	}
}

bool readViaTransformFromInstruction(const Base& cmd, engine::RigidTransform& outViaInBase)
{
	if (!cmd.hasViaPoseProperty())
	{
		return false;
	}
	const auto& ext = cmd.extensionProperties();
	const auto itQ = ext.find(kExtContextViaTransformQuatCsv);
	const auto itT = ext.find(kExtContextViaTransformTransMmCsv);
	if (itQ != ext.end() && itT != ext.end() && !itQ->second.empty() && !itT->second.empty())
	{
		Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
		Eigen::Vector3d t = Eigen::Vector3d::Zero();
		if (parseQuatCsv(itQ->second, q) && parseTransCsv(itT->second, t))
		{
			outViaInBase = engine::RigidTransform::fromTranslationQuat(t, q);
			return true;
		}
	}
	const Vec3 p = cmd.viaPose();
	if (cmd.hasViaEulerProperty())
	{
		const Vec3 e = cmd.viaEulerDeg();
		outViaInBase = engine::RigidTransform::fromTranslationEulerDeg(p.x, p.y, p.z, e.x, e.y, e.z);
	}
	else
	{
		outViaInBase =
			engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d(p.x, p.y, p.z), Eigen::Quaterniond::Identity());
	}
	return true;
}

bool applyViaDisplayComponent(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg)
{
	if (!cmd.hasViaPoseProperty())
	{
		if (errMsg)
		{
			*errMsg = "instruction has no via pose property";
		}
		return false;
	}
	double parsed = 0.0;
	try
	{
		size_t idx = 0;
		parsed = std::stod(value, &idx);
		if (idx != value.size())
		{
			if (errMsg)
			{
				*errMsg = "invalid number";
			}
			return false;
		}
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "invalid number";
		}
		return false;
	}
	engine::RigidTransform t{};
	if (!readViaTransformFromInstruction(cmd, t))
	{
		if (errMsg)
		{
			*errMsg = "failed to read via transform";
		}
		return false;
	}
	Eigen::Vector3d trans = t.translationMm();
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	t.eulerDegForDisplay(ex, ey, ez);
	if (key == "motion.via.pose.x")
	{
		trans.x() = parsed;
	}
	else if (key == "motion.via.pose.y")
	{
		trans.y() = parsed;
	}
	else if (key == "motion.via.pose.z")
	{
		trans.z() = parsed;
	}
	else if (key == "motion.via.euler.rx")
	{
		ex = parsed;
	}
	else if (key == "motion.via.euler.ry")
	{
		ey = parsed;
	}
	else if (key == "motion.via.euler.rz")
	{
		ez = parsed;
	}
	else
	{
		if (errMsg)
		{
			*errMsg = "unknown via display key";
		}
		return false;
	}
	if (key == "motion.via.pose.x" || key == "motion.via.pose.y" || key == "motion.via.pose.z")
	{
		writeViaTransformToInstruction(cmd,
									   engine::RigidTransform::fromTranslationQuat(trans, t.rotation().normalized()));
	}
	else
	{
		writeViaTransformToInstruction(
			cmd, engine::RigidTransform::fromTranslationEulerDeg(trans.x(), trans.y(), trans.z(), ex, ey, ez));
	}
	return true;
}

void writeWorkingTcpToInstruction(Base& cmd, const engine::RigidTransform& tcpInWorking)
{
	const Eigen::Quaterniond q = tcpInWorking.rotation().normalized();
	const Eigen::Vector3d t = tcpInWorking.translationMm();
	cmd.setExtensionProperty(RobotExternal::kExtContextWorkingTcpQuatCsv, encodeQuatCsv(q));
	cmd.setExtensionProperty(RobotExternal::kExtContextWorkingTcpTransMmCsv, encodeTransCsv(t));
}

bool readWorkingTcpFromInstruction(const Base& cmd, engine::RigidTransform& outTcpInWorking)
{
	const auto& ext = cmd.extensionProperties();
	const auto itQ = ext.find(RobotExternal::kExtContextWorkingTcpQuatCsv);
	const auto itT = ext.find(RobotExternal::kExtContextWorkingTcpTransMmCsv);
	if (itQ == ext.end() || itT == ext.end() || itQ->second.empty() || itT->second.empty())
	{
		return false;
	}
	Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
	Eigen::Vector3d t = Eigen::Vector3d::Zero();
	if (!parseQuatCsv(itQ->second, q) || !parseTransCsv(itT->second, t))
	{
		return false;
	}
	outTcpInWorking = engine::RigidTransform::fromTranslationQuat(t, q);
	return true;
}

} // namespace RobotInstruction
