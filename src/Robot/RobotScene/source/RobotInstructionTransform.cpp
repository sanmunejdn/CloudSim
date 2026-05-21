#include "RobotInstructionTransform.h"

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
	out = Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
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
	cmd.setPose(Vec3{ px, py, pz });
	if (cmd.hasEulerProperty())
	{
		cmd.setEulerDeg(Vec3{ ex, ey, ez });
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
		outTargetInBase = engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d(p.x, p.y, p.z),
			Eigen::Quaterniond::Identity());
	}
	return true;
}

} // namespace RobotInstruction
