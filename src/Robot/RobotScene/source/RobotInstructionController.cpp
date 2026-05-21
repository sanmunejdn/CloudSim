#include "RobotInstructionController.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotInstructionAxisConfiguration.h"
#include "BackendDataBase.h"
#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

#include <QHash>
#include <QString>
#include <QStringList>

#include <osg/Matrixd>
#include <osg/Quat>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
// #region agent log
void agentDebugLogScene(
	const char* hypothesisId,
	const char* location,
	const char* message,
	const std::string& dataJson,
	const char* runId = "pre-fix")
{
	(void)hypothesisId;
	(void)location;
	(void)message;
	(void)dataJson;
	(void)runId;
}
// #endregion

bool solveLinearSystem(std::vector<double>& a, std::vector<double>& b, int n)
{
	if (n <= 0 || static_cast<int>(a.size()) != n * n || static_cast<int>(b.size()) != n)
	{
		return false;
	}
	for (int col = 0; col < n; ++col)
	{
		int pivot = col;
		double maxAbs = std::abs(a[col * n + col]);
		for (int r = col + 1; r < n; ++r)
		{
			const double v = std::abs(a[r * n + col]);
			if (v > maxAbs)
			{
				maxAbs = v;
				pivot = r;
			}
		}
		if (maxAbs < 1e-12)
		{
			return false;
		}
		if (pivot != col)
		{
			for (int c = col; c < n; ++c)
			{
				std::swap(a[col * n + c], a[pivot * n + c]);
			}
			std::swap(b[col], b[pivot]);
		}
		const double diag = a[col * n + col];
		for (int c = col; c < n; ++c)
		{
			a[col * n + c] /= diag;
		}
		b[col] /= diag;
		for (int r = 0; r < n; ++r)
		{
			if (r == col)
			{
				continue;
			}
			const double f = a[r * n + col];
			if (std::abs(f) < 1e-15)
			{
				continue;
			}
			for (int c = col; c < n; ++c)
			{
				a[r * n + c] -= f * a[col * n + c];
			}
			b[r] -= f * b[col];
		}
	}
	return true;
}

void normalizeQuatSafe(osg::Quat& q)
{
	const double n2 = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
	if (n2 <= 1e-24)
	{
		q.set(0.0, 0.0, 0.0, 1.0);
		return;
	}
	const double invN = 1.0 / std::sqrt(n2);
	q.set(q.x() * invN, q.y() * invN, q.z() * invN, q.w() * invN);
}

osg::Quat eulerDegToQuat(const RobotInstruction::Vec3& eulerDeg);
osg::Matrixd backendMat4ToOsg(const BackendMat4& m);

struct IkLinkTarget
{
	double pos[3] = { 0.0, 0.0, 0.0 };
	osg::Quat quat;
	bool hasOrientation = false;
};

/// Sole tool handling before IK: pose/euler = T_base_target; solver input = T_base_flange only.
bool ikLinkTargetFromInstruction(const RobotInstruction::Base& cmd, IkLinkTarget& out)
{
	if (!cmd.hasPoseProperty())
	{
		return false;
	}
	engine::RigidTransform T_base_target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(cmd, T_base_target))
	{
		return false;
	}
	BackendMat4 T_flange_tool = BackendMat4::identity();
	const auto& ext = cmd.extensionProperties();
	const auto itTool = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	if (itTool != ext.end() && !itTool->second.empty())
	{
		(void)RobotCoordinate::parseMat4Csv(itTool->second, T_flange_tool);
	}
	double rawTx = 0.0;
	double rawTy = 0.0;
	double rawTz = 0.0;
	T_base_target.translationMm(rawTx, rawTy, rawTz);
	const engine::RigidTransform T_tool = RobotCoordinate::rigidTransformFromBackendMat4(T_flange_tool);
	double toolTx = 0.0;
	double toolTy = 0.0;
	double toolTz = 0.0;
	T_tool.translationMm(toolTx, toolTy, toolTz);
	const engine::RigidTransform T_base_link = engine::flangeFromToolOrigin(
		T_base_target,
		T_tool);
	double tx = 0.0;
	double ty = 0.0;
	double tz = 0.0;
	T_base_link.translationMm(tx, ty, tz);
	out.pos[0] = tx;
	out.pos[1] = ty;
	out.pos[2] = tz;
	if (cmd.hasEulerProperty())
	{
		const Eigen::Quaterniond q = T_base_link.rotation().normalized();
		out.quat.set(q.x(), q.y(), q.z(), q.w());
		normalizeQuatSafe(out.quat);
		out.hasOrientation = true;
	}
	// #region agent log
	{
		std::ostringstream d;
		d << "{\"rawTarget\":{\"x\":" << rawTx << ",\"y\":" << rawTy << ",\"z\":" << rawTz
		  << "},\"toolOffset\":{\"x\":" << toolTx << ",\"y\":" << toolTy << ",\"z\":" << toolTz
		  << "},\"computedFlange\":{\"x\":" << tx << ",\"y\":" << ty << ",\"z\":" << tz << "}}";
		agentDebugLogScene("H12", "ikLinkTargetFromInstruction", "tool_to_flange_conversion", d.str());
	}
	// #endregion
	return true;
}

osg::Matrixd backendMat4ToOsg(const BackendMat4& m)
{
	return RobotMatrixOsg::matrixFromBackendColMajor(m);
}

osg::Quat eulerDegToQuat(const RobotInstruction::Vec3& eulerDeg)
{
	const double rx = eulerDeg.x * kDegToRad;
	const double ry = eulerDeg.y * kDegToRad;
	const double rz = eulerDeg.z * kDegToRad;
	const double cx = std::cos(rx);
	const double sx = std::sin(rx);
	const double cy = std::cos(ry);
	const double sy = std::sin(ry);
	const double cz = std::cos(rz);
	const double sz = std::sin(rz);
	// Keep exactly the same convention as backendvisual_math::eulerDegToQuat:
	// R = Rz(rz) * Ry(ry) * Rx(rx), then extract quaternion from rotation matrix.
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
	normalizeQuatSafe(q);
	return q;
}

bool quatErrorAxisAngle(const osg::Quat& from, const osg::Quat& to, double outErrRad[3])
{
	if (!outErrRad)
	{
		return false;
	}
	osg::Quat qFrom = from;
	osg::Quat qTo = to;
	normalizeQuatSafe(qFrom);
	normalizeQuatSafe(qTo);
	osg::Quat qErr = qFrom.inverse() * qTo;
	if (qErr.w() < 0.0)
	{
		qErr.set(-qErr.x(), -qErr.y(), -qErr.z(), -qErr.w());
	}
	const double vx = qErr.x();
	const double vy = qErr.y();
	const double vz = qErr.z();
	const double vNorm = std::sqrt(vx * vx + vy * vy + vz * vz);
	if (vNorm < 1e-12)
	{
		outErrRad[0] = 0.0;
		outErrRad[1] = 0.0;
		outErrRad[2] = 0.0;
		return true;
	}
	const double angle = 2.0 * std::atan2(vNorm, std::max(1e-12, qErr.w()));
	const double inv = 1.0 / vNorm;
	outErrRad[0] = vx * inv * angle;
	outErrRad[1] = vy * inv * angle;
	outErrRad[2] = vz * inv * angle;
	return true;
}

double trapezoidDuration(double distance, double vmax, double amax)
{
	const double d = std::max(0.0, distance);
	const double v = std::max(1e-6, vmax);
	const double a = std::max(1e-6, amax);
	if (d < 1e-9)
	{
		return 0.05;
	}
	const double tAcc = v / a;
	const double dAcc = 0.5 * a * tAcc * tAcc;
	double t = 0.0;
	if (d <= 2.0 * dAcc)
	{
		t = 2.0 * std::sqrt(d / a);
	}
	else
	{
		t = 2.0 * tAcc + (d - 2.0 * dAcc) / v;
	}
	return std::max(0.05, t);
}

std::vector<double> parseCsvDoubles(const std::string& text)
{
	std::vector<double> out;
	std::stringstream ss(text);
	std::string token;
	while (std::getline(ss, token, ','))
	{
		if (token.empty())
		{
			continue;
		}
		try
		{
			out.push_back(std::stod(token));
		}
		catch (...)
		{
			return {};
		}
	}
	return out;
}

std::vector<double> currentJointVectorFromInstruction(const RobotInstruction::Base& cmd)
{
	const auto& ext = cmd.extensionProperties();
	const auto it = ext.find("context.currentJointRadCsv");
	if (it == ext.end())
	{
		return {};
	}
	return parseCsvDoubles(it->second);
}

bool hasTcpLinkContext(const RobotInstruction::Base& cmd)
{
	const auto& ext = cmd.extensionProperties();
	const auto it = ext.find("context.tcpLinkName");
	return it != ext.end() && !it->second.empty();
}

bool legacyJointDeltaFromInstruction(const RobotInstruction::Base& cmd, int& outJointIndex, double& outDeltaRad)
{
	const auto& ext = cmd.extensionProperties();
	const auto itIdx = ext.find("legacy.jointIndex");
	const auto itDeg = ext.find("legacy.angleDeg");
	if (itIdx == ext.end() || itDeg == ext.end())
	{
		return false;
	}
	try
	{
		outJointIndex = std::stoi(itIdx->second);
		outDeltaRad = std::stod(itDeg->second) * kDegToRad;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

std::vector<double> solveTargetByLegacyJointDelta(const RobotInstruction::Base& cmd)
{
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		return {};
	}
	int jidx = -1;
	double dq = 0.0;
	if (!legacyJointDeltaFromInstruction(cmd, jidx, dq))
	{
		return {};
	}
	if (jidx < 0 || jidx >= static_cast<int>(q.size()))
	{
		return {};
	}
	q[static_cast<size_t>(jidx)] += dq;
	return q;
}

std::vector<double> solveTargetByIkIfPossible(
	const RobotInstruction::Base& cmd,
	const std::vector<robot_kinematics::DhRow>& dhRows,
	std::string* failReason)
{
	if (dhRows.empty() || !cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const std::size_t nJoint = robot_kinematics::jointCountFromDhRows(dhRows);
	if (nJoint == 0 || q.size() != nJoint)
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double targetPos[3] = { linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2] };

	double reachMm = 0.0;
	for (const robot_kinematics::DhRow& row : dhRows)
	{
		reachMm += std::sqrt(row.a * row.a + row.d * row.d);
	}
	reachMm = std::max(reachMm, 1.0);
	const double targetNormMm = std::sqrt(targetPos[0] * targetPos[0] + targetPos[1] * targetPos[1] + targetPos[2] * targetPos[2]);
	if (targetNormMm > reachMm * 2.5 || targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	double curPos[3] = { 0.0, 0.0, 0.0 };
	if (robot_kinematics::endEffectorPosition(dhRows, q, curPos))
	{
		const double dx = targetPos[0] - curPos[0];
		const double dy = targetPos[1] - curPos[1];
		const double dz = targetPos[2] - curPos[2];
		const double moveDistanceMm = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (moveDistanceMm > reachMm * 2.0)
		{
			if (failReason)
			{
				*failReason = "目标越界/单位不一致";
			}
			return {};
		}
	}

	std::vector<double> qSolved = q;
	int iters = 0;
	if (!robot_kinematics::ikPositionDampedLeastSquares(dhRows, targetPos, qSolved, 120, 1e-3, 1e-2, &iters))
	{
		if (failReason)
		{
			*failReason = "IK未收敛/超迭代";
		}
		return {};
	}
	(void)iters;
	return qSolved;
}

bool tcpPositionFromUrdf(
	const QString& urdfPath,
	const QString& tcpLink,
	const std::vector<double>& q,
	double outPos[3],
	osg::Quat* outRot = nullptr)
{
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(q.size()));
	for (double v : q)
	{
		qQt.push_back(v);
	}
	QHash<QString, osg::Matrixd> linkWorldByName;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, qQt, linkWorldByName, nullptr))
	{
		return false;
	}
	if (!linkWorldByName.contains(tcpLink))
	{
		return false;
	}
	const osg::Vec3d t = linkWorldByName.value(tcpLink).getTrans();
	outPos[0] = t.x();
	outPos[1] = t.y();
	outPos[2] = t.z();
	if (outRot)
	{
		*outRot = linkWorldByName.value(tcpLink).getRotate();
		normalizeQuatSafe(*outRot);
	}
	return true;
}

bool currentTcpPositionFromInstruction(const RobotInstruction::Base& cmd, double outPos[3])
{
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	const auto itTcp = ext.find("context.tcpLinkName");
	if (itUrdf == ext.end() || itTcp == ext.end() || itUrdf->second.empty() || itTcp->second.empty())
	{
		return false;
	}
	const std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		return false;
	}
	return tcpPositionFromUrdf(QString::fromStdString(itUrdf->second), QString::fromStdString(itTcp->second), q, outPos);
}

void logIkSolveResidual(
	const RobotInstruction::Base& cmd,
	const std::vector<double>& qSolved,
	const char* plannerName)
{
	if (!RunLogger::isDiagnosticsEnabled())
	{
		return;
	}
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		return;
	}
	if (qSolved.empty() || !cmd.hasPoseProperty())
	{
		return;
	}
	std::string flangeLink;
	const auto itFlange = ext.find("context.flangeLinkName");
	if (itFlange != ext.end() && !itFlange->second.empty())
	{
		flangeLink = itFlange->second;
	}
	else
	{
		const auto itTcp = ext.find("context.tcpLinkName");
		if (itTcp == ext.end() || itTcp->second.empty())
		{
			return;
		}
		flangeLink = itTcp->second;
	}

	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const QString flangeLinkQ = QString::fromStdString(flangeLink);

	engine::RigidTransform T_base_target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(cmd, T_base_target))
	{
		return;
	}
	double toolOriginPos[3]{};
	T_base_target.translationMm(toolOriginPos[0], toolOriginPos[1], toolOriginPos[2]);

	BackendMat4 T_flange_tool = BackendMat4::identity();
	const auto itTool = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	if (itTool != ext.end() && !itTool->second.empty())
	{
		(void)RobotCoordinate::parseMat4Csv(itTool->second, T_flange_tool);
	}

	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		return;
	}
	const RobotInstruction::Vec3 flangeTarget{
		linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2] };
	const bool hasTargetEuler = linkTarget.hasOrientation;

	double fkFlangePos[3] = { 0.0, 0.0, 0.0 };
	osg::Quat fkFlangeRot;
	if (!tcpPositionFromUrdf(urdfPath, flangeLinkQ, qSolved, fkFlangePos, &fkFlangeRot))
	{
		std::ostringstream os;
		os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown")
		   << ", flangeLink=" << flangeLink << ", FK failed (cannot recompute flange pose from q)";
		RunLogger::warn(os.str());
		return;
	}

	double fkToolOriginPos[3]{};
	engine::RigidTransform fkToolRt{};
	bool hasFkToolRot = false;
	{
		QVector<double> qQt;
		qQt.reserve(static_cast<int>(qSolved.size()));
		for (double v : qSolved)
		{
			qQt.push_back(v);
		}
		QHash<QString, osg::Matrixd> linkWorld;
		if (UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, qQt, linkWorld, nullptr)
			&& linkWorld.contains(flangeLinkQ))
		{
			const BackendMat4 fkTcpMat = RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(
				linkWorld.value(flangeLinkQ), T_flange_tool);
			fkToolRt = RobotCoordinate::rigidTransformFromBackendMat4(fkTcpMat);
			fkToolRt.translationMm(fkToolOriginPos[0], fkToolOriginPos[1], fkToolOriginPos[2]);
			hasFkToolRot = true;
		}
	}

	std::ostringstream qPreview;
	const int previewN = std::min(6, static_cast<int>(qSolved.size()));
	for (int i = 0; i < previewN; ++i)
	{
		if (i > 0)
		{
			qPreview << ", ";
		}
		qPreview << qSolved[static_cast<size_t>(i)];
	}
	if (static_cast<int>(qSolved.size()) > previewN)
	{
		qPreview << ", ...";
	}

	const double dFlangeX = fkFlangePos[0] - flangeTarget.x;
	const double dFlangeY = fkFlangePos[1] - flangeTarget.y;
	const double dFlangeZ = fkFlangePos[2] - flangeTarget.z;
	const double residualFlangeMm =
		std::sqrt(dFlangeX * dFlangeX + dFlangeY * dFlangeY + dFlangeZ * dFlangeZ);

	const double dTcpX = fkToolOriginPos[0] - toolOriginPos[0];
	const double dTcpY = fkToolOriginPos[1] - toolOriginPos[1];
	const double dTcpZ = fkToolOriginPos[2] - toolOriginPos[2];
	const double residualTcpMm = std::sqrt(dTcpX * dTcpX + dTcpY * dTcpY + dTcpZ * dTcpZ);

	double residualTcpRotDeg = 0.0;
	double residualFlangeRotDeg = 0.0;
	if (hasTargetEuler)
	{
		const osg::Quat targetFlangeQuat = linkTarget.quat;
		double eRot[3] = { 0.0, 0.0, 0.0 };
		quatErrorAxisAngle(fkFlangeRot, targetFlangeQuat, eRot);
		const double errRotRad = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		residualFlangeRotDeg = errRotRad * (180.0 / 3.14159265358979323846);

		if (hasFkToolRot)
		{
			residualTcpRotDeg = T_base_target.rotationErrorDeg(fkToolRt);
		}
	}

	std::ostringstream os;
	os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown")
	   << ", flangeLink=" << flangeLink
	   << ", toolOrigin=(" << toolOriginPos[0] << ", " << toolOriginPos[1] << ", " << toolOriginPos[2] << ")"
	   << ", flangeTarget=(" << flangeTarget.x << ", " << flangeTarget.y << ", " << flangeTarget.z << ")"
	   << ", fkFlange=(" << fkFlangePos[0] << ", " << fkFlangePos[1] << ", " << fkFlangePos[2] << ")"
	   << ", fkToolOrigin=(" << fkToolOriginPos[0] << ", " << fkToolOriginPos[1] << ", " << fkToolOriginPos[2]
	   << ")"
	   << ", solvedQ[0..]=[" << qPreview.str() << "]"
	   << ", residualTcpMm=" << residualTcpMm
	   << ", residualFlangeMm=" << residualFlangeMm;
	if (hasTargetEuler)
	{
		os << ", residualTcpRotDeg=" << residualTcpRotDeg
		   << ", residualFlangeRotDeg=" << residualFlangeRotDeg;
	}
	if (residualTcpMm > 1.0 && residualFlangeMm < 0.1)
	{
		os << " (hint: flange OK but tool matrix/link mismatch)";
	}
	RunLogger::info(os.str());
}

int jointIndexByNameHint(const std::vector<std::string>& jointNames, const char* hint, int fallbackIndex)
{
	if (hint)
	{
		for (size_t i = 0; i < jointNames.size(); ++i)
		{
			std::string lower = jointNames[i];
			std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
				return static_cast<char>(std::tolower(c));
			});
			if (lower.find(hint) != std::string::npos)
			{
				return static_cast<int>(i);
			}
		}
	}
	if (fallbackIndex >= 0 && fallbackIndex < static_cast<int>(jointNames.size()))
	{
		return fallbackIndex;
	}
	return -1;
}

double jointVectorDistance(const std::vector<double>& a, const std::vector<double>& b)
{
	if (a.size() != b.size())
	{
		return 1e30;
	}
	double sum = 0.0;
	for (size_t i = 0; i < a.size(); ++i)
	{
		const double d = a[i] - b[i];
		sum += d * d;
	}
	return std::sqrt(sum);
}

std::vector<std::string> revoluteJointNamesFromInstructionContext(const RobotInstruction::Base& cmd)
{
	std::vector<std::string> jointNames;
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		return jointNames;
	}
	QStringList ql;
	(void)UrdfRobotLoader::loadRevoluteJointNamesInOrder(QString::fromStdString(itUrdf->second), ql, nullptr);
	jointNames.reserve(static_cast<size_t>(ql.size()));
	for (const QString& n : ql)
	{
		jointNames.push_back(n.toStdString());
	}
	return jointNames;
}

std::vector<std::vector<double>> buildIkSeedVariants(
	const std::vector<double>& qSeed,
	const std::vector<std::string>& jointNames,
	const RobotInstruction::MotionAxisConfiguration* axisCfg)
{
	std::vector<std::vector<double>> seeds;
	if (qSeed.empty())
	{
		return seeds;
	}
	auto pushUnique = [&](std::vector<double> q) {
		for (const auto& existing : seeds)
		{
			if (jointVectorDistance(existing, q) < 1e-6)
			{
				return;
			}
		}
		seeds.push_back(std::move(q));
	};
	pushUnique(qSeed);
	const int elbowIdx = jointIndexByNameHint(jointNames, "elbow", 2);
	const int wristIdx = jointIndexByNameHint(jointNames, "wrist", 4);
	const int j1Idx = jointIndexByNameHint(jointNames, nullptr, 0);
	if (elbowIdx >= 0)
	{
		std::vector<double> qElbow = qSeed;
		qElbow[static_cast<size_t>(elbowIdx)] = -qElbow[static_cast<size_t>(elbowIdx)];
		pushUnique(std::move(qElbow));
	}
	if (wristIdx >= 0)
	{
		std::vector<double> qWrist = qSeed;
		qWrist[static_cast<size_t>(wristIdx)] += 3.14159265358979323846;
		pushUnique(std::move(qWrist));
		if (elbowIdx >= 0)
		{
			std::vector<double> qBoth = qSeed;
			qBoth[static_cast<size_t>(elbowIdx)] = -qBoth[static_cast<size_t>(elbowIdx)];
			qBoth[static_cast<size_t>(wristIdx)] += 3.14159265358979323846;
			pushUnique(std::move(qBoth));
		}
	}
	if (axisCfg && RobotInstruction::motionAxisConfigurationRequiresConstraint(*axisCfg))
	{
		RobotInstruction::JointConfigurationClass want{};
		axisCfg->resolveEffective(want);
		if (elbowIdx >= 0)
		{
			if (want.elbow == RobotInstruction::ElbowPosture::Up)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = 1.2;
				pushUnique(std::move(q));
				q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = 2.0;
				pushUnique(std::move(q));
			}
			else if (want.elbow == RobotInstruction::ElbowPosture::Down)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = -1.2;
				pushUnique(std::move(q));
				q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = -2.0;
				pushUnique(std::move(q));
			}
		}
		if (wristIdx >= 0)
		{
			if (want.wrist == RobotInstruction::WristPosture::Flip)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(wristIdx)] += 3.14159265358979323846;
				pushUnique(std::move(q));
			}
			else if (want.wrist == RobotInstruction::WristPosture::NoFlip)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(wristIdx)] *= 0.25;
				pushUnique(std::move(q));
			}
		}
		if (j1Idx >= 0)
		{
			if (want.arm == RobotInstruction::ArmPosture::Front)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(j1Idx)] = 0.0;
				pushUnique(std::move(q));
			}
			else if (want.arm == RobotInstruction::ArmPosture::Back)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(j1Idx)] = 3.14159265358979323846;
				pushUnique(std::move(q));
			}
		}
	}
	return seeds;
}

std::vector<double> solveTargetByUrdfNumericalIkFromSeed(
	const RobotInstruction::Base& cmd,
	std::vector<double> q,
	std::string* failReason)
{
	if (!cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const auto itTcp = ext.find("context.tcpLinkName");
	const auto itFlange = ext.find("context.flangeLinkName");
	const std::string ikLinkName = (itFlange != ext.end() && !itFlange->second.empty())
		? itFlange->second
		: ((itTcp != ext.end()) ? itTcp->second : std::string());
	if (ikLinkName.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const QString ikLink = QString::fromStdString(ikLinkName);
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double target[3] = { linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2] };
	const bool useOrientation = linkTarget.hasOrientation;
	const osg::Quat targetQuat = useOrientation ? linkTarget.quat : osg::Quat();
	engine::RigidTransform targetToolRt{};
	const bool hasTargetToolRt = RobotInstruction::readTargetTransformFromInstruction(cmd, targetToolRt);
	BackendMat4 T_flange_tool = BackendMat4::identity();
	if (const auto itToolMat = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
		itToolMat != ext.end() && !itToolMat->second.empty())
	{
		(void)RobotCoordinate::parseMat4Csv(itToolMat->second, T_flange_tool);
	}
	const engine::RigidTransform flangeToolRt = RobotCoordinate::rigidTransformFromBackendMat4(T_flange_tool);
	// #region agent log
	{
		const auto itMotionTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
		const auto itCtxTool = ext.find("context.activeToolFrameId");
		const std::string motionToolId = (itMotionTool != ext.end()) ? itMotionTool->second : std::string();
		const std::string ctxToolId = (itCtxTool != ext.end()) ? itCtxTool->second : std::string();
		std::ostringstream d;
		d << "{\"ikLink\":\"" << ikLinkName << "\",\"motionToolId\":\"" << motionToolId
		  << "\",\"ctxToolId\":\"" << ctxToolId << "\",\"target\":{\"x\":" << target[0]
		  << ",\"y\":" << target[1] << ",\"z\":" << target[2] << "}}";
		agentDebugLogScene("H10", "solveTargetByUrdfNumericalIkFromSeed", "ik_input_link_target", d.str());
	}
	// #endregion
	const double targetNormMm = std::sqrt(target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
	if (targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	double pos[3] = { 0.0, 0.0, 0.0 };
	osg::Quat curQuat;
	if (!tcpPositionFromUrdf(urdfPath, ikLink, q, pos, useOrientation ? &curQuat : nullptr))
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double initialErr = std::sqrt(
		(target[0] - pos[0]) * (target[0] - pos[0]) +
		(target[1] - pos[1]) * (target[1] - pos[1]) +
		(target[2] - pos[2]) * (target[2] - pos[2]));
	if (initialErr > 10000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	const double eps = 1e-4;
	const double lambda = 1e-2;
	const int maxIters = 180;
	const int taskDim = useOrientation ? 6 : 3;
	// Unit balancing for 6D IK: translation is in mm, rotation is in rad.
	// Without scaling, orientation residual is numerically too small.
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	for (int iter = 0; iter < maxIters; ++iter)
	{
		if (!tcpPositionFromUrdf(urdfPath, ikLink, q, pos, useOrientation ? &curQuat : nullptr))
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		std::vector<double> e(static_cast<size_t>(taskDim), 0.0);
		e[0] = target[0] - pos[0];
		e[1] = target[1] - pos[1];
		e[2] = target[2] - pos[2];
		double posErr = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
		double rotErr = 0.0;
		if (useOrientation)
		{
			double eRot[3] = { 0.0, 0.0, 0.0 };
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			e[3] = eRot[0] * orientationWeight;
			e[4] = eRot[1] * orientationWeight;
			e[5] = eRot[2] * orientationWeight;
			rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}
		if (posErr < 1e-2 && (!useOrientation || rotErr < 0.1 * kDegToRad))
		{
			// #region agent log
			{
				double toolPosErr = -1.0;
				if (useOrientation && hasTargetToolRt)
				{
					const Eigen::Quaterniond qFlange(
						curQuat.w(), curQuat.x(), curQuat.y(), curQuat.z());
					const engine::RigidTransform fkFlangeRt =
						engine::RigidTransform::fromTranslationQuat(
							Eigen::Vector3d(pos[0], pos[1], pos[2]), qFlange);
					const engine::RigidTransform fkToolRt =
						engine::toolOriginFromFlange(fkFlangeRt, flangeToolRt);
					toolPosErr = (fkToolRt.translationMm() - targetToolRt.translationMm()).norm();
				}
				std::ostringstream d;
				d << "{\"ikLink\":\"" << ikLinkName << "\",\"finalPosErrMm\":" << posErr
				  << ",\"finalRotErrRad\":" << rotErr << ",\"toolPosErrMm\":" << toolPosErr
				  << ",\"iter\":" << iter << "}";
				agentDebugLogScene("H10", "solveTargetByUrdfNumericalIkFromSeed", "ik_converged", d.str());
			}
			// #endregion
			return q;
		}

		const int n = static_cast<int>(q.size());
		std::vector<double> J(static_cast<size_t>(taskDim * n), 0.0);
		for (int j = 0; j < n; ++j)
		{
			std::vector<double> qPert = q;
			qPert[static_cast<size_t>(j)] += eps;
			double p2[3] = { 0.0, 0.0, 0.0 };
			osg::Quat q2;
			if (!tcpPositionFromUrdf(urdfPath, ikLink, qPert, p2, useOrientation ? &q2 : nullptr))
			{
				if (failReason)
				{
					*failReason = "无DH上下文";
				}
				return {};
			}
			J[0 * n + j] = (p2[0] - pos[0]) / eps;
			J[1 * n + j] = (p2[1] - pos[1]) / eps;
			J[2 * n + j] = (p2[2] - pos[2]) / eps;
			if (useOrientation)
			{
				double dRot[3] = { 0.0, 0.0, 0.0 };
				quatErrorAxisAngle(curQuat, q2, dRot);
				J[3 * n + j] = (dRot[0] / eps) * orientationWeight;
				J[4 * n + j] = (dRot[1] / eps) * orientationWeight;
				J[5 * n + j] = (dRot[2] / eps) * orientationWeight;
			}
		}

		std::vector<double> jtj(static_cast<size_t>(n * n), 0.0);
		std::vector<double> jte(static_cast<size_t>(n), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				jte[static_cast<size_t>(c)] += J[static_cast<size_t>(r * n + c)] * e[static_cast<size_t>(r)];
			}
		}
		for (int r = 0; r < n; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < taskDim; ++k)
				{
					s += J[static_cast<size_t>(k * n + r)] * J[static_cast<size_t>(k * n + c)];
				}
				jtj[static_cast<size_t>(r * n + c)] = s;
			}
		}
		for (int i = 0; i < n; ++i)
		{
			jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
		}
		if (!solveLinearSystem(jtj, jte, n))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			return {};
		}
		for (int j = 0; j < n; ++j)
		{
			jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
		}
	}

	if (useOrientation)
	{
		IkLinkTarget posOnlyTarget = linkTarget;
		posOnlyTarget.hasOrientation = false;
		std::vector<double> qPos = q;
		const double posTarget[3] = { posOnlyTarget.pos[0], posOnlyTarget.pos[1], posOnlyTarget.pos[2] };
		for (int iter = 0; iter < maxIters; ++iter)
		{
			if (!tcpPositionFromUrdf(urdfPath, ikLink, qPos, pos, nullptr))
			{
				break;
			}
			const double e0 = posTarget[0] - pos[0];
			const double e1 = posTarget[1] - pos[1];
			const double e2 = posTarget[2] - pos[2];
			const double posErr = std::sqrt(e0 * e0 + e1 * e1 + e2 * e2);
			if (posErr < 1e-2)
			{
				return qPos;
			}
			const int n = static_cast<int>(qPos.size());
			std::vector<double> J(static_cast<size_t>(3 * n), 0.0);
			for (int j = 0; j < n; ++j)
			{
				std::vector<double> qPert = qPos;
				qPert[static_cast<size_t>(j)] += eps;
				double p2[3] = { 0.0, 0.0, 0.0 };
				if (!tcpPositionFromUrdf(urdfPath, ikLink, qPert, p2, nullptr))
				{
					break;
				}
				J[0 * n + j] = (p2[0] - pos[0]) / eps;
				J[1 * n + j] = (p2[1] - pos[1]) / eps;
				J[2 * n + j] = (p2[2] - pos[2]) / eps;
			}
			std::vector<double> jtj(static_cast<size_t>(n * n), 0.0);
			std::vector<double> jte(static_cast<size_t>(n), 0.0);
			const double e[3] = { e0, e1, e2 };
			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < n; ++c)
				{
					jte[static_cast<size_t>(c)] += J[static_cast<size_t>(r * n + c)] * e[r];
				}
			}
			for (int r = 0; r < n; ++r)
			{
				for (int c = 0; c < n; ++c)
				{
					double s = 0.0;
					for (int k = 0; k < 3; ++k)
					{
						s += J[static_cast<size_t>(k * n + r)] * J[static_cast<size_t>(k * n + c)];
					}
					jtj[static_cast<size_t>(r * n + c)] = s;
				}
			}
			for (int i = 0; i < n; ++i)
			{
				jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
			}
			if (!solveLinearSystem(jtj, jte, n))
			{
				break;
			}
			for (int j = 0; j < n; ++j)
			{
				jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
				qPos[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
			}
		}
	}

	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

std::vector<double> solveTargetByUrdfNumericalIkIfPossible(const RobotInstruction::Base& cmd, std::string* failReason)
{
	std::vector<double> q0 = currentJointVectorFromInstruction(cmd);
	if (q0.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	return solveTargetByUrdfNumericalIkFromSeed(cmd, std::move(q0), failReason);
}

std::vector<double> solveIkWithAxisConfiguration(
	const RobotInstruction::Base& cmd,
	const RobotInstruction::MotionAxisConfiguration& cfg,
	std::string* failReason)
{
	std::vector<double> qSeed = currentJointVectorFromInstruction(cmd);
	if (qSeed.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const std::vector<std::string> jointNames = revoluteJointNamesFromInstructionContext(cmd);
	const std::vector<std::vector<double>> seeds = buildIkSeedVariants(qSeed, jointNames, &cfg);

	struct Candidate
	{
		std::vector<double> q;
		double dist = 0.0;
	};
	std::vector<Candidate> matching;
	std::vector<Candidate> converged;
	for (const std::vector<double>& seed : seeds)
	{
		std::vector<double> qTry = solveTargetByUrdfNumericalIkFromSeed(cmd, seed, nullptr);
		if (qTry.empty())
		{
			continue;
		}
		const double dist = jointVectorDistance(qTry, qSeed);
		converged.push_back({ std::move(qTry), dist });
	}
	for (Candidate& c : converged)
	{
		const RobotInstruction::JointConfigurationClass observed =
			RobotInstruction::classifyJointConfiguration(c.q, jointNames, &qSeed);
		if (cfg.matchesClass(observed))
		{
			matching.push_back(std::move(c));
		}
	}
	const std::vector<Candidate>& pool = cfg.isFullyAuto() ? converged : matching;
	if (pool.empty())
	{
		if (failReason)
		{
			*failReason = cfg.isFullyAuto() ? "IK未收敛/超迭代" : "无满足轴配置的IK解";
		}
		return {};
	}
	const auto isCandidateBetter = [](const Candidate& c, const Candidate& best) {
		if (c.dist < best.dist - 1e-9)
		{
			return true;
		}
		if (c.dist > best.dist + 1e-9)
		{
			return false;
		}
		const size_t n = std::min(c.q.size(), best.q.size());
		for (size_t i = 0; i < n; ++i)
		{
			if (c.q[i] < best.q[i] - 1e-12)
			{
				return true;
			}
			if (c.q[i] > best.q[i] + 1e-12)
			{
				return false;
			}
		}
		return c.q.size() < best.q.size();
	};
	const Candidate* best = &pool.front();
	for (const Candidate& c : pool)
	{
		if (isCandidateBetter(c, *best))
		{
			best = &c;
		}
	}
	return best->q;
}

std::vector<double> solveIkWithAxisConfiguration(const RobotInstruction::Base& cmd, std::string* failReason)
{
	RobotInstruction::MotionAxisConfiguration cfg;
	if (cmd.hasMotionAxisConfigurationProperty())
	{
		cfg = cmd.motionAxisConfiguration();
	}
	return solveIkWithAxisConfiguration(cmd, cfg, failReason);
}

bool canSolveIkWithAxisConfiguration(
	const RobotInstruction::Base& cmd,
	const RobotInstruction::MotionAxisConfiguration& cfg)
{
	return !solveIkWithAxisConfiguration(cmd, cfg, nullptr).empty();
}

struct IkPostureClassEntry
{
	RobotInstruction::JointConfigurationClass cls;
};

bool postureClassEquivalent(
	const RobotInstruction::JointConfigurationClass& a,
	const RobotInstruction::JointConfigurationClass& b)
{
	return a.elbow == b.elbow && a.wrist == b.wrist && a.arm == b.arm && a.turnJ1 == b.turnJ1
		&& a.turnJ4 == b.turnJ4 && a.turnJ6 == b.turnJ6;
}

void appendUniquePostureClass(
	std::vector<IkPostureClassEntry>& out,
	const RobotInstruction::JointConfigurationClass& cls)
{
	for (const IkPostureClassEntry& e : out)
	{
		if (postureClassEquivalent(e.cls, cls))
		{
			return;
		}
	}
	out.push_back({ cls });
}

std::vector<IkPostureClassEntry> collectIkPostureClassesForTarget(const RobotInstruction::Base& cmd)
{
	std::vector<IkPostureClassEntry> out;
	std::vector<double> qSeed = currentJointVectorFromInstruction(cmd);
	if (qSeed.empty() || !hasTcpLinkContext(cmd))
	{
		return out;
	}
	const std::vector<std::string> jointNames = revoluteJointNamesFromInstructionContext(cmd);
	const std::vector<std::vector<double>> seeds = buildIkSeedVariants(qSeed, jointNames, nullptr);
	for (const std::vector<double>& seed : seeds)
	{
		std::vector<double> q = solveTargetByUrdfNumericalIkFromSeed(cmd, seed, nullptr);
		if (q.empty())
		{
			continue;
		}
		const RobotInstruction::JointConfigurationClass cls =
			RobotInstruction::classifyJointConfiguration(q, jointNames, &qSeed);
		appendUniquePostureClass(out, cls);
	}
	return out;
}

bool anyPostureMatchesConfiguration(
	const std::vector<IkPostureClassEntry>& postures,
	const RobotInstruction::MotionAxisConfiguration& cfg)
{
	for (const IkPostureClassEntry& e : postures)
	{
		if (cfg.matchesClass(e.cls))
		{
			return true;
		}
	}
	return false;
}

class PtpPlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit PtpPlanner(const std::vector<robot_kinematics::DhRow>* dhRows)
		: m_dhRows(dhRows)
	{
	}

	bool canHandle(RobotInstruction::Type type) const override { return type == RobotInstruction::Type::PTP; }

	bool validate(const RobotInstruction::Base& cmd, std::string* errMsg) const override
	{
		if (!cmd.hasSpeedProperty() || cmd.speed() <= 0.0)
		{
			if (errMsg)
				*errMsg = "PTP instruction requires speed > 0.";
			return false;
		}
		if (!cmd.hasAccelProperty() || cmd.accel() <= 0.0)
		{
			if (errMsg)
				*errMsg = "PTP instruction requires acceleration > 0.";
			return false;
		}
		return true;
	}

	bool plan(const RobotInstruction::Base& cmd, RobotInstruction::PlanResult& out, std::string* errMsg) const override
	{
		if (!validate(cmd, errMsg))
		{
			return false;
		}
		const std::vector<double> q0 = currentJointVectorFromInstruction(cmd);
		if (q0.empty())
		{
			if (errMsg)
			{
				*errMsg = "无DH上下文";
			}
			return false;
		}
		std::vector<double> targetQ;
		std::string ikFailReason;
		std::string solvePath = "none";
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		RobotInstruction::MotionAxisConfiguration axisCfg;
		if (cmd.hasMotionAxisConfigurationProperty())
		{
			axisCfg = cmd.motionAxisConfiguration();
		}
		const bool constrainAxis = cmd.hasMotionAxisConfigurationProperty()
			&& RobotInstruction::motionAxisConfigurationRequiresConstraint(axisCfg);
		if (preferUrdfIk)
		{
			if (cmd.hasMotionAxisConfigurationProperty())
			{
				targetQ = solveIkWithAxisConfiguration(cmd, &ikFailReason);
				if (!targetQ.empty())
				{
					solvePath = "urdf_axis_cfg";
				}
			}
			else
			{
				targetQ = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
				if (!targetQ.empty())
				{
					solvePath = "urdf";
				}
			}
			if (targetQ.empty() && !constrainAxis)
			{
				targetQ = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
				if (!targetQ.empty())
				{
					solvePath = "urdf_retry";
				}
			}
		}
		if (targetQ.empty() && m_dhRows && !m_dhRows->empty() && !constrainAxis)
		{
			targetQ = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
			if (!targetQ.empty())
			{
				solvePath = "dh";
			}
		}
		if (targetQ.empty() && !preferUrdfIk && !constrainAxis)
		{
			targetQ = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			if (!targetQ.empty())
			{
				solvePath = "urdf_late";
			}
		}
		if (targetQ.empty() && !constrainAxis)
		{
			targetQ = solveTargetByLegacyJointDelta(cmd);
			if (!targetQ.empty())
			{
				solvePath = "legacy";
			}
		}
		// #region agent log
		{
			const auto& ext = cmd.extensionProperties();
			const auto itTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
			const auto itCtx = ext.find("context.activeToolFrameId");
			const std::string motionToolId = (itTool != ext.end()) ? itTool->second : std::string();
			const std::string ctxToolId = (itCtx != ext.end()) ? itCtx->second : std::string();
			std::ostringstream d;
			d << "{\"preferUrdfIk\":" << (preferUrdfIk ? "true" : "false")
			  << ",\"constrainAxis\":" << (constrainAxis ? "true" : "false") << ",\"solvePath\":\"" << solvePath
			  << "\",\"targetQSize\":" << targetQ.size() << ",\"ikFailReason\":\"" << ikFailReason
			  << "\",\"motionToolId\":\"" << motionToolId << "\",\"ctxToolId\":\"" << ctxToolId << "\"}";
			agentDebugLogScene("H9", "PtpPlanner::plan", "ptp_solver_path", d.str());
		}
		// #endregion
		if (targetQ.empty())
		{
			if (errMsg)
			{
				*errMsg = ikFailReason.empty() ? "无DH上下文" : ikFailReason;
			}
			return false;
		}
		if (targetQ.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = "IK未收敛/超迭代";
			}
			return false;
		}
		double maxJointDelta = 0.0;
		for (size_t i = 0; i < targetQ.size(); ++i)
		{
			maxJointDelta = std::max(maxJointDelta, std::abs(targetQ[i] - q0[i]));
		}
		const double vmax = std::max(1e-6, cmd.speed() * kDegToRad);
		const double amax = std::max(1e-6, cmd.accel() * kDegToRad);
		const double durationSec = trapezoidDuration(maxJointDelta, vmax, amax);

		out.ok = true;
		out.plannerName = "PtpPlanner";
		out.summary = "PTP solved with joint target output.";
		out.durationSec = durationSec;
		out.jointTargetsRad = targetQ;
		out.jointTrajectoryRad = { targetQ };
		logIkSolveResidual(cmd, targetQ, "PtpPlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};

class LinePlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit LinePlanner(const std::vector<robot_kinematics::DhRow>* dhRows)
		: m_dhRows(dhRows)
	{
	}

	bool canHandle(RobotInstruction::Type type) const override { return type == RobotInstruction::Type::LINE; }

	bool validate(const RobotInstruction::Base& cmd, std::string* errMsg) const override
	{
		if (!cmd.hasPoseProperty())
		{
			if (errMsg)
				*errMsg = "LINE instruction requires target pose.";
			return false;
		}
		if (!cmd.hasSpeedProperty() || cmd.speed() <= 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction requires speed > 0.";
			return false;
		}
		if (!cmd.hasAccelProperty() || cmd.accel() <= 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction requires acceleration > 0.";
			return false;
		}
		if (cmd.hasBlendRadiusProperty() && cmd.blendRadius() < 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction blend radius must be >= 0.";
			return false;
		}
		return true;
	}

	bool plan(const RobotInstruction::Base& cmd, RobotInstruction::PlanResult& out, std::string* errMsg) const override
	{
		if (!validate(cmd, errMsg))
		{
			return false;
		}

		std::vector<double> q0 = currentJointVectorFromInstruction(cmd);
		if (q0.empty())
		{
			if (errMsg)
				*errMsg = "LINE planning failed: missing context.currentJointRadCsv.";
			return false;
		}

		std::vector<double> qTarget;
		std::string ikFailReason;
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		RobotInstruction::MotionAxisConfiguration axisCfg;
		if (cmd.hasMotionAxisConfigurationProperty())
		{
			axisCfg = cmd.motionAxisConfiguration();
		}
		const bool constrainAxis = cmd.hasMotionAxisConfigurationProperty()
			&& RobotInstruction::motionAxisConfigurationRequiresConstraint(axisCfg);
		if (preferUrdfIk)
		{
			if (cmd.hasMotionAxisConfigurationProperty())
			{
				qTarget = solveIkWithAxisConfiguration(cmd, &ikFailReason);
			}
			else
			{
				qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			}
			if (qTarget.empty() && !constrainAxis)
			{
				qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			}
		}
		if (qTarget.empty() && m_dhRows && !m_dhRows->empty() && !constrainAxis)
		{
			qTarget = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
		}
		if (qTarget.empty() && !preferUrdfIk)
		{
			if (cmd.hasMotionAxisConfigurationProperty())
			{
				qTarget = solveIkWithAxisConfiguration(cmd, &ikFailReason);
			}
			else if (!constrainAxis)
			{
				qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			}
		}
		if (qTarget.empty() && !constrainAxis)
		{
			qTarget = solveTargetByLegacyJointDelta(cmd);
		}
		if (qTarget.empty() || qTarget.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = ikFailReason.empty() ? "无DH上下文" : ikFailReason;
			}
			return false;
		}

		double durationSec = 0.0;
		double curTcp[3] = { 0.0, 0.0, 0.0 };
		const RobotInstruction::Vec3 p = cmd.pose();
		if (currentTcpPositionFromInstruction(cmd, curTcp))
		{
			const double dx = p.x - curTcp[0];
			const double dy = p.y - curTcp[1];
			const double dz = p.z - curTcp[2];
			const double cartDistMm = std::sqrt(dx * dx + dy * dy + dz * dz);
			durationSec = trapezoidDuration(cartDistMm, cmd.speed(), cmd.accel());
		}
		else
		{
			double maxJointDelta = 0.0;
			for (size_t i = 0; i < qTarget.size(); ++i)
			{
				maxJointDelta = std::max(maxJointDelta, std::abs(qTarget[i] - q0[i]));
			}
			durationSec = trapezoidDuration(
				maxJointDelta,
				std::max(1e-6, cmd.speed() * kDegToRad),
				std::max(1e-6, cmd.accel() * kDegToRad));
		}

		out.jointTrajectoryRad.clear();
		const int samples = 24;
		out.jointTrajectoryRad.reserve(static_cast<size_t>(samples + 1));
		for (int i = 1; i <= samples; ++i)
		{
			const double u = static_cast<double>(i) / static_cast<double>(samples);
			std::vector<double> q = q0;
			for (size_t j = 0; j < q.size(); ++j)
			{
				q[j] = q0[j] + (qTarget[j] - q0[j]) * u;
			}
			out.jointTrajectoryRad.push_back(std::move(q));
		}

		out.ok = true;
		out.plannerName = "LinePlanner";
		out.summary = "LINE solved with joint trajectory output.";
		out.durationSec = durationSec;
		out.jointTargetsRad = qTarget;
		logIkSolveResidual(cmd, qTarget, "LinePlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};
} // namespace

namespace RobotInstruction
{
void Controller::setDhRows(const std::vector<robot_kinematics::DhRow>& rows)
{
	m_dhRows = rows;
}

void Controller::clearDhRows()
{
	m_dhRows.clear();
}

bool Controller::hasDhRows() const
{
	return !m_dhRows.empty();
}

void Controller::registerPlanner(const std::shared_ptr<PlannerBase>& planner)
{
	if (planner)
	{
		m_planners.push_back(planner);
	}
}

void Controller::clearPlanners()
{
	m_planners.clear();
}

void Controller::buildDefaultPlanners()
{
	clearPlanners();
	registerPlanner(std::make_shared<PtpPlanner>(&m_dhRows));
	registerPlanner(std::make_shared<LinePlanner>(&m_dhRows));
}

bool Controller::validate(const Base& cmd, std::string* errMsg) const
{
	if (cmd.category() != Category::Motion)
	{
		return true;
	}
	const PlannerBase* planner = findPlanner(cmd.type());
	if (!planner)
	{
		if (errMsg)
		{
			*errMsg = "No planner is registered for instruction type.";
		}
		return false;
	}
	return planner->validate(cmd, errMsg);
}

bool Controller::plan(const Base& cmd, PlanResult& out, std::string* errMsg) const
{
	if (cmd.category() != Category::Motion)
	{
		out = PlanResult{};
		out.ok = true;
		out.plannerName = "logic";
		out.summary = "Logic instruction (no motion plan)";
		out.durationSec = 0.0;
		return true;
	}
	const PlannerBase* planner = findPlanner(cmd.type());
	if (!planner)
	{
		if (errMsg)
		{
			*errMsg = "No planner is registered for instruction type.";
		}
		return false;
	}
	return planner->plan(cmd, out, errMsg);
}

const PlannerBase* Controller::findPlanner(Type t) const
{
	const auto it = std::find_if(m_planners.begin(), m_planners.end(), [t](const std::shared_ptr<PlannerBase>& p) {
		return p && p->canHandle(t);
	});
	if (it == m_planners.end())
	{
		return nullptr;
	}
	return it->get();
}

namespace
{
void appendUniqueToken(std::vector<std::string>& out, const std::string& token)
{
	if (std::find(out.begin(), out.end(), token) == out.end())
	{
		out.push_back(token);
	}
}

bool tokenInList(const std::vector<std::string>& list, const std::string& token)
{
	return std::find(list.begin(), list.end(), token) != list.end();
}

FeasibleMotionAxisConfigurationOptions allAxisConfigurationEnumOptions()
{
	FeasibleMotionAxisConfigurationOptions out;
	out.presetTokens = motionAxisConfigPresetTokens();
	out.elbowTokens = elbowPostureTokens();
	out.wristTokens = wristPostureTokens();
	out.armTokens = armPostureTokens();
	out.turnJ1Tokens = motionAxisTurnTokens();
	out.turnJ4Tokens = motionAxisTurnTokens();
	out.turnJ6Tokens = motionAxisTurnTokens();
	return out;
}
} // namespace

FeasibleMotionAxisConfigurationOptions Controller::queryFeasibleMotionAxisConfigurationOptions(const Base& cmd) const
{
	FeasibleMotionAxisConfigurationOptions out;
	if (!cmd.hasMotionAxisConfigurationProperty())
	{
		return out;
	}
	if (!hasTcpLinkContext(cmd) || currentJointVectorFromInstruction(cmd).empty())
	{
		return allAxisConfigurationEnumOptions();
	}

	const std::vector<IkPostureClassEntry> postures = collectIkPostureClassesForTarget(cmd);
	if (postures.empty())
	{
		appendUniqueToken(out.presetTokens, "AUTO");
		appendUniqueToken(out.elbowTokens, "AUTO");
		appendUniqueToken(out.wristTokens, "AUTO");
		appendUniqueToken(out.armTokens, "AUTO");
		return out;
	}

	MotionAxisConfiguration autoCfg;
	autoCfg.preset = "AUTO";
	if (anyPostureMatchesConfiguration(postures, autoCfg))
	{
		appendUniqueToken(out.presetTokens, "AUTO");
	}

	for (const std::string& token : motionAxisConfigPresetTokens())
	{
		if (token == "AUTO" || token == "CUSTOM")
		{
			continue;
		}
		MotionAxisConfiguration cfg;
		cfg.preset = token;
		applyPresetToConfiguration(token, cfg);
		if (anyPostureMatchesConfiguration(postures, cfg))
		{
			appendUniqueToken(out.presetTokens, token);
		}
	}

	MotionAxisConfiguration customBase = cmd.motionAxisConfiguration();
	customBase.preset = "CUSTOM";

	for (const std::string& elbowTok : elbowPostureTokens())
	{
		MotionAxisConfiguration cfg = customBase;
		cfg.preset = "CUSTOM";
		ElbowPosture e{};
		if (!elbowPostureFromToken(elbowTok, e))
		{
			continue;
		}
		cfg.elbow = e;
		if (anyPostureMatchesConfiguration(postures, cfg))
		{
			appendUniqueToken(out.elbowTokens, elbowTok);
		}
	}
	for (const std::string& wristTok : wristPostureTokens())
	{
		MotionAxisConfiguration cfg = customBase;
		cfg.preset = "CUSTOM";
		WristPosture w{};
		if (!wristPostureFromToken(wristTok, w))
		{
			continue;
		}
		cfg.wrist = w;
		if (anyPostureMatchesConfiguration(postures, cfg))
		{
			appendUniqueToken(out.wristTokens, wristTok);
		}
	}
	for (const std::string& armTok : armPostureTokens())
	{
		MotionAxisConfiguration cfg = customBase;
		cfg.preset = "CUSTOM";
		ArmPosture a{};
		if (!armPostureFromToken(armTok, a))
		{
			continue;
		}
		cfg.arm = a;
		if (anyPostureMatchesConfiguration(postures, cfg))
		{
			appendUniqueToken(out.armTokens, armTok);
		}
	}

	const auto hasNonAuto = [](const std::vector<std::string>& tokens) {
		for (const std::string& t : tokens)
		{
			if (t != "AUTO")
			{
				return true;
			}
		}
		return false;
	};
	if (hasNonAuto(out.elbowTokens) || hasNonAuto(out.wristTokens) || hasNonAuto(out.armTokens))
	{
		appendUniqueToken(out.presetTokens, "CUSTOM");
	}

	if (out.presetTokens.empty())
	{
		appendUniqueToken(out.presetTokens, "AUTO");
	}
	if (out.elbowTokens.empty())
	{
		appendUniqueToken(out.elbowTokens, "AUTO");
	}
	if (out.wristTokens.empty())
	{
		appendUniqueToken(out.wristTokens, "AUTO");
	}
	if (out.armTokens.empty())
	{
		appendUniqueToken(out.armTokens, "AUTO");
	}

	const std::vector<IkPostureClassEntry>& posturePool = postures;
	auto collectTurnTokens = [&](int JointConfigurationClass::* turnMember, std::vector<std::string>& tokens) {
		tokens.clear();
		appendUniqueToken(tokens, "AUTO");
		for (const IkPostureClassEntry& e : posturePool)
		{
			const int t = e.cls.*turnMember;
			if (t != RobotInstruction::kMotionAxisTurnAuto)
			{
				appendUniqueToken(tokens, RobotInstruction::jointTurnToToken(t));
			}
		}
	};
	collectTurnTokens(&JointConfigurationClass::turnJ1, out.turnJ1Tokens);
	collectTurnTokens(&JointConfigurationClass::turnJ4, out.turnJ4Tokens);
	collectTurnTokens(&JointConfigurationClass::turnJ6, out.turnJ6Tokens);
	return out;
}
} // namespace RobotInstruction
