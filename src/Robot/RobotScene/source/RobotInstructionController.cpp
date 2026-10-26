/// @file RobotInstructionController.cpp
/// @brief Sole tool handling before IK: pose/euler = T_base_target; solver input = T_base_flange only.

#include "RobotInstructionController.h"

#include "BackendDataBase.h"
#include "CircularArcGeometry.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotTeachIk.h"
#include "RuckigPtpTrajectory.h"
#include "RunLogger.h"
#include "UrdfIkSolverOptions.h"
#include "IkAcceptanceGates.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"
#include "IkSeedExpand.h"
#include "KinematicCoreUrdfIk.h"
#include "SphericalWristAnalyticalIk.h"
#include "UrdfIkMachineProfile.h"

#include <QElapsedTimer>
#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>
#include <osg/Matrixd>
#include <osg/Quat>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/// 规划期碰撞软代价（Controller::setIkCollisionCostProbe）
RobotInstruction::Controller::IkCollisionCostFn g_ikCollisionCostProbe;

/// 规划期间生效的外轴配置（Controller::plan 设置）
const RobotExternal::RobotExternalAxisConfigSet* g_activeExternalAxes = nullptr;
const RobotInstruction::Controller::WorkpieceIkFrameContext* g_activeWorkpieceIkFrame = nullptr;
bool g_lastIkHasExternalAxisQ = false;
double g_lastIkExternalAxisQ = 0.0;
double g_lastIkSeedExternalAxisQ = 0.0;
std::vector<double> g_lastIkExternalAxisQs;
std::vector<double> g_lastIkSeedExternalAxisQs;

double trapezoidDuration(double distance, double vmax, double amax);

struct ActiveExternalAxesGuard
{
	explicit ActiveExternalAxesGuard(const RobotExternal::RobotExternalAxisConfigSet* axes)
	{
		g_activeExternalAxes = axes;
	}
	~ActiveExternalAxesGuard() { g_activeExternalAxes = nullptr; }
	ActiveExternalAxesGuard(const ActiveExternalAxesGuard&) = delete;
	ActiveExternalAxesGuard& operator=(const ActiveExternalAxesGuard&) = delete;
};

struct ActiveWorkpieceIkFrameGuard
{
	explicit ActiveWorkpieceIkFrameGuard(const RobotInstruction::Controller::WorkpieceIkFrameContext* ctx)
	{
		g_activeWorkpieceIkFrame = ctx;
	}
	~ActiveWorkpieceIkFrameGuard() { g_activeWorkpieceIkFrame = nullptr; }
	ActiveWorkpieceIkFrameGuard(const ActiveWorkpieceIkFrameGuard&) = delete;
	ActiveWorkpieceIkFrameGuard& operator=(const ActiveWorkpieceIkFrameGuard&) = delete;
};

double firstCompatExternalAxisQ(const std::vector<double>& qs, const RobotExternal::RobotExternalAxisConfigSet& set)
{
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	for (int idx : idxs)
	{
		if (idx >= 0 && idx < static_cast<int>(set.axes.size()) &&
			set.axes[static_cast<size_t>(idx)].attachment == RobotExternal::RobotExternalAttachment::RobotBase &&
			idx < static_cast<int>(qs.size()))
		{
			return qs[static_cast<size_t>(idx)];
		}
	}
	for (int idx : idxs)
	{
		if (idx >= 0 && idx < static_cast<int>(qs.size()))
		{
			return qs[static_cast<size_t>(idx)];
		}
	}
	return qs.empty() ? 0.0 : qs.front();
}

void clearLastIkExternalAxis()
{
	g_lastIkHasExternalAxisQ = false;
	g_lastIkExternalAxisQ = 0.0;
	g_lastIkSeedExternalAxisQ = 0.0;
	g_lastIkExternalAxisQs.clear();
	g_lastIkSeedExternalAxisQs.clear();
}

void noteLastIkExternalAxis(const std::vector<double>& qsFull)
{
	g_lastIkHasExternalAxisQ = true;
	g_lastIkExternalAxisQs = qsFull;
	if (g_activeExternalAxes)
	{
		g_lastIkExternalAxisQ = firstCompatExternalAxisQ(qsFull, *g_activeExternalAxes);
	}
	else
	{
		g_lastIkExternalAxisQ = qsFull.empty() ? 0.0 : qsFull.front();
	}
}

void noteLastIkExternalAxisSeed(const std::vector<double>& qsFull)
{
	g_lastIkSeedExternalAxisQs = qsFull;
	if (g_activeExternalAxes)
	{
		g_lastIkSeedExternalAxisQ = firstCompatExternalAxisQ(qsFull, *g_activeExternalAxes);
	}
	else
	{
		g_lastIkSeedExternalAxisQ = qsFull.empty() ? 0.0 : qsFull.front();
	}
}

void applyLastIkExternalAxisToPlan(RobotInstruction::PlanResult& out)
{
	if (!g_lastIkHasExternalAxisQ)
	{
		return;
	}
	out.hasExternalAxisQ = true;
	out.externalAxisQ = g_lastIkExternalAxisQ;
	out.externalAxisQs = g_lastIkExternalAxisQs;
	// 臂几乎不动、外轴行程大时，仅靠关节 Δq 估时会接近 0，播放像瞬移
	constexpr double kTranslateVMmPerSec = 250.0;
	constexpr double kTranslateAMmPerSec2 = 600.0;
	constexpr double kRotateVRadPerSec = 1.0;
	constexpr double kRotateARadPerSec2 = 2.0;
	const size_t n = std::max(g_lastIkExternalAxisQs.size(), g_lastIkSeedExternalAxisQs.size());
	for (size_t i = 0; i < n; ++i)
	{
		const double q = i < g_lastIkExternalAxisQs.size() ? g_lastIkExternalAxisQs[i] : 0.0;
		const double qSeed = i < g_lastIkSeedExternalAxisQs.size() ? g_lastIkSeedExternalAxisQs[i] : 0.0;
		const double dQ = std::abs(q - qSeed);
		bool isTranslate = true;
		if (g_activeExternalAxes && i < g_activeExternalAxes->axes.size())
		{
			isTranslate = g_activeExternalAxes->axes[i].motionType == RobotExternal::RobotExternalMotionType::Translate;
		}
		if (isTranslate)
		{
			out.durationSec =
				std::max(out.durationSec, trapezoidDuration(dQ, kTranslateVMmPerSec, kTranslateAMmPerSec2));
		}
		else
		{
			out.durationSec = std::max(out.durationSec, trapezoidDuration(dQ, kRotateVRadPerSec, kRotateARadPerSec2));
		}
	}
}

bool coupledExternalIkRequired()
{
	return g_activeExternalAxes && RobotExternal::hasEnabledExternalAxes(*g_activeExternalAxes);
}

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
	double pos[3] = {0.0, 0.0, 0.0};
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
		// 脏矩阵当单位阵会把工具偏置吞掉，法兰目标静默偏一截
		if (!RobotCoordinate::parseMat4Csv(itTool->second, T_flange_tool))
		{
			return false;
		}
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
	const engine::RigidTransform T_base_link = engine::flangeFromToolOrigin(T_base_target, T_tool);
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

std::vector<double> solveTargetByIkIfPossible(const RobotInstruction::Base& cmd,
											  const std::vector<robot_kinematics::DhRow>& dhRows,
											  std::string* failReason)
{
	if (dhRows.empty() || !cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = dhRows.empty() ? "缺少DH参数" : "缺少目标位姿";
		}
		return {};
	}
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "缺少IK种子关节角";
		}
		return {};
	}
	const std::size_t nJoint = robot_kinematics::jointCountFromDhRows(dhRows);
	if (nJoint == 0 || q.size() != nJoint)
	{
		if (failReason)
		{
			*failReason = "DH关节数与种子关节数不一致";
		}
		return {};
	}
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		if (failReason)
		{
			*failReason = "无法解析目标法兰位姿";
		}
		return {};
	}
	const double targetPos[3] = {linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};

	double reachMm = 0.0;
	for (const robot_kinematics::DhRow& row : dhRows)
	{
		reachMm += std::sqrt(row.a * row.a + row.d * row.d);
	}
	reachMm = std::max(reachMm, 1.0);
	const double targetNormMm =
		std::sqrt(targetPos[0] * targetPos[0] + targetPos[1] * targetPos[1] + targetPos[2] * targetPos[2]);
	if (targetNormMm > reachMm * 2.5 || targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标超出近似臂展(DH)";
		}
		return {};
	}

	double curPos[3] = {0.0, 0.0, 0.0};
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
				*failReason = "目标相对当前TCP过远(DH)";
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
			double endPos[3] = {0.0, 0.0, 0.0};
			if (robot_kinematics::endEffectorPosition(dhRows, qSolved, endPos))
			{
				const double dx = targetPos[0] - endPos[0];
				const double dy = targetPos[1] - endPos[1];
				const double dz = targetPos[2] - endPos[2];
				const double residual = std::sqrt(dx * dx + dy * dy + dz * dz);
				std::ostringstream oss;
				oss.setf(std::ios::fixed);
				oss.precision(2);
				oss << "DH IK未收敛；位置残差" << residual << "mm";
				*failReason = oss.str();
			}
			else
			{
				*failReason = "DH IK未收敛";
			}
		}
		return {};
	}
	(void)iters;
	return qSolved;
}

bool tcpPositionFromUrdf(const QString& urdfPath, const QString& tcpLink, const std::vector<double>& q,
						 double outPos[3], osg::Quat* outRot = nullptr)
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
	return tcpPositionFromUrdf(QString::fromStdString(itUrdf->second), QString::fromStdString(itTcp->second), q,
							   outPos);
}

/// 由关节角求基座系工具原点（LINE 笛卡尔起点）
bool toolOriginTransformFromJoints(const RobotInstruction::Base& cmd, const std::vector<double>& q,
								   engine::RigidTransform& outToolOrigin)
{
	if (q.empty())
	{
		return false;
	}
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		return false;
	}
	std::string flangeLink;
	const auto itFlange = ext.find("context.flangeLinkName");
	const auto itTcp = ext.find("context.tcpLinkName");
	BackendMat4 T_flange_tool = BackendMat4::identity();
	const auto itTool = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	bool toolHasOffset = false;
	if (itTool != ext.end() && !itTool->second.empty())
	{
		if (!RobotCoordinate::parseMat4Csv(itTool->second, T_flange_tool))
		{
			return false;
		}
		const BackendMat4 id = BackendMat4::identity();
		for (int i = 0; i < 16; ++i)
		{
			if (std::abs(T_flange_tool.v[i] - id.v[i]) > 1e-9)
			{
				toolHasOffset = true;
				break;
			}
		}
	}
	if (toolHasOffset)
	{
		if (itFlange == ext.end() || itFlange->second.empty())
		{
			return false;
		}
		flangeLink = itFlange->second;
	}
	else
	{
		flangeLink = (itTcp != ext.end() && !itTcp->second.empty())
						 ? itTcp->second
						 : ((itFlange != ext.end()) ? itFlange->second : std::string());
		if (flangeLink.empty())
		{
			return false;
		}
	}
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(q.size()));
	for (double v : q)
	{
		qQt.push_back(v);
	}
	QHash<QString, osg::Matrixd> linkWorld;
	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const QString flangeLinkQ = QString::fromStdString(flangeLink);
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, qQt, linkWorld, nullptr) ||
		!linkWorld.contains(flangeLinkQ))
	{
		return false;
	}
	const BackendMat4 toolMat =
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(flangeLinkQ), T_flange_tool);
	outToolOrigin = RobotCoordinate::rigidTransformFromBackendMat4(toolMat);
	return true;
}

/// 工具原点 FK 相对指令 T_base_target 的残差（与 UI 门控一致，勿用法兰 DLS 内部误差）
bool measureToolTargetResidual(const RobotInstruction::Base& cmd, const std::vector<double>& q, double& posMm,
							   double& rotDeg)
{
	engine::RigidTransform target{};
	engine::RigidTransform fk{};
	if (!RobotInstruction::readTargetTransformFromInstruction(cmd, target))
	{
		return false;
	}
	if (!toolOriginTransformFromJoints(cmd, q, fk))
	{
		return false;
	}
	double tx = 0.0;
	double ty = 0.0;
	double tz = 0.0;
	double fx = 0.0;
	double fy = 0.0;
	double fz = 0.0;
	target.translationMm(tx, ty, tz);
	fk.translationMm(fx, fy, fz);
	const double dx = tx - fx;
	const double dy = ty - fy;
	const double dz = tz - fz;
	posMm = std::sqrt(dx * dx + dy * dy + dz * dz);
	const Eigen::Quaterniond qErr = target.rotation().inverse() * fk.rotation();
	const double w = std::clamp(std::abs(qErr.w()), 0.0, 1.0);
	rotDeg = 2.0 * std::acos(w) / kDegToRad;
	return true;
}

void logIkSolveResidual(const RobotInstruction::Base& cmd, const std::vector<double>& qSolved, const char* plannerName)
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
	const auto itTcp = ext.find("context.tcpLinkName");

	const QString urdfPath = QString::fromStdString(itUrdf->second);

	engine::RigidTransform T_base_target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(cmd, T_base_target))
	{
		return;
	}
	double toolOriginPos[3]{};
	T_base_target.translationMm(toolOriginPos[0], toolOriginPos[1], toolOriginPos[2]);

	BackendMat4 T_flange_tool = BackendMat4::identity();
	const auto itTool = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	bool toolHasOffset = false;
	if (itTool != ext.end() && !itTool->second.empty())
	{
		if (!RobotCoordinate::parseMat4Csv(itTool->second, T_flange_tool))
		{
			return;
		}
		const BackendMat4 id = BackendMat4::identity();
		for (int i = 0; i < 16; ++i)
		{
			if (std::abs(T_flange_tool.v[i] - id.v[i]) > 1e-9)
			{
				toolHasOffset = true;
				break;
			}
		}
	}
	if (toolHasOffset)
	{
		if (itFlange == ext.end() || itFlange->second.empty())
		{
			return;
		}
		flangeLink = itFlange->second;
	}
	else
	{
		flangeLink = (itTcp != ext.end() && !itTcp->second.empty())
						 ? itTcp->second
						 : ((itFlange != ext.end()) ? itFlange->second : std::string());
		if (flangeLink.empty())
		{
			return;
		}
	}
	const QString flangeLinkQ = QString::fromStdString(flangeLink);

	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		return;
	}
	const RobotInstruction::Vec3 flangeTarget{linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};
	const bool hasTargetEuler = linkTarget.hasOrientation;

	double fkFlangePos[3] = {0.0, 0.0, 0.0};
	osg::Quat fkFlangeRot;
	if (!tcpPositionFromUrdf(urdfPath, flangeLinkQ, qSolved, fkFlangePos, &fkFlangeRot))
	{
		std::ostringstream os;
		os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown") << ", flangeLink=" << flangeLink
		   << ", FK failed (cannot recompute flange pose from q)";
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
		if (UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, qQt, linkWorld, nullptr) &&
			linkWorld.contains(flangeLinkQ))
		{
			const BackendMat4 fkTcpMat =
				RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(flangeLinkQ), T_flange_tool);
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
	const double residualFlangeMm = std::sqrt(dFlangeX * dFlangeX + dFlangeY * dFlangeY + dFlangeZ * dFlangeZ);

	const double dTcpX = fkToolOriginPos[0] - toolOriginPos[0];
	const double dTcpY = fkToolOriginPos[1] - toolOriginPos[1];
	const double dTcpZ = fkToolOriginPos[2] - toolOriginPos[2];
	const double residualTcpMm = std::sqrt(dTcpX * dTcpX + dTcpY * dTcpY + dTcpZ * dTcpZ);

	double residualTcpRotDeg = 0.0;
	double residualFlangeRotDeg = 0.0;
	if (hasTargetEuler)
	{
		const osg::Quat targetFlangeQuat = linkTarget.quat;
		double eRot[3] = {0.0, 0.0, 0.0};
		quatErrorAxisAngle(fkFlangeRot, targetFlangeQuat, eRot);
		const double errRotRad = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		residualFlangeRotDeg = errRotRad * (180.0 / 3.14159265358979323846);

		if (hasFkToolRot)
		{
			residualTcpRotDeg = T_base_target.rotationErrorDeg(fkToolRt);
		}
	}

	std::ostringstream os;
	os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown") << ", flangeLink=" << flangeLink
	   << ", toolOrigin=(" << toolOriginPos[0] << ", " << toolOriginPos[1] << ", " << toolOriginPos[2] << ")"
	   << ", flangeTarget=(" << flangeTarget.x << ", " << flangeTarget.y << ", " << flangeTarget.z << ")"
	   << ", fkFlange=(" << fkFlangePos[0] << ", " << fkFlangePos[1] << ", " << fkFlangePos[2] << ")"
	   << ", fkToolOrigin=(" << fkToolOriginPos[0] << ", " << fkToolOriginPos[1] << ", " << fkToolOriginPos[2] << ")"
	   << ", solvedQ[0..]=[" << qPreview.str() << "]"
	   << ", residualTcpMm=" << residualTcpMm << ", residualFlangeMm=" << residualFlangeMm;
	if (hasTargetEuler)
	{
		os << ", residualTcpRotDeg=" << residualTcpRotDeg << ", residualFlangeRotDeg=" << residualFlangeRotDeg;
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
			std::transform(lower.begin(), lower.end(), lower.begin(),
						   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
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

std::string formatRadAsDegText(const double rad)
{
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(1);
	oss << (rad / kDegToRad);
	return oss.str();
}

/// 列出超限位关节；无超限返回 false
bool formatJointLimitViolations(const QString& urdfPath, const std::vector<double>& q, std::string& outMsg)
{
	QStringList names;
	QVector<double> lower;
	QVector<double> upper;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(urdfPath, names, lower, upper, nullptr))
	{
		return false;
	}
	const int n = std::min(static_cast<int>(q.size()), std::min(names.size(), std::min(lower.size(), upper.size())));
	if (n <= 0)
	{
		return false;
	}
	constexpr double kEpsRad = 1e-4;
	std::ostringstream oss;
	int hit = 0;
	for (int i = 0; i < n; ++i)
	{
		const double qi = q[static_cast<size_t>(i)];
		const double lo = lower[i];
		const double hi = upper[i];
		if (qi >= lo - kEpsRad && qi <= hi + kEpsRad)
		{
			continue;
		}
		if (hit > 0)
		{
			oss << "; ";
		}
		const QString label = names[i].isEmpty() ? QStringLiteral("J%1").arg(i + 1) : names[i];
		oss << label.toStdString() << "=" << formatRadAsDegText(qi) << "°";
		if (qi < lo - kEpsRad)
		{
			oss << "<下限" << formatRadAsDegText(lo) << "°";
		}
		else
		{
			oss << ">上限" << formatRadAsDegText(hi) << "°";
		}
		++hit;
	}
	if (hit <= 0)
	{
		return false;
	}
	outMsg = "关节超限: " + oss.str();
	return true;
}

// 同构角折回 URDF 区间：跑完后种子常落在 ±2π 外侧，二次跳转 IK 会“收敛但超限”
void foldJointsIntoUrdfLimits(const QString& urdfPath, std::vector<double>& q)
{
	QStringList names;
	QVector<double> lower;
	QVector<double> upper;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(urdfPath, names, lower, upper, nullptr))
	{
		return;
	}
	const int n = std::min(static_cast<int>(q.size()), std::min(names.size(), std::min(lower.size(), upper.size())));
	if (n <= 0)
	{
		return;
	}
	constexpr double kTwoPi = 6.2831853071795864769;
	constexpr double kEpsRad = 1e-4;
	for (int i = 0; i < n; ++i)
	{
		const double lo = lower[i];
		const double hi = upper[i];
		double qi = q[static_cast<size_t>(i)];
		if (qi >= lo - kEpsRad && qi <= hi + kEpsRad)
		{
			continue;
		}
		bool found = false;
		double best = qi;
		int bestAbsK = 0;
		for (int k = -16; k <= 16; ++k)
		{
			const double c = qi + static_cast<double>(k) * kTwoPi;
			if (c < lo - kEpsRad || c > hi + kEpsRad)
			{
				continue;
			}
			const int absK = k < 0 ? -k : k;
			if (!found || absK < bestAbsK)
			{
				best = c;
				bestAbsK = absK;
				found = true;
			}
		}
		if (found)
		{
			q[static_cast<size_t>(i)] = best;
		}
	}
}

std::string formatIkResidualText(const double posErrMm, const bool useOrientation, const double rotErrRad)
{
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(2);
	oss << "位置残差" << posErrMm << "mm";
	if (useOrientation)
	{
		oss.precision(1);
		oss << " 姿态残差" << (rotErrRad / kDegToRad) << "°";
	}
	return oss.str();
}

std::vector<std::vector<double>> buildIkSeedVariants(const std::vector<double>& qSeed,
													 const std::vector<std::string>& jointNames,
													 const RobotInstruction::MotionAxisConfiguration* axisCfg,
													 const bool sphericalWrist = true,
													 const int elbowJointIndexOverride = -1,
													 const int wristJointIndexOverride = -1)
{
	std::vector<std::vector<double>> seeds = UrdfRobotLoader::expandPoseIkSeeds(
		qSeed, jointNames, nullptr, sphericalWrist, elbowJointIndexOverride, wristJointIndexOverride);
	auto pushUnique = [&](std::vector<double> q)
	{
		for (const auto& existing : seeds)
		{
			if (jointVectorDistance(existing, q) < 1e-6)
			{
				return;
			}
		}
		seeds.push_back(std::move(q));
	};
	const int n = static_cast<int>(qSeed.size());
	int elbowIdx = elbowJointIndexOverride;
	if (elbowIdx < 0 || elbowIdx >= n)
	{
		elbowIdx = jointIndexByNameHint(jointNames, "elbow", 2);
	}
	int wristIdx = wristJointIndexOverride;
	if (wristIdx < 0 || wristIdx >= n)
	{
		wristIdx = jointIndexByNameHint(jointNames, "wrist", 4);
	}
	const int j1Idx = jointIndexByNameHint(jointNames, nullptr, 0);
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
		// 腕+π / 解析向扰动仅球形腕有意义
		if (sphericalWrist && wristIdx >= 0)
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
	const RobotInstruction::Base& cmd, std::vector<double> q, std::string* failReason,
	const bool expandSeedVariants = true, const bool preferNumericalOnly = false,
	const bool raceAnalyticalAndNumerical = false, const double seedAttractionWeight = 0.0)
{
	if (!cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = "缺少目标位姿";
		}
		return {};
	}
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		if (failReason)
		{
			*failReason = "缺少URDF路径";
		}
		return {};
	}
	const auto itTcp = ext.find("context.tcpLinkName");
	const auto itFlange = ext.find("context.flangeLinkName");
	const auto itToolMat = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	const bool hasToolMatCsv = itToolMat != ext.end() && !itToolMat->second.empty();
	BackendMat4 T_flange_toolProbe = BackendMat4::identity();
	bool toolHasOffset = false;
	if (hasToolMatCsv)
	{
		if (!RobotCoordinate::parseMat4Csv(itToolMat->second, T_flange_toolProbe))
		{
			if (failReason)
			{
				*failReason = "工具矩阵解析失败";
			}
			return {};
		}
		const BackendMat4 id = BackendMat4::identity();
		for (int i = 0; i < 16; ++i)
		{
			if (std::abs(T_flange_toolProbe.v[i] - id.v[i]) > 1e-9)
			{
				toolHasOffset = true;
				break;
			}
		}
	}
	// 有真实工具偏置：目标是工具原点，必须解法兰；否则优先示教 tcpLink
	std::string ikLinkName;
	if (toolHasOffset)
	{
		if (itFlange == ext.end() || itFlange->second.empty())
		{
			if (failReason)
			{
				*failReason = "有工具偏置时缺少法兰连杆名";
			}
			return {};
		}
		ikLinkName = itFlange->second;
	}
	else
	{
		ikLinkName = (itTcp != ext.end() && !itTcp->second.empty())
						 ? itTcp->second
						 : ((itFlange != ext.end()) ? itFlange->second : std::string());
		// FANUC 等：示教 FK 在 link_6，tcp 却落成固定叶 flange(Rxπ) → 姿态差 180° 永不可达
		if (itFlange != ext.end() && !itFlange->second.empty() && !ikLinkName.empty() && ikLinkName != itFlange->second)
		{
			ikLinkName = itFlange->second;
		}
	}
	if (ikLinkName.empty())
	{
		if (failReason)
		{
			*failReason = "缺少法兰/TCP连杆名";
		}
		return {};
	}
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "缺少IK种子关节角";
		}
		return {};
	}
	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const std::vector<double> seedAttractionQ = q;
	foldJointsIntoUrdfLimits(urdfPath, q);
	const QString ikLink = QString::fromStdString(ikLinkName);
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromInstruction(cmd, linkTarget))
	{
		if (failReason)
		{
			*failReason = "无法解析目标法兰位姿";
		}
		return {};
	}
	const double target[3] = {linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};
	const bool useOrientation = linkTarget.hasOrientation;
	const osg::Quat targetQuat = useOrientation ? linkTarget.quat : osg::Quat();
	BackendMat4 T_flange_tool = T_flange_toolProbe;
	const engine::RigidTransform flangeToolRt = RobotCoordinate::rigidTransformFromBackendMat4(T_flange_tool);
	(void)flangeToolRt;

	clearLastIkExternalAxis();
	if (g_activeExternalAxes && RobotExternal::hasEnabledExternalAxes(*g_activeExternalAxes))
	{
		// 指令存的是 T_p0；REP 时再经工作架采样重建目标
		engine::RigidTransform T_p0_stored{};
		if (!RobotInstruction::readTargetTransformFromInstruction(cmd, T_p0_stored))
		{
			if (failReason)
			{
				*failReason = "无法解析外轴联立目标位姿";
			}
			return {};
		}

		const RobotExternal::RobotExternalAxisConfigSet& axisSet = *g_activeExternalAxes;
		const std::vector<int> enabledIdx = RobotExternal::enabledExternalAxisIndices(axisSet);
		const std::vector<int> robotBaseIdx = RobotExternal::enabledExternalAxisIndicesForAttachment(
			axisSet, RobotExternal::RobotExternalAttachment::RobotBase);
		const std::vector<int> workpieceIdx = RobotExternal::enabledExternalAxisIndicesForAttachment(
			axisSet, RobotExternal::RobotExternalAttachment::Workpiece);
		const int configCount = static_cast<int>(axisSet.axes.size());

		std::vector<double> qeSeedFull(static_cast<size_t>(configCount), 0.0);
		for (size_t i = 0; i < axisSet.axes.size(); ++i)
		{
			qeSeedFull[i] = std::clamp(axisSet.axes[i].home, axisSet.axes[i].lower, axisSet.axes[i].upper);
		}
		{
			const auto& extProps = cmd.extensionProperties();
			const auto itCsv = extProps.find(RobotExternal::kExtContextExternalAxisQCsv);
			if (itCsv != extProps.end() && !itCsv->second.empty())
			{
				const std::vector<double> parsed = RobotExternal::parseExternalAxisQCsv(itCsv->second);
				for (size_t i = 0; i < qeSeedFull.size() && i < parsed.size(); ++i)
				{
					qeSeedFull[i] = std::clamp(parsed[i], axisSet.axes[i].lower, axisSet.axes[i].upper);
				}
			}
			else
			{
				const auto itQ = extProps.find(RobotExternal::kExtContextExternalAxisQMm);
				if (itQ != extProps.end() && !itQ->second.empty())
				{
					try
					{
						const double qScalar = std::stod(itQ->second);
						for (int idx : enabledIdx)
						{
							if (idx >= 0 && idx < configCount &&
								axisSet.axes[static_cast<size_t>(idx)].attachment ==
									RobotExternal::RobotExternalAttachment::RobotBase)
							{
								qeSeedFull[static_cast<size_t>(idx)] =
									std::clamp(qScalar, axisSet.axes[static_cast<size_t>(idx)].lower,
											   axisSet.axes[static_cast<size_t>(idx)].upper);
								break;
							}
						}
					}
					catch (...)
					{
					}
				}
			}
		}
		noteLastIkExternalAxisSeed(qeSeedFull);

		auto rigidFromColMajor16 = [](const double m[16]) -> engine::RigidTransform
		{
			BackendMat4 bm{};
			for (int i = 0; i < 16; ++i)
			{
				bm.v[i] = m[i];
			}
			return RobotCoordinate::rigidTransformFromBackendMat4(bm);
		};

		auto buildDofFromIndices = [&](const std::vector<int>& idxs,
									   const std::vector<double>& qeFull) -> RobotTeachIk::TeachIkExternalAxisDof
		{
			RobotTeachIk::TeachIkExternalAxisDof dof;
			dof.qExternal.reserve(idxs.size());
			for (int idx : idxs)
			{
				if (idx < 0 || idx >= configCount)
				{
					continue;
				}
				const RobotExternal::RobotExternalAxisConfig& cfg = axisSet.axes[static_cast<size_t>(idx)];
				RobotTeachIk::TeachIkExternalAxisSlot slot;
				slot.configIndex = idx;
				slot.isPrismatic = cfg.motionType == RobotExternal::RobotExternalMotionType::Translate;
				slot.axis[0] = cfg.axis[0];
				slot.axis[1] = cfg.axis[1];
				slot.axis[2] = cfg.axis[2];
				slot.originMm[0] = cfg.originMm[0];
				slot.originMm[1] = cfg.originMm[1];
				slot.originMm[2] = cfg.originMm[2];
				slot.lower = cfg.lower;
				slot.upper = cfg.upper;
				dof.axes.push_back(slot);
				dof.qExternal.push_back(qeFull[static_cast<size_t>(idx)]);
			}
			return dof;
		};

		auto appendSparseGridSamples =
			[](const RobotTeachIk::TeachIkExternalAxisDof& dof, std::vector<std::vector<double>>& samples)
		{
			const int dofN = static_cast<int>(dof.axes.size());
			if (dofN <= 0)
			{
				return;
			}
			if (dofN == 1)
			{
				const auto& ax = dof.axes.front();
				const double span = std::max(0.0, ax.upper - ax.lower);
				const int gridN = span < 1e-9 ? 1 : std::clamp(static_cast<int>(span / 200.0) + 1, 3, 7);
				for (int i = 0; i < gridN; ++i)
				{
					const double t = gridN <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(gridN - 1);
					samples.push_back({ax.lower + t * (ax.upper - ax.lower)});
				}
				return;
			}
			std::vector<int> gridN(static_cast<size_t>(dofN), 1);
			int total = 1;
			for (int i = 0; i < dofN; ++i)
			{
				const double span =
					std::max(0.0, dof.axes[static_cast<size_t>(i)].upper - dof.axes[static_cast<size_t>(i)].lower);
				gridN[static_cast<size_t>(i)] =
					span < 1e-9 ? 1 : std::clamp(static_cast<int>(span / 250.0) + 1, 2, dofN >= 3 ? 3 : 5);
				total *= gridN[static_cast<size_t>(i)];
			}
			while (total > 64)
			{
				int maxIdx = 0;
				for (int i = 1; i < dofN; ++i)
				{
					if (gridN[static_cast<size_t>(i)] > gridN[static_cast<size_t>(maxIdx)])
					{
						maxIdx = i;
					}
				}
				if (gridN[static_cast<size_t>(maxIdx)] <= 2)
				{
					break;
				}
				total /= gridN[static_cast<size_t>(maxIdx)];
				--gridN[static_cast<size_t>(maxIdx)];
				total *= gridN[static_cast<size_t>(maxIdx)];
			}
			std::vector<int> idx(static_cast<size_t>(dofN), 0);
			for (;;)
			{
				std::vector<double> sample(static_cast<size_t>(dofN), 0.0);
				for (int i = 0; i < dofN; ++i)
				{
					const auto& ax = dof.axes[static_cast<size_t>(i)];
					const int gn = gridN[static_cast<size_t>(i)];
					const double t =
						gn <= 1 ? 0.0 : static_cast<double>(idx[static_cast<size_t>(i)]) / static_cast<double>(gn - 1);
					sample[static_cast<size_t>(i)] = ax.lower + t * (ax.upper - ax.lower);
				}
				samples.push_back(std::move(sample));
				int carry = 0;
				++idx[0];
				while (carry < dofN && idx[static_cast<size_t>(carry)] >= gridN[static_cast<size_t>(carry)])
				{
					idx[static_cast<size_t>(carry)] = 0;
					++carry;
					if (carry < dofN)
					{
						++idx[static_cast<size_t>(carry)];
					}
				}
				if (carry >= dofN)
				{
					break;
				}
			}
		};

		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = urdfPath;
		ctx.ikLinkName = ikLink;
		ctx.seedJointRad = q;
		ctx.useOrientation = useOrientation;
		ctx.T_flange_tool = T_flange_tool;
		ctx.externalAxisConfigCount = configCount;
		ctx.externalAxis.enabled = false;
		// 指令 pose 已是基座系工具原点（P0）；此处勿再按用户系解释
		{
			const auto& extOpt = cmd.extensionProperties();
			const auto itApprox = extOpt.find("context.allowApproximateOrientation");
			if (itApprox != extOpt.end() && (itApprox->second == "1" || itApprox->second == "true"))
			{
				ctx.options.allowApproximateOrientation = true;
			}
		}

		double bestRes = std::numeric_limits<double>::infinity();
		std::vector<double> bestQ;
		std::vector<double> bestQsFull = qeSeedFull;
		bool bestOk = false;
		std::string lastTeachError;
		double lastTeachResMm = std::numeric_limits<double>::infinity();

		auto considerDof = [&](RobotTeachIk::TeachIkExternalAxisDof tryDof, const bool optimize, const int maxIters,
							   const std::vector<double>& qeTryFull)
		{
			tryDof.optimizeExternal = optimize;
			tryDof.adaptiveExternalDamping = true;
			ctx.externalAxes = tryDof;
			ctx.maxIkIterations = maxIters;
			ctx.seedJointRad = bestOk && !bestQ.empty() ? bestQ : q;
			const RobotTeachIk::TeachIkResult r = RobotTeachIk::solveTeachIk(ctx);
			if (!r.ok)
			{
				if (!r.error.empty())
				{
					lastTeachError = r.error;
				}
				if (r.residualTcpMm > 0.0 && r.residualTcpMm < lastTeachResMm)
				{
					lastTeachResMm = r.residualTcpMm;
				}
				return;
			}
			if (r.residualTcpMm < bestRes)
			{
				bestRes = r.residualTcpMm;
				bestQ = r.jointRad;
				bestOk = true;
				// TeachIk 只填 RobotBase 槽；工件 q 保留外层采样
				bestQsFull = qeTryFull;
				if (static_cast<int>(r.externalAxisQs.size()) == configCount)
				{
					for (int idx : robotBaseIdx)
					{
						if (idx >= 0 && idx < configCount)
						{
							bestQsFull[static_cast<size_t>(idx)] = r.externalAxisQs[static_cast<size_t>(idx)];
						}
					}
				}
				else
				{
					for (size_t i = 0; i < tryDof.axes.size() && i < r.externalAxisQs.size(); ++i)
					{
						const int cidx = tryDof.axes[i].configIndex;
						if (cidx >= 0 && cidx < configCount)
						{
							bestQsFull[static_cast<size_t>(cidx)] = r.externalAxisQs[i];
						}
					}
					if (r.externalAxisQs.empty() && !tryDof.qExternal.empty())
					{
						for (size_t i = 0; i < tryDof.axes.size(); ++i)
						{
							const int cidx = tryDof.axes[i].configIndex;
							if (cidx >= 0 && cidx < configCount)
							{
								bestQsFull[static_cast<size_t>(cidx)] = tryDof.qExternal[i];
							}
						}
					}
				}
			}
		};

		auto runRobotBaseSearch = [&](const engine::RigidTransform& T_p0_goal, const std::vector<double>& qeTryFull)
		{
			ctx.T_base_target = T_p0_goal;
			RobotTeachIk::TeachIkExternalAxisDof dof = buildDofFromIndices(robotBaseIdx, qeTryFull);
			const int dofN = static_cast<int>(dof.axes.size());
			const double resAtEntry = bestRes;

			considerDof(dof, false, 48, qeTryFull);
			if (bestOk && bestRes < 1.5 && dofN <= 2)
			{
				RobotTeachIk::TeachIkExternalAxisDof refine = dof;
				for (size_t i = 0; i < refine.axes.size(); ++i)
				{
					const int cidx = refine.axes[i].configIndex;
					if (cidx >= 0 && cidx < configCount)
					{
						refine.qExternal[i] = bestQsFull[static_cast<size_t>(cidx)];
					}
				}
				refine.externalDeltaPriorWeight = 0.05;
				considerDof(refine, true, 56, qeTryFull);
			}

			// 其它工件样本的优解不能跳过本轮 RobotBase 网格
			if (!bestOk || bestRes > 3.0 || !(bestRes < resAtEntry))
			{
				std::vector<std::vector<double>> samples;
				samples.push_back(dof.qExternal);
				appendSparseGridSamples(dof, samples);
				for (size_t si = 1; si < samples.size(); ++si)
				{
					RobotTeachIk::TeachIkExternalAxisDof tryDof = dof;
					tryDof.qExternal = samples[si];
					considerDof(tryDof, false, 36, qeTryFull);
					if (bestOk && bestRes < 1.0)
					{
						break;
					}
				}
				if (bestOk && dofN <= 2)
				{
					RobotTeachIk::TeachIkExternalAxisDof refine = dof;
					for (size_t i = 0; i < refine.axes.size(); ++i)
					{
						const int cidx = refine.axes[i].configIndex;
						if (cidx >= 0 && cidx < configCount)
						{
							refine.qExternal[i] = bestQsFull[static_cast<size_t>(cidx)];
						}
					}
					refine.externalDeltaPriorWeight = 0.03;
					considerDof(refine, true, 64, qeTryFull);
				}
			}
		};

		const bool useWorkpieceRep = RobotExternal::hasEnabledWorkpieceExternalAxes(axisSet) &&
									 g_activeWorkpieceIkFrame && g_activeWorkpieceIkFrame->valid;

		if (useWorkpieceRep)
		{
			engine::RigidTransform T_work{};
			if (!RobotInstruction::readWorkingTcpFromInstruction(cmd, T_work))
			{
				double tp0WorkSeed[16];
				if (!RobotExternal::composeWorkpieceWorkingFrameInRobotP0(
						g_activeWorkpieceIkFrame->p0World.data(), g_activeWorkpieceIkFrame->w0World.data(), axisSet,
						g_activeWorkpieceIkFrame->boundBackendId, qeSeedFull,
						g_activeWorkpieceIkFrame->offsetW0Local.data(), tp0WorkSeed))
				{
					if (failReason)
					{
						*failReason = "无法合成工件工作架（检查 P0/W0/绑定）";
					}
					return {};
				}
				const engine::RigidTransform T_p0_work_seed = rigidFromColMajor16(tp0WorkSeed);
				T_work = T_p0_work_seed.inverse().composeColumn(T_p0_stored);
			}

			RobotTeachIk::TeachIkExternalAxisDof wpDof = buildDofFromIndices(workpieceIdx, qeSeedFull);
			std::vector<std::vector<double>> wpSamples;
			wpSamples.push_back(wpDof.qExternal);
			appendSparseGridSamples(wpDof, wpSamples);

			for (const std::vector<double>& qw : wpSamples)
			{
				std::vector<double> qeTryFull = qeSeedFull;
				for (size_t i = 0; i < workpieceIdx.size() && i < qw.size(); ++i)
				{
					const int cidx = workpieceIdx[i];
					if (cidx >= 0 && cidx < configCount)
					{
						qeTryFull[static_cast<size_t>(cidx)] = qw[i];
					}
				}

				double tp0Work[16];
				if (!RobotExternal::composeWorkpieceWorkingFrameInRobotP0(
						g_activeWorkpieceIkFrame->p0World.data(), g_activeWorkpieceIkFrame->w0World.data(), axisSet,
						g_activeWorkpieceIkFrame->boundBackendId, qeTryFull,
						g_activeWorkpieceIkFrame->offsetW0Local.data(), tp0Work))
				{
					continue;
				}
				const engine::RigidTransform T_p0_goal = rigidFromColMajor16(tp0Work).composeColumn(T_work);
				runRobotBaseSearch(T_p0_goal, qeTryFull);
				if (bestOk && bestRes < 1.0)
				{
					break;
				}
			}
		}
		else
		{
			runRobotBaseSearch(T_p0_stored, qeSeedFull);
		}

		if (bestOk && !bestQ.empty())
		{
			// TeachIk 已按 IkAcceptanceGates 收紧 ok；此处不再重测残差
			noteLastIkExternalAxis(bestQsFull);
			return bestQ;
		}
		if (failReason)
		{
			if (!lastTeachError.empty())
			{
				*failReason = lastTeachError;
			}
			else if (lastTeachResMm < std::numeric_limits<double>::infinity())
			{
				std::ostringstream os;
				os << "外轴联立未找到可行解（最佳位置残差" << lastTeachResMm << "mm）";
				*failReason = os.str();
			}
			else
			{
				*failReason = "外轴联立未找到可行解（请检查行程/方向）";
			}
		}
		return {};
	}

	UrdfRobotLoader::UrdfPoseIkTarget poseTarget{};
	poseTarget.posMm[0] = target[0];
	poseTarget.posMm[1] = target[1];
	poseTarget.posMm[2] = target[2];
	poseTarget.hasOrientation = useOrientation;
	if (useOrientation)
	{
		poseTarget.quatXyzw[0] = targetQuat.x();
		poseTarget.quatXyzw[1] = targetQuat.y();
		poseTarget.quatXyzw[2] = targetQuat.z();
		poseTarget.quatXyzw[3] = targetQuat.w();
	}
	UrdfRobotLoader::UrdfIkSolverOptions opt{};
	opt.maxIterations = 120;
	opt.maxPosThenOriAttempts = 18;
	opt.preferNumericalOnly = preferNumericalOnly;
	opt.raceAnalyticalAndNumerical = raceAnalyticalAndNumerical && !preferNumericalOnly;
	opt.seedAttractionWeight = seedAttractionWeight;
	opt.seedAttractionQ = seedAttractionQ;
	{
		const auto& extOpt = cmd.extensionProperties();
		const auto itApprox = extOpt.find("context.allowApproximateOrientation");
		if (itApprox != extOpt.end() && (itApprox->second == "1" || itApprox->second == "true"))
		{
			opt.allowApproximateOrientation = true;
		}
	}
	using Gates = UrdfRobotLoader::IkAcceptanceGates;
	const double kAcceptPosMm = Gates::acceptPosMm(opt.allowApproximateOrientation);
	const double kAcceptRotDeg = Gates::acceptOrientDeg(opt.allowApproximateOrientation);
	const std::vector<std::string> jointNames = revoluteJointNamesFromInstructionContext(cmd);
	RobotInstruction::MotionAxisConfiguration axisCfg{};
	if (cmd.hasMotionAxisConfigurationProperty())
	{
		axisCfg = cmd.motionAxisConfiguration();
	}
	std::vector<std::vector<double>> seeds;
	if (expandSeedVariants)
	{
		const auto& extSeeds = cmd.extensionProperties();
		std::string urdfForSeeds;
		{
			const auto itU = extSeeds.find("context.urdfPath");
			if (itU != extSeeds.end())
			{
				urdfForSeeds = itU->second;
			}
		}
		int elbowOv = -1;
		int wristOv = -1;
		RobotInstruction::parseIkJointIndexOverrides(extSeeds, elbowOv, wristOv);
		bool spherical = false;
		if (!urdfForSeeds.empty())
		{
			const auto mach = UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(urdfForSeeds));
			if (elbowOv < 0 && mach.elbowJointIndex >= 0)
			{
				elbowOv = mach.elbowJointIndex;
			}
			if (wristOv < 0 && mach.wristJointIndex >= 0)
			{
				wristOv = mach.wristJointIndex;
			}
			if (mach.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::No)
			{
				spherical = false;
			}
			else if (mach.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::Yes)
			{
				spherical = true;
			}
			else
			{
				spherical = UrdfRobotLoader::SphericalWristAnalyticalIk::isEligible(
					QString::fromStdString(urdfForSeeds));
			}
		}
		seeds = buildIkSeedVariants(q, jointNames, &axisCfg, spherical, elbowOv, wristOv);
	}
	else
	{
		seeds.push_back(q);
	}
	const std::vector<double> qRef = q;
	struct AcceptCand
	{
		std::vector<double> q;
		double wrapDist = 0.0;
		double wristJump = 0.0;
		double cost = 0.0;
	};
	std::vector<AcceptCand> acceptCands;
	std::vector<double> bestQ;
	double bestPosMm = 1e30;
	double bestRotDeg = 1e30;
	double bestCost = 1e30;
	std::string lastFail;
	for (const std::vector<double>& seed : seeds)
	{
		std::string seedFail;
		UrdfRobotLoader::IkConvergenceStatus ikStatus = UrdfRobotLoader::IkConvergenceStatus::Failed;
		std::vector<double> qTry = UrdfRobotLoader::solveArmPoseDampedLeastSquares(
			urdfPath, ikLink, poseTarget, seed, opt, &seedFail, &ikStatus);
		if (qTry.empty() || ikStatus == UrdfRobotLoader::IkConvergenceStatus::Failed)
		{
			if (!seedFail.empty())
			{
				lastFail = std::move(seedFail);
			}
			continue;
		}
		foldJointsIntoUrdfLimits(urdfPath, qTry);
		double posMm = 0.0;
		double rotDeg = 0.0;
		if (!measureToolTargetResidual(cmd, qTry, posMm, rotDeg))
		{
			continue;
		}
		const double cost = posMm + 100.0 * rotDeg;
		if (cost < bestCost)
		{
			bestCost = cost;
			bestPosMm = posMm;
			bestRotDeg = rotDeg;
			bestQ = qTry;
		}
		if (posMm <= kAcceptPosMm && rotDeg <= kAcceptRotDeg)
		{
			AcceptCand ac;
			ac.q = std::move(qTry);
			ac.cost = cost;
			ac.wrapDist = UrdfRobotLoader::wrappedJointDistance(ac.q, qRef);
			ac.wristJump = RobotInstruction::wristConfigurationJumpRad(ac.q, qRef, jointNames);
			acceptCands.push_back(std::move(ac));
		}
	}
	if (!acceptCands.empty())
	{
		const AcceptCand* best = &acceptCands.front();
		for (const AcceptCand& c : acceptCands)
		{
			if (c.wrapDist < best->wrapDist - 1e-9)
			{
				best = &c;
				continue;
			}
			if (c.wrapDist > best->wrapDist + 1e-9)
			{
				continue;
			}
			const bool cCont = c.wristJump <= RobotInstruction::kWristContinuityGateRad;
			const bool bCont = best->wristJump <= RobotInstruction::kWristContinuityGateRad;
			if (cCont != bCont)
			{
				if (cCont)
				{
					best = &c;
				}
				continue;
			}
			if (c.wristJump < best->wristJump - 1e-9)
			{
				best = &c;
				continue;
			}
			if (c.cost < best->cost - 1e-9)
			{
				best = &c;
			}
		}
		return best->q;
	}
	if (!bestQ.empty() && bestPosMm <= kAcceptPosMm && bestRotDeg <= kAcceptRotDeg)
	{
		return bestQ;
	}
	if (failReason)
	{
		if (!bestQ.empty())
		{
			std::ostringstream os;
			os << "IK姿态残差超门限(pos=" << bestPosMm << "mm, orient=" << bestRotDeg << "deg, gate=" << kAcceptRotDeg
			   << "deg)";
			*failReason = os.str();
		}
		else
		{
			*failReason = lastFail.empty() ? std::string("目标不可达/IK未收敛") : lastFail;
		}
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
			*failReason = "缺少IK种子关节角";
		}
		return {};
	}
	return solveTargetByUrdfNumericalIkFromSeed(cmd, std::move(q0), failReason);
}

/// Yoshikawa：sqrt(det(J J^T))，J 为 6×n 行主序
double yoshikawaManipulability(const std::vector<double>& Jrow, const int rows, const int cols)
{
	if (rows <= 0 || cols <= 0 || static_cast<int>(Jrow.size()) < rows * cols)
	{
		return 0.0;
	}
	std::vector<double> JJT(static_cast<size_t>(rows * rows), 0.0);
	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < rows; ++c)
		{
			double s = 0.0;
			for (int k = 0; k < cols; ++k)
			{
				s += Jrow[static_cast<size_t>(r * cols + k)] * Jrow[static_cast<size_t>(c * cols + k)];
			}
			JJT[static_cast<size_t>(r * rows + c)] = s;
		}
	}
	// 高斯消元求 det（小维 6）
	std::vector<double> a = JJT;
	double det = 1.0;
	for (int i = 0; i < rows; ++i)
	{
		int piv = i;
		for (int r = i + 1; r < rows; ++r)
		{
			if (std::abs(a[static_cast<size_t>(r * rows + i)]) > std::abs(a[static_cast<size_t>(piv * rows + i)]))
			{
				piv = r;
			}
		}
		if (std::abs(a[static_cast<size_t>(piv * rows + i)]) < 1e-18)
		{
			return 0.0;
		}
		if (piv != i)
		{
			for (int c = 0; c < rows; ++c)
			{
				std::swap(a[static_cast<size_t>(i * rows + c)], a[static_cast<size_t>(piv * rows + c)]);
			}
			det = -det;
		}
		const double diag = a[static_cast<size_t>(i * rows + i)];
		det *= diag;
		for (int r = i + 1; r < rows; ++r)
		{
			const double f = a[static_cast<size_t>(r * rows + i)] / diag;
			for (int c = i; c < rows; ++c)
			{
				a[static_cast<size_t>(r * rows + c)] -= f * a[static_cast<size_t>(i * rows + c)];
			}
		}
	}
	if (det <= 0.0)
	{
		return 0.0;
	}
	return std::sqrt(det);
}

double estimateManipulabilityForCmd(const RobotInstruction::Base& cmd, const std::vector<double>& qRad)
{
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	const auto itTcp = ext.find("context.tcpLinkName");
	const auto itFlange = ext.find("context.flangeLinkName");
	if (itUrdf == ext.end() || itUrdf->second.empty() || qRad.empty())
	{
		return 0.0;
	}
	std::string link = (itFlange != ext.end() && !itFlange->second.empty())
						   ? itFlange->second
						   : ((itTcp != ext.end()) ? itTcp->second : std::string());
	if (link.empty())
	{
		return 0.0;
	}
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(qRad.size()));
	for (double v : qRad)
	{
		qQt.push_back(v);
	}
	double pos[3] = {0, 0, 0};
	double quat[4] = {0, 0, 0, 1};
	std::vector<double> J;
	if (!UrdfRobotLoader::computeLinkPoseAndJacobianViaCore(QString::fromStdString(itUrdf->second), qQt,
															QString::fromStdString(link), pos, quat, J, true, 1.0,
															nullptr))
	{
		return 0.0;
	}
	const int cols = static_cast<int>(qRad.size());
	const int rows = cols > 0 ? static_cast<int>(J.size()) / cols : 0;
	if (rows < 6)
	{
		return 0.0;
	}
	return yoshikawaManipulability(J, 6, cols);
}

/// 轴配置多解择优；classRefQ 缺省用 qSeed（路径样点可传起点作转数参照）
std::vector<double> solveIkWithAxisConfiguration(const RobotInstruction::Base& cmd,
												 const RobotInstruction::MotionAxisConfiguration& cfg,
												 const std::vector<double>& qSeed, std::string* failReason,
												 const RobotInstruction::IkSelectionPolicy& policy,
												 const std::vector<double>* classRefQ = nullptr)
{
	if (qSeed.empty())
	{
		if (failReason)
		{
			*failReason = "缺少IK种子关节角";
		}
		return {};
	}
	const std::vector<std::string> jointNames = revoluteJointNamesFromInstructionContext(cmd);
	const std::vector<double>& classRef = (classRefQ && classRefQ->size() == qSeed.size()) ? *classRefQ : qSeed;
	const auto& ext = cmd.extensionProperties();
	std::string urdfPath;
	{
		const auto itUrdf = ext.find("context.urdfPath");
		if (itUrdf != ext.end())
		{
			urdfPath = itUrdf->second;
		}
	}
	int elbowOverride = -1;
	int wristOverride = -1;
	RobotInstruction::parseIkJointIndexOverrides(ext, elbowOverride, wristOverride);
	UrdfRobotLoader::UrdfIkMachineProfile machProfile;
	if (!urdfPath.empty())
	{
		machProfile = UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(urdfPath));
	}
	// 扩展键优先于机型表
	if (elbowOverride < 0 && machProfile.elbowJointIndex >= 0)
	{
		elbowOverride = machProfile.elbowJointIndex;
	}
	if (wristOverride < 0 && machProfile.wristJointIndex >= 0)
	{
		wristOverride = machProfile.wristJointIndex;
	}
	bool sphericalWrist = false;
	if (!urdfPath.empty())
	{
		if (machProfile.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::No)
		{
			sphericalWrist = false;
		}
		else if (machProfile.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::Yes)
		{
			sphericalWrist = true;
		}
		else
		{
			sphericalWrist =
				UrdfRobotLoader::SphericalWristAnalyticalIk::isEligible(QString::fromStdString(urdfPath));
		}
	}

	RobotInstruction::IkSelectionPolicy effectivePolicy = policy;
	RobotInstruction::applyIkPolicyExtensionOverrides(ext, effectivePolicy);
	RobotInstruction::adaptIkPolicyForRobot(urdfPath, effectivePolicy, &ext);
	RobotInstruction::clearIkEscalateSummaryTag();

	const bool dragProfile = [&]()
	{
		const auto it = ext.find("context.ikProfile");
		return it != ext.end() && it->second == "drag";
	}();

	struct Candidate
	{
		std::vector<double> q;
		double dist = 0.0;
		double wristJump = 0.0;
		double manip = 0.0;
		double collisionCost = 0.0;
		int seedOrder = 0;
	};

	std::vector<Candidate> matching;
	std::vector<Candidate> converged;
	std::string lastSeedFail;
	bool budgetExhausted = false;
	bool usedNumRace = false;

	auto runSearchPass = [&](RobotInstruction::IkSelectionPolicy& pol)
	{
		matching.clear();
		converged.clear();
		lastSeedFail.clear();
		budgetExhausted = false;
		usedNumRace = false;
		int seedOrder = 0;

		std::vector<std::vector<double>> seeds;
		if (pol.mode == RobotInstruction::IkSolveMode::Search)
		{
			seeds = buildIkSeedVariants(qSeed, jointNames, &cfg, sphericalWrist, elbowOverride, wristOverride);
			std::sort(seeds.begin(), seeds.end(),
					  [&](const std::vector<double>& a, const std::vector<double>& b)
					  {
						  return UrdfRobotLoader::wrappedJointDistance(a, qSeed) <
								 UrdfRobotLoader::wrappedJointDistance(b, qSeed);
					  });
		}
		else
		{
			seeds.push_back(qSeed);
		}
		if (pol.maxSeeds > 0 && static_cast<int>(seeds.size()) > pol.maxSeeds)
		{
			seeds.resize(static_cast<size_t>(pol.maxSeeds));
		}

		QElapsedTimer budget;
		budget.start();
		const bool useBudget = pol.budgetMs > 0;

		auto trySeedPass = [&](const bool numericalOnly)
		{
			for (const std::vector<double>& seed : seeds)
			{
				if (useBudget && budget.elapsed() >= pol.budgetMs)
				{
					budgetExhausted = true;
					break;
				}
				std::string seedFail;
				const bool raceBoth = pol.raceAnalyticalAndNumerical && !numericalOnly;
				std::vector<double> qTry = solveTargetByUrdfNumericalIkFromSeed(
					cmd, seed, &seedFail, false, numericalOnly, raceBoth, pol.seedAttractionWeight);
				++seedOrder;
				if (qTry.empty())
				{
					if (!seedFail.empty())
					{
						lastSeedFail = std::move(seedFail);
					}
					continue;
				}
				Candidate c;
				c.dist = UrdfRobotLoader::wrappedJointDistance(qTry, qSeed);
				c.wristJump = RobotInstruction::wristConfigurationJumpRad(qTry, qSeed, jointNames);
				c.collisionCost = g_ikCollisionCostProbe ? g_ikCollisionCostProbe(qTry) : 0.0;
				if (c.collisionCost >= 1e6)
				{
					continue;
				}
				c.manip = (pol.rankMode == RobotInstruction::IkRankMode::Manip)
							  ? estimateManipulabilityForCmd(cmd, qTry)
							  : 0.0;
				c.seedOrder = seedOrder;
				c.q = std::move(qTry);
				converged.push_back(std::move(c));
				if (pol.rankMode == RobotInstruction::IkRankMode::Speed && !pol.requireWristContinuity &&
					!pol.requireMatchesClass)
				{
					break;
				}
			}
		};

		trySeedPass(false);
		if (converged.empty() && pol.raceNumericalFallback && !(useBudget && budget.elapsed() >= pol.budgetMs))
		{
			usedNumRace = true;
			trySeedPass(true);
		}
		else if (converged.empty() && useBudget && budget.elapsed() >= pol.budgetMs)
		{
			budgetExhausted = true;
		}
	};

	runSearchPass(effectivePolicy);
	// drag 预算耗尽：升 preview 再试一次（不改指令持久化 profile）
	if (converged.empty() && budgetExhausted && dragProfile)
	{
		effectivePolicy.budgetMs = 120;
		effectivePolicy.maxSeeds = 16;
		runSearchPass(effectivePolicy);
		if (!converged.empty())
		{
			RobotInstruction::setIkEscalateSummaryTag(" [ikEscalate=preview]");
		}
	}
	(void)usedNumRace;

	if (effectivePolicy.requireMatchesClass)
	{
		for (const Candidate& c : converged)
		{
			const RobotInstruction::JointConfigurationClass observed =
				RobotInstruction::classifyJointConfiguration(c.q, jointNames, &classRef, elbowOverride, wristOverride);
			if (cfg.matchesClass(observed))
			{
				matching.push_back(c);
			}
		}
	}
	const std::vector<Candidate>& poolBase = effectivePolicy.requireMatchesClass ? matching : converged;
	if (poolBase.empty())
	{
		if (failReason)
		{
			if (budgetExhausted && converged.empty())
			{
				// 超时优先标 Budget，避免拓扑文案冒充主因
				*failReason = "IK预算耗尽";
			}
			else if (effectivePolicy.requireMatchesClass && !converged.empty())
			{
				*failReason = "无满足轴配置的IK解（已收敛但肘/腕/臂/转数不匹配）";
			}
			else if (!lastSeedFail.empty())
			{
				*failReason = lastSeedFail;
			}
			else
			{
				*failReason = "IK未收敛";
			}
			const RobotInstruction::IkFailKind kind = RobotInstruction::classifyIkFailMessage(*failReason);
			*failReason += RobotInstruction::formatIkFailKindTag(kind);
		}
		return {};
	}
	std::vector<Candidate> continuousOnly;
	const std::vector<Candidate>* pool = &poolBase;
	if (effectivePolicy.requireWristContinuity)
	{
		for (const Candidate& c : poolBase)
		{
			if (c.wristJump <= RobotInstruction::kWristContinuityGateRad)
			{
				continuousOnly.push_back(c);
			}
		}
		if (continuousOnly.empty())
		{
			if (failReason)
			{
				*failReason = "轴配置解腕部跳变过大（相对种子超过连续性门限）";
				*failReason +=
					RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::Wrist);
			}
			return {};
		}
		pool = &continuousOnly;
	}
	// Distance 平局才补算 Manip，避免每候选雅可比
	if (effectivePolicy.rankMode == RobotInstruction::IkRankMode::Distance &&
		effectivePolicy.manipOnDistanceTie && pool->size() >= 2)
	{
		std::vector<Candidate>& mutablePool =
			effectivePolicy.requireWristContinuity
				? continuousOnly
				: (effectivePolicy.requireMatchesClass ? matching : converged);
		double bestDist = 1e30;
		for (const Candidate& c : mutablePool)
		{
			bestDist = std::min(bestDist, c.dist);
		}
		for (Candidate& c : mutablePool)
		{
			if (c.dist <= bestDist + 1e-3 && c.manip <= 0.0)
			{
				c.manip = estimateManipulabilityForCmd(cmd, c.q);
			}
		}
		pool = &mutablePool;
	}
	const RobotInstruction::IkRankMode rankMode = effectivePolicy.rankMode;
	const auto isCandidateBetter = [rankMode](const Candidate& c, const Candidate& best)
	{
		if (c.collisionCost + 1e-9 < best.collisionCost)
		{
			return true;
		}
		if (c.collisionCost > best.collisionCost + 1e-9)
		{
			return false;
		}
		const bool cCont = c.wristJump <= RobotInstruction::kWristContinuityGateRad;
		const bool bCont = best.wristJump <= RobotInstruction::kWristContinuityGateRad;
		if (cCont != bCont)
		{
			return cCont;
		}
		if (rankMode == RobotInstruction::IkRankMode::Speed)
		{
			return c.seedOrder < best.seedOrder;
		}
		if (rankMode == RobotInstruction::IkRankMode::Manip)
		{
			if (c.manip > best.manip + 1e-12)
			{
				return true;
			}
			if (c.manip < best.manip - 1e-12)
			{
				return false;
			}
		}
		if (c.wristJump < best.wristJump - 1e-9)
		{
			return true;
		}
		if (c.wristJump > best.wristJump + 1e-9)
		{
			return false;
		}
		if (c.dist < best.dist - 1e-9)
		{
			return true;
		}
		if (c.dist > best.dist + 1e-9)
		{
			return false;
		}
		if (c.manip > best.manip + 1e-12)
		{
			return true;
		}
		if (c.manip < best.manip - 1e-12)
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
	const Candidate* best = &pool->front();
	for (const Candidate& c : *pool)
	{
		if (isCandidateBetter(c, *best))
		{
			best = &c;
		}
	}
	return best->q;
}

std::vector<double> solveIkWithAxisConfiguration(const RobotInstruction::Base& cmd,
												 const RobotInstruction::MotionAxisConfiguration& cfg,
												 std::string* failReason,
												 const RobotInstruction::IkSelectionPolicy& policy)
{
	return solveIkWithAxisConfiguration(cmd, cfg, currentJointVectorFromInstruction(cmd), failReason, policy, nullptr);
}

std::vector<double> solveIkWithAxisConfiguration(const RobotInstruction::Base& cmd, std::string* failReason,
												 const RobotInstruction::IkSelectionPolicy& policy)
{
	RobotInstruction::MotionAxisConfiguration cfg;
	if (cmd.hasMotionAxisConfigurationProperty())
	{
		cfg = cmd.motionAxisConfiguration();
	}
	return solveIkWithAxisConfiguration(cmd, cfg, failReason, policy);
}

bool canSolveIkWithAxisConfiguration(const RobotInstruction::Base& cmd,
									 const RobotInstruction::MotionAxisConfiguration& cfg)
{
	// 可行性按笛卡尔路径策略，避免 LINE 误判可解
	return !solveIkWithAxisConfiguration(cmd, cfg, nullptr, RobotInstruction::makeCartesianPathIkPolicy(cfg)).empty();
}

/// LINE/ARC 样点：NearSeed→必要时 Search；腕相对上一点与路径起点双门
std::vector<double> solveCartesianPathSampleIk(const RobotInstruction::Base& cmd,
											   const RobotInstruction::MotionAxisConfiguration& cfg,
											   const std::vector<double>& pathStartQ,
											   const std::vector<double>& rollingSeedQ, std::string* failReason)
{
	int elbowOv = -1;
	int wristOv = -1;
	RobotInstruction::parseIkJointIndexOverrides(cmd.extensionProperties(), elbowOv, wristOv);
	{
		const auto itU = cmd.extensionProperties().find("context.urdfPath");
		if (itU != cmd.extensionProperties().end() && !itU->second.empty())
		{
			const auto mach = UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(itU->second));
			if (elbowOv < 0 && mach.elbowJointIndex >= 0)
			{
				elbowOv = mach.elbowJointIndex;
			}
			if (wristOv < 0 && mach.wristJointIndex >= 0)
			{
				wristOv = mach.wristJointIndex;
			}
		}
	}
	RobotInstruction::IkSelectionPolicy nearPol = RobotInstruction::makeCartesianPathIkPolicy(cfg);
	nearPol.mode = RobotInstruction::IkSolveMode::NearSeed;
	nearPol.maxSeeds = 1;
	nearPol.budgetMs = 40;
	nearPol.raceNumericalFallback = true;
	std::string err;
	std::vector<double> q =
		solveIkWithAxisConfiguration(cmd, cfg, rollingSeedQ, &err, nearPol, &pathStartQ);
	const std::vector<std::string> jointNames = revoluteJointNamesFromInstructionContext(cmd);
	auto acceptSample = [&](const std::vector<double>& cand) -> bool
	{
		if (cand.empty() || cand.size() != pathStartQ.size())
		{
			return false;
		}
		const double jumpPrev =
			RobotInstruction::wristConfigurationJumpRad(cand, rollingSeedQ, jointNames);
		const double jumpStart = RobotInstruction::wristConfigurationJumpRad(cand, pathStartQ, jointNames);
		if (jumpPrev > RobotInstruction::kWristContinuityGateRad ||
			jumpStart > RobotInstruction::kWristContinuityGateRad)
		{
			return false;
		}
		if (nearPol.requireMatchesClass)
		{
			const RobotInstruction::JointConfigurationClass observed =
				RobotInstruction::classifyJointConfiguration(cand, jointNames, &pathStartQ, elbowOv, wristOv);
			if (!cfg.matchesClass(observed))
			{
				return false;
			}
		}
		return true;
	};
	if (acceptSample(q))
	{
		return q;
	}
	RobotInstruction::IkSelectionPolicy searchPol = RobotInstruction::makeCartesianPathIkPolicy(cfg);
	searchPol.mode = RobotInstruction::IkSolveMode::Search;
	searchPol.maxSeeds = 16;
	searchPol.budgetMs = 60;
	std::string err2;
	q = solveIkWithAxisConfiguration(cmd, cfg, rollingSeedQ, &err2, searchPol, &pathStartQ);
	if (acceptSample(q))
	{
		return q;
	}
	if (failReason)
	{
		if (nearPol.requireMatchesClass && !q.empty())
		{
			const RobotInstruction::JointConfigurationClass observed =
				RobotInstruction::classifyJointConfiguration(q, jointNames, &pathStartQ, elbowOv, wristOv);
			if (!cfg.matchesClass(observed))
			{
				*failReason = "路径样点轴配置不匹配";
				*failReason +=
					RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::AxisClass);
				return {};
			}
		}
		*failReason = err2.empty() ? (err.empty() ? std::string("路径样点腕连续失败") : err) : err2;
		if (failReason->find("腕") == std::string::npos && failReason->find("轴配置") == std::string::npos &&
			failReason->find("[fail=") == std::string::npos)
		{
			*failReason = "路径样点腕连续失败（相对上一点或起点超过门限）";
		}
		if (failReason->find("[fail=") == std::string::npos)
		{
			const RobotInstruction::IkFailKind kind = RobotInstruction::classifyIkFailMessage(*failReason);
			*failReason += RobotInstruction::formatIkFailKindTag(kind);
		}
	}
	return {};
}

void applyConditionalShortestAngleFold(const std::vector<double>& q0, std::vector<double>& qTarget,
									   const std::vector<std::string>& jointNames, bool* outAxisSwitched)
{
	bool axisSwitched = false;
	if (!jointNames.empty() && jointNames.size() == q0.size() && jointNames.size() == qTarget.size())
	{
		axisSwitched = RobotInstruction::shouldSkipShortestAngleFold(q0, qTarget, jointNames);
	}
	if (!axisSwitched)
	{
		constexpr double kTwoPi = 6.283185307179586;
		for (size_t i = 0; i < qTarget.size(); ++i)
		{
			double d = qTarget[i] - q0[i];
			d -= kTwoPi * std::round(d / kTwoPi);
			qTarget[i] = q0[i] + d;
		}
	}
	if (outAxisSwitched)
	{
		*outAxisSwitched = axisSwitched;
	}
}

bool cartesianPathEndAccepted(const std::vector<double>& qEnd, const std::vector<double>& pathStartQ,
							  const RobotInstruction::MotionAxisConfiguration& cfg,
							  const std::vector<std::string>& jointNames, std::string* failReason,
							  const int elbowJointIndexOverride = -1, const int wristJointIndexOverride = -1)
{
	if (qEnd.empty() || qEnd.size() != pathStartQ.size())
	{
		if (failReason)
		{
			*failReason = "路径终点关节维数无效";
			*failReason += RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::Other);
		}
		return false;
	}
	const double jumpStart = RobotInstruction::wristConfigurationJumpRad(qEnd, pathStartQ, jointNames);
	if (jumpStart > RobotInstruction::kWristContinuityGateRad)
	{
		if (failReason)
		{
			*failReason = "路径终点腕连续失败（相对起点超过门限）";
			*failReason += RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::Wrist);
		}
		return false;
	}
	if (RobotInstruction::motionAxisConfigurationRequiresConstraint(cfg))
	{
		const RobotInstruction::JointConfigurationClass observed = RobotInstruction::classifyJointConfiguration(
			qEnd, jointNames, &pathStartQ, elbowJointIndexOverride, wristJointIndexOverride);
		if (!cfg.matchesClass(observed))
		{
			if (failReason)
			{
				*failReason = "路径终点轴配置不匹配";
				*failReason +=
					RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::AxisClass);
			}
			return false;
		}
	}
	return true;
}

struct IkPostureClassEntry
{
	RobotInstruction::JointConfigurationClass cls;
};

void appendUniquePostureClass(std::vector<IkPostureClassEntry>& out,
							  const RobotInstruction::JointConfigurationClass& cls)
{
	for (const IkPostureClassEntry& e : out)
	{
		if (RobotInstruction::sameJointConfigurationClass(e.cls, cls))
		{
			return;
		}
	}
	out.push_back({cls});
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
		std::vector<double> q = solveTargetByUrdfNumericalIkFromSeed(cmd, seed, nullptr, false);
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

bool anyPostureMatchesConfiguration(const std::vector<IkPostureClassEntry>& postures,
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
	explicit PtpPlanner(const std::vector<robot_kinematics::DhRow>* dhRows) : m_dhRows(dhRows) {}

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
				*errMsg = "缺少IK种子关节角";
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
		const bool constrainAxis = cmd.hasMotionAxisConfigurationProperty() &&
								   RobotInstruction::motionAxisConfigurationRequiresConstraint(axisCfg);
		if (constrainAxis && !preferUrdfIk)
		{
			if (errMsg)
			{
				*errMsg = "轴配置IK需要URDF/TCP上下文";
			}
			return false;
		}
		if (preferUrdfIk)
		{
			// AUTO 与显式约束统一走 PTP policy（腕软 + Search 预算）
			targetQ = solveIkWithAxisConfiguration(cmd, axisCfg, &ikFailReason,
												   RobotInstruction::makePtpIkPolicy(axisCfg));
			if (!targetQ.empty())
			{
				solvePath = constrainAxis ? "urdf_axis_cfg" : "urdf";
			}
		}
		// 有 URDF/TCP 时禁止 DH/legacy 冒充位姿解；仅无 URDF 且无姿态时允许 DH 位置回退
		if (targetQ.empty() && !preferUrdfIk && m_dhRows && !m_dhRows->empty() && !constrainAxis &&
			!cmd.hasEulerProperty() && !coupledExternalIkRequired())
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
		if (targetQ.empty() && !preferUrdfIk && !constrainAxis && !cmd.hasEulerProperty() &&
			!coupledExternalIkRequired())
		{
			targetQ = solveTargetByLegacyJointDelta(cmd);
			if (!targetQ.empty())
			{
				solvePath = "legacy";
			}
		}
		if (targetQ.empty())
		{
			if (errMsg)
			{
				*errMsg = ikFailReason.empty() ? "IK无解" : ikFailReason;
			}
			return false;
		}
		if (targetQ.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = "IK解关节数与种子不一致";
			}
			return false;
		}
		// 相对种子切支保留转数；同支最短折圈
		bool axisSwitched = false;
		applyConditionalShortestAngleFold(q0, targetQ, revoluteJointNamesFromInstructionContext(cmd), &axisSwitched);
		double maxJointDelta = 0.0;
		for (size_t i = 0; i < targetQ.size(); ++i)
		{
			maxJointDelta = std::max(maxJointDelta, std::abs(targetQ[i] - q0[i]));
		}
		// speed/accel 与历史梯形一致：按 °/s、°/s² 解读后转 rad
		const double vmax = std::max(1e-6, cmd.speed() * kDegToRad);
		const double amax = std::max(1e-6, cmd.accel() * kDegToRad);
		const double durationSec = trapezoidDuration(maxJointDelta, vmax, amax);

		out.ok = true;
		out.plannerName = "PtpPlanner";
		{
			RobotInstruction::IkSelectionPolicy ptpPol = RobotInstruction::makePtpIkPolicy(axisCfg);
			const auto& ptpExt = cmd.extensionProperties();
			RobotInstruction::applyIkPolicyExtensionOverrides(ptpExt, ptpPol);
			std::string urdfForPol;
			{
				const auto itU = ptpExt.find("context.urdfPath");
				if (itU != ptpExt.end())
				{
					urdfForPol = itU->second;
				}
			}
			RobotInstruction::adaptIkPolicyForRobot(urdfForPol, ptpPol, &ptpExt);
			const std::string escalateTag = RobotInstruction::consumeIkEscalateSummaryTag();
			out.summary = "PTP solved with joint target output.";
			out.summary += RobotInstruction::formatIkPolicySummaryTags(ptpPol, false, false, false);
			out.summary += escalateTag;
			if (solvePath == "dh" || solvePath == "legacy" || solvePath == "urdf_late")
			{
				out.summary += " [ikPath=" + solvePath + "]";
			}
			if (axisSwitched)
			{
				out.summary += " [axisSwitch]";
			}
			out.durationSec = durationSec;
			out.jointTargetsRad = targetQ;
			out.jointTrajectoryRad = {targetQ};
			{
				const RobotInstruction::RuckigPtpLimits lim =
					RobotInstruction::ruckigLimitsFromPtpDegScalars(q0.size(), cmd.speed(), cmd.accel());
				std::vector<std::vector<double>> ruckigTraj;
				double ruckigDur = 0.0;
				if (RobotInstruction::buildRuckigPtpJointTrajectory(q0, targetQ, lim, 0.02, ruckigTraj, ruckigDur) &&
					ruckigTraj.size() >= 2U)
				{
					out.durationSec = std::max(0.05, ruckigDur);
					out.jointTrajectoryRad = std::move(ruckigTraj);
					out.summary = "PTP solved with Ruckig joint trajectory.";
					out.summary += RobotInstruction::formatIkPolicySummaryTags(ptpPol, false, false, false);
					out.summary += escalateTag;
					if (solvePath == "dh" || solvePath == "legacy" || solvePath == "urdf_late")
					{
						out.summary += " [ikPath=" + solvePath + "]";
					}
					if (axisSwitched)
					{
						out.summary += " [axisSwitch]";
					}
				}
			}
		}
		applyLastIkExternalAxisToPlan(out);
		if (out.hasExternalAxisQ)
		{
			out.summary += " (with external axis)";
		}
		logIkSolveResidual(cmd, targetQ, "PtpPlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};

class LinePlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit LinePlanner(const std::vector<robot_kinematics::DhRow>* dhRows) : m_dhRows(dhRows) {}

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
		std::string solvePath = "none";
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		RobotInstruction::MotionAxisConfiguration axisCfg;
		if (cmd.hasMotionAxisConfigurationProperty())
		{
			axisCfg = cmd.motionAxisConfiguration();
		}
		const bool constrainAxis = cmd.hasMotionAxisConfigurationProperty() &&
								   RobotInstruction::motionAxisConfigurationRequiresConstraint(axisCfg);
		if (constrainAxis && !preferUrdfIk)
		{
			if (errMsg)
			{
				*errMsg = "轴配置IK需要URDF/TCP上下文";
			}
			return false;
		}
		if (preferUrdfIk)
		{
			// AUTO 亦走路径策略（腕硬门）；显式约束另加 matchesClass
			qTarget = solveIkWithAxisConfiguration(cmd, axisCfg, &ikFailReason,
												   RobotInstruction::makeCartesianPathIkPolicy(axisCfg));
			if (!qTarget.empty())
			{
				solvePath = constrainAxis ? "urdf_axis_cfg" : "urdf";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && m_dhRows && !m_dhRows->empty() && !constrainAxis &&
			!cmd.hasEulerProperty() && !coupledExternalIkRequired())
		{
			qTarget = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
			if (!qTarget.empty())
			{
				solvePath = "dh";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && !constrainAxis)
		{
			qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			if (!qTarget.empty())
			{
				solvePath = "urdf_late";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && !constrainAxis && !cmd.hasEulerProperty() &&
			!coupledExternalIkRequired())
		{
			qTarget = solveTargetByLegacyJointDelta(cmd);
			if (!qTarget.empty())
			{
				solvePath = "legacy";
			}
		}
		if (qTarget.empty() || qTarget.size() != q0.size())
		{
			if (errMsg)
			{
				if (qTarget.empty())
				{
					*errMsg = ikFailReason.empty() ? "IK无解" : ikFailReason;
				}
				else
				{
					*errMsg = "IK解关节数与种子不一致";
				}
			}
			return false;
		}
		applyConditionalShortestAngleFold(q0, qTarget, revoluteJointNamesFromInstructionContext(cmd), nullptr);

		double durationSec = 0.0;
		double cartDistMm = 0.0;
		double curTcp[3] = {0.0, 0.0, 0.0};
		const RobotInstruction::Vec3 p = cmd.pose();
		const bool haveCartStart = currentTcpPositionFromInstruction(cmd, curTcp);
		if (haveCartStart)
		{
			const double dx = p.x - curTcp[0];
			const double dy = p.y - curTcp[1];
			const double dz = p.z - curTcp[2];
			cartDistMm = std::sqrt(dx * dx + dy * dy + dz * dz);
			durationSec = trapezoidDuration(cartDistMm, cmd.speed(), cmd.accel());
		}
		else
		{
			double maxJointDelta = 0.0;
			for (size_t i = 0; i < qTarget.size(); ++i)
			{
				maxJointDelta = std::max(maxJointDelta, std::abs(qTarget[i] - q0[i]));
			}
			durationSec = trapezoidDuration(maxJointDelta, std::max(1e-6, cmd.speed() * kDegToRad),
											std::max(1e-6, cmd.accel() * kDegToRad));
		}

		out.jointTrajectoryRad.clear();
		bool usedCartesianSamples = false;
		bool cartDegraded = false;
		bool denseRetry = false;
		RobotInstruction::IkSelectionPolicy pathPol = RobotInstruction::makeCartesianPathIkPolicy(axisCfg);
		const auto& pathExt = cmd.extensionProperties();
		RobotInstruction::applyIkPolicyExtensionOverrides(pathExt, pathPol);
		{
			std::string urdfForPol;
			const auto itU = pathExt.find("context.urdfPath");
			if (itU != pathExt.end())
			{
				urdfForPol = itU->second;
			}
			RobotInstruction::adaptIkPolicyForRobot(urdfForPol, pathPol, &pathExt);
		}
		engine::RigidTransform T_end{};
		engine::RigidTransform T_start{};
		const bool canCartesian = preferUrdfIk && haveCartStart &&
								  RobotInstruction::readTargetTransformFromInstruction(cmd, T_end) &&
								  toolOriginTransformFromJoints(cmd, q0, T_start);
		if (canCartesian)
		{
			const int maxSamples = [&]()
			{
				const auto& ext = cmd.extensionProperties();
				const auto itLite = ext.find("context.playbackPlanLite");
				if (itLite != ext.end() && itLite->second == "1")
				{
					return 16;
				}
				return 64;
			}();
			const int baseSamples =
				std::max(8, std::min(maxSamples, static_cast<int>(std::ceil(std::max(cartDistMm, 1.0) / 8.0))));
			RobotInstruction::Base& mutableCmd = const_cast<RobotInstruction::Base&>(cmd);
			engine::RigidTransform T_backup{};
			const bool hadBackup = RobotInstruction::readTargetTransformFromInstruction(cmd, T_backup);
			const RobotInstruction::Vec3 poseBackup = cmd.pose();
			const RobotInstruction::Vec3 eulerBackup =
				cmd.hasEulerProperty() ? cmd.eulerDeg() : RobotInstruction::Vec3{};
			std::string quatCsvBackup;
			std::string transCsvBackup;
			{
				const auto& ext = cmd.extensionProperties();
				const auto itQ = ext.find(RobotInstruction::kExtContextTargetTransformQuatCsv);
				const auto itT = ext.find(RobotInstruction::kExtContextTargetTransformTransMmCsv);
				if (itQ != ext.end())
				{
					quatCsvBackup = itQ->second;
				}
				if (itT != ext.end())
				{
					transCsvBackup = itT->second;
				}
			}

			const std::vector<double> endpointQ = qTarget;
			const std::vector<std::string> sampleJointNames = revoluteJointNamesFromInstructionContext(cmd);
			int pathElbowOv = -1;
			int pathWristOv = -1;
			RobotInstruction::parseIkJointIndexOverrides(cmd.extensionProperties(), pathElbowOv, pathWristOv);
			{
				const auto itU = cmd.extensionProperties().find("context.urdfPath");
				if (itU != cmd.extensionProperties().end() && !itU->second.empty())
				{
					const auto mach =
						UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(itU->second));
					if (pathElbowOv < 0 && mach.elbowJointIndex >= 0)
					{
						pathElbowOv = mach.elbowJointIndex;
					}
					if (pathWristOv < 0 && mach.wristJointIndex >= 0)
					{
						pathWristOv = mach.wristJointIndex;
					}
				}
			}
			auto runSamplePass = [&](const int samples, std::string* failOut) -> bool
			{
				out.jointTrajectoryRad.clear();
				out.jointTrajectoryRad.reserve(static_cast<size_t>(samples));
				std::vector<double> seedQ = q0;
				for (int i = 1; i <= samples; ++i)
				{
					const double u = static_cast<double>(i) / static_cast<double>(samples);
					const Eigen::Vector3d t = T_start.translationMm() * (1.0 - u) + T_end.translationMm() * u;
					Eigen::Quaterniond q = T_start.rotation().normalized().slerp(u, T_end.rotation().normalized());
					if (q.coeffs().hasNaN())
					{
						q = T_end.rotation().normalized();
					}
					const engine::RigidTransform T_sample = engine::RigidTransform::fromTranslationQuat(t, q);
					RobotInstruction::writeTargetTransformToInstruction(mutableCmd, T_sample);
					std::string sampleFail;
					std::vector<double> qSample =
						solveCartesianPathSampleIk(cmd, axisCfg, q0, seedQ, &sampleFail);
					if (qSample.empty() || qSample.size() != q0.size())
					{
						if (failOut)
						{
							*failOut = sampleFail.empty()
										   ? ("LINE 路径中间点 IK 失败（样点 " + std::to_string(i) + ")")
										   : (sampleFail + " (样点 " + std::to_string(i) + ")");
						}
						out.jointTrajectoryRad.clear();
						return false;
					}
					seedQ = qSample;
					out.jointTrajectoryRad.push_back(std::move(qSample));
				}
				std::string endFail;
				if (!cartesianPathEndAccepted(out.jointTrajectoryRad.back(), q0, axisCfg, sampleJointNames, &endFail,
											 pathElbowOv, pathWristOv))
				{
					if (failOut)
					{
						*failOut = endFail;
					}
					out.jointTrajectoryRad.clear();
					return false;
				}
				return true;
			};

			std::string sampleFail;
			bool sampleOk = runSamplePass(baseSamples, &sampleFail);
			if (!sampleOk)
			{
				const int denseSamples = std::min(128, std::max(baseSamples * 2, baseSamples + 8));
				if (denseSamples > baseSamples)
				{
					denseRetry = true;
					sampleOk = runSamplePass(denseSamples, &sampleFail);
				}
			}

			if (hadBackup)
			{
				RobotInstruction::writeTargetTransformToInstruction(mutableCmd, T_backup);
			}
			else
			{
				mutableCmd.setPose(poseBackup);
				if (cmd.hasEulerProperty())
				{
					mutableCmd.setEulerDeg(eulerBackup);
				}
				if (quatCsvBackup.empty())
				{
					mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformQuatCsv, "");
				}
				else
				{
					mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformQuatCsv, quatCsvBackup);
				}
				if (transCsvBackup.empty())
				{
					mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformTransMmCsv, "");
				}
				else
				{
					mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformTransMmCsv,
													transCsvBackup);
				}
			}

			if (sampleOk && !out.jointTrajectoryRad.empty())
			{
				usedCartesianSamples = true;
				qTarget = out.jointTrajectoryRad.back();
			}
			else
			{
				out.jointTrajectoryRad.clear();
				qTarget = endpointQ;
				if (!pathPol.allowCartDegrade)
				{
					if (errMsg)
					{
						*errMsg = sampleFail.empty() ? "LINE 笛卡尔路径失败且禁止关节降级" : sampleFail;
					}
					return false;
				}
				cartDegraded = true;
			}
		}

		if (!usedCartesianSamples)
		{
			const int samples = 24;
			out.jointTrajectoryRad.reserve(static_cast<size_t>(samples));
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
		}

		out.ok = true;
		out.cartDegraded = cartDegraded;
		out.plannerName = "LinePlanner";
		out.summary = usedCartesianSamples ? "LINE cartesian samples with IK."
										   : "LINE joint-space trajectory (no URDF cartesian path).";
		out.summary += RobotInstruction::formatIkPolicySummaryTags(pathPol, denseRetry, false, cartDegraded);
		if (solvePath == "dh" || solvePath == "legacy" || solvePath == "urdf_late")
		{
			out.summary += " [ikPath=" + solvePath + "]";
		}
		out.durationSec = durationSec;
		out.jointTargetsRad = qTarget;
		applyLastIkExternalAxisToPlan(out);
		if (out.hasExternalAxisQ)
		{
			out.summary += " (with external axis)";
		}
		logIkSolveResidual(cmd, qTarget, "LinePlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};

class ArcPlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit ArcPlanner(const std::vector<robot_kinematics::DhRow>* dhRows) : m_dhRows(dhRows) {}

	bool canHandle(RobotInstruction::Type type) const override { return type == RobotInstruction::Type::ARC; }

	bool validate(const RobotInstruction::Base& cmd, std::string* errMsg) const override
	{
		if (!cmd.hasPoseProperty() || !cmd.hasViaPoseProperty())
		{
			if (errMsg)
				*errMsg = "ARC instruction requires via and end pose.";
			return false;
		}
		if (!cmd.hasSpeedProperty() || cmd.speed() <= 0.0)
		{
			if (errMsg)
				*errMsg = "ARC instruction requires speed > 0.";
			return false;
		}
		if (!cmd.hasAccelProperty() || cmd.accel() <= 0.0)
		{
			if (errMsg)
				*errMsg = "ARC instruction requires acceleration > 0.";
			return false;
		}
		if (cmd.hasBlendRadiusProperty() && cmd.blendRadius() < 0.0)
		{
			if (errMsg)
				*errMsg = "ARC instruction blend radius must be >= 0.";
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
				*errMsg = "ARC planning failed: missing context.currentJointRadCsv.";
			return false;
		}

		std::vector<double> qTarget;
		std::string ikFailReason;
		std::string solvePath = "none";
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		RobotInstruction::MotionAxisConfiguration axisCfg;
		if (cmd.hasMotionAxisConfigurationProperty())
		{
			axisCfg = cmd.motionAxisConfiguration();
		}
		const bool constrainAxis = cmd.hasMotionAxisConfigurationProperty() &&
								   RobotInstruction::motionAxisConfigurationRequiresConstraint(axisCfg);
		if (constrainAxis && !preferUrdfIk)
		{
			if (errMsg)
			{
				*errMsg = "轴配置IK需要URDF/TCP上下文";
			}
			return false;
		}
		if (preferUrdfIk)
		{
			qTarget = solveIkWithAxisConfiguration(cmd, axisCfg, &ikFailReason,
												   RobotInstruction::makeCartesianPathIkPolicy(axisCfg));
			if (!qTarget.empty())
			{
				solvePath = constrainAxis ? "urdf_axis_cfg" : "urdf";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && m_dhRows && !m_dhRows->empty() && !constrainAxis &&
			!cmd.hasEulerProperty() && !coupledExternalIkRequired())
		{
			qTarget = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
			if (!qTarget.empty())
			{
				solvePath = "dh";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && !constrainAxis)
		{
			qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
			if (!qTarget.empty())
			{
				solvePath = "urdf_late";
			}
		}
		if (qTarget.empty() && !preferUrdfIk && !constrainAxis && !cmd.hasEulerProperty() &&
			!coupledExternalIkRequired())
		{
			qTarget = solveTargetByLegacyJointDelta(cmd);
			if (!qTarget.empty())
			{
				solvePath = "legacy";
			}
		}
		if (qTarget.empty() || qTarget.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = qTarget.empty() ? (ikFailReason.empty() ? "IK无解" : ikFailReason) : "IK解关节数与种子不一致";
			}
			return false;
		}
		applyConditionalShortestAngleFold(q0, qTarget, revoluteJointNamesFromInstructionContext(cmd), nullptr);

		engine::RigidTransform T_end{};
		engine::RigidTransform T_start{};
		engine::RigidTransform T_via{};
		const bool haveEnd = RobotInstruction::readTargetTransformFromInstruction(cmd, T_end);
		const bool haveVia = RobotInstruction::readViaTransformFromInstruction(cmd, T_via);
		const bool haveStart = toolOriginTransformFromJoints(cmd, q0, T_start);
		const bool canCartesian = preferUrdfIk && haveStart && haveEnd && haveVia;

		double durationSec = 0.0;
		out.jointTrajectoryRad.clear();

		if (!canCartesian)
		{
			if (errMsg)
			{
				if (!preferUrdfIk)
					*errMsg = "ARC requires URDF TCP context for circular path.";
				else if (!haveStart)
					*errMsg = "ARC failed: cannot FK start pose from seed joints.";
				else if (!haveVia)
					*errMsg = "ARC failed: missing via pose/transform.";
				else
					*errMsg = "ARC failed: missing end pose/transform.";
			}
			return false;
		}

		{
			const double p0[3] = {T_start.translationMm().x(), T_start.translationMm().y(),
								  T_start.translationMm().z()};
			const double p1[3] = {T_via.translationMm().x(), T_via.translationMm().y(), T_via.translationMm().z()};
			const double p2[3] = {T_end.translationMm().x(), T_end.translationMm().y(), T_end.translationMm().z()};
			robot_kinematics::Circle3Fit fit{};
			if (!robot_kinematics::fitCircle3Points(p0, p1, p2, fit))
			{
				if (errMsg)
					*errMsg = "ARC failed: points are colinear or radius too small.";
				return false;
			}
			const double arcLen = robot_kinematics::arcLengthMm(fit);
			durationSec = trapezoidDuration(arcLen, cmd.speed(), cmd.accel());

			const int maxSamples = [&]()
			{
				const auto& ext = cmd.extensionProperties();
				const auto itLite = ext.find("context.playbackPlanLite");
				if (itLite != ext.end() && itLite->second == "1")
				{
					return 16;
				}
				return 64;
			}();
			RobotInstruction::Base& mutableCmd = const_cast<RobotInstruction::Base&>(cmd);
			engine::RigidTransform T_backup{};
			const bool hadBackup = RobotInstruction::readTargetTransformFromInstruction(cmd, T_backup);
			const RobotInstruction::Vec3 poseBackup = cmd.pose();
			const RobotInstruction::Vec3 eulerBackup =
				cmd.hasEulerProperty() ? cmd.eulerDeg() : RobotInstruction::Vec3{};
			std::string quatCsvBackup;
			std::string transCsvBackup;
			{
				const auto& ext = cmd.extensionProperties();
				const auto itQ = ext.find(RobotInstruction::kExtContextTargetTransformQuatCsv);
				const auto itT = ext.find(RobotInstruction::kExtContextTargetTransformTransMmCsv);
				if (itQ != ext.end())
					quatCsvBackup = itQ->second;
				if (itT != ext.end())
					transCsvBackup = itT->second;
			}
			const Eigen::Quaterniond qStart = T_start.rotation().normalized();
			const Eigen::Quaterniond qEnd = T_end.rotation().normalized();
			const std::vector<std::string> sampleJointNames = revoluteJointNamesFromInstructionContext(cmd);
			int arcElbowOv = -1;
			int arcWristOv = -1;
			RobotInstruction::parseIkJointIndexOverrides(cmd.extensionProperties(), arcElbowOv, arcWristOv);
			{
				const auto itU = cmd.extensionProperties().find("context.urdfPath");
				if (itU != cmd.extensionProperties().end() && !itU->second.empty())
				{
					const auto mach =
						UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(itU->second));
					if (arcElbowOv < 0 && mach.elbowJointIndex >= 0)
					{
						arcElbowOv = mach.elbowJointIndex;
					}
					if (arcWristOv < 0 && mach.wristJointIndex >= 0)
					{
						arcWristOv = mach.wristJointIndex;
					}
				}
			}

			auto runArcSamplePass = [&](const double chordMm, std::string* failOut) -> bool
			{
				std::vector<double> samplesXyz;
				std::vector<double> sampleU;
				if (!robot_kinematics::sampleArcByChord(fit, chordMm, 8, maxSamples, samplesXyz, &sampleU))
				{
					if (failOut)
					{
						*failOut = "ARC sampling failed.";
					}
					return false;
				}
				const int samples = static_cast<int>(samplesXyz.size() / 3);
				if (samples <= 0 || static_cast<int>(sampleU.size()) != samples)
				{
					if (failOut)
					{
						*failOut = "ARC sampling failed.";
					}
					return false;
				}
				out.jointTrajectoryRad.clear();
				out.jointTrajectoryRad.reserve(static_cast<size_t>(samples));
				std::vector<double> seedQ = q0;
				for (int i = 0; i < samples; ++i)
				{
					const double u = sampleU[static_cast<size_t>(i)];
					const Eigen::Vector3d t(samplesXyz[static_cast<size_t>(i) * 3u],
											samplesXyz[static_cast<size_t>(i) * 3u + 1u],
											samplesXyz[static_cast<size_t>(i) * 3u + 2u]);
					Eigen::Quaterniond q = qStart.slerp(u, qEnd);
					if (q.coeffs().hasNaN())
					{
						q = qEnd;
					}
					const engine::RigidTransform T_sample = engine::RigidTransform::fromTranslationQuat(t, q);
					RobotInstruction::writeTargetTransformToInstruction(mutableCmd, T_sample);
					std::string sampleFail;
					std::vector<double> qSample =
						solveCartesianPathSampleIk(cmd, axisCfg, q0, seedQ, &sampleFail);
					if (qSample.empty() || qSample.size() != q0.size())
					{
						if (failOut)
						{
							*failOut = sampleFail.empty()
										   ? ("ARC 路径中间点 IK 失败（样点 " + std::to_string(i) + ")")
										   : (sampleFail + " (样点 " + std::to_string(i) + ")");
						}
						out.jointTrajectoryRad.clear();
						return false;
					}
					seedQ = qSample;
					out.jointTrajectoryRad.push_back(std::move(qSample));
				}
				std::string endFail;
				if (!cartesianPathEndAccepted(out.jointTrajectoryRad.back(), q0, axisCfg, sampleJointNames, &endFail,
											 arcElbowOv, arcWristOv))
				{
					if (failOut)
					{
						*failOut = endFail;
					}
					out.jointTrajectoryRad.clear();
					return false;
				}
				return true;
			};

			std::string sampleFail;
			bool sampleOk = runArcSamplePass(8.0, &sampleFail);
			if (!sampleOk)
			{
				sampleOk = runArcSamplePass(4.0, &sampleFail);
			}

			if (hadBackup)
			{
				RobotInstruction::writeTargetTransformToInstruction(mutableCmd, T_backup);
			}
			else
			{
				mutableCmd.setPose(poseBackup);
				if (cmd.hasEulerProperty())
				{
					mutableCmd.setEulerDeg(eulerBackup);
				}
				mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformQuatCsv, quatCsvBackup);
				mutableCmd.setExtensionProperty(RobotInstruction::kExtContextTargetTransformTransMmCsv, transCsvBackup);
			}

			if (sampleOk && !out.jointTrajectoryRad.empty())
			{
				qTarget = out.jointTrajectoryRad.back();
			}
			else
			{
				out.jointTrajectoryRad.clear();
				if (errMsg)
					*errMsg = sampleFail.empty() ? "ARC cartesian sample IK failed" : sampleFail;
				return false;
			}
		}

		out.ok = true;
		out.plannerName = "ArcPlanner";
		out.summary = "ARC cartesian samples with IK.";
		if (solvePath == "dh" || solvePath == "legacy" || solvePath == "urdf_late")
		{
			out.summary += " [ikPath=" + solvePath + "]";
		}
		out.durationSec = durationSec;
		out.jointTargetsRad = qTarget;
		applyLastIkExternalAxisToPlan(out);
		if (out.hasExternalAxisQ)
		{
			out.summary += " (with external axis)";
		}
		logIkSolveResidual(cmd, qTarget, "ArcPlanner");
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

void Controller::setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes)
{
	m_externalAxes = axes;
}

void Controller::clearExternalAxes()
{
	m_externalAxes = {};
}

bool Controller::hasEnabledExternalAxes() const
{
	return RobotExternal::hasEnabledExternalAxes(m_externalAxes);
}

void Controller::setWorkpieceIkFrameContext(const WorkpieceIkFrameContext& ctx)
{
	m_workpieceIkFrame = ctx;
}

void Controller::clearWorkpieceIkFrameContext()
{
	m_workpieceIkFrame = {};
}

void Controller::setIkCollisionCostProbe(IkCollisionCostFn fn)
{
	g_ikCollisionCostProbe = std::move(fn);
}

void Controller::clearIkCollisionCostProbe()
{
	g_ikCollisionCostProbe = {};
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
	registerPlanner(std::make_shared<ArcPlanner>(&m_dhRows));
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
	const ActiveExternalAxesGuard extGuard(hasEnabledExternalAxes() ? &m_externalAxes : nullptr);
	const ActiveWorkpieceIkFrameGuard wpGuard(m_workpieceIkFrame.valid ? &m_workpieceIkFrame : nullptr);
	clearLastIkExternalAxis();
	return planner->plan(cmd, out, errMsg);
}

const PlannerBase* Controller::findPlanner(Type t) const
{
	const auto it = std::find_if(m_planners.begin(), m_planners.end(),
								 [t](const std::shared_ptr<PlannerBase>& p) { return p && p->canHandle(t); });
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
	const ActiveExternalAxesGuard extGuard(hasEnabledExternalAxes() ? &m_externalAxes : nullptr);
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

	const auto hasNonAuto = [](const std::vector<std::string>& tokens)
	{
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
	auto collectTurnTokens = [&](int JointConfigurationClass::*turnMember, std::vector<std::string>& tokens)
	{
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
