#include "RobotSimulationMath.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionProgram.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotTeachIk.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>
#include <QFile>
#include <QRegularExpression>
#include <QSet>
#include <QXmlStreamReader>
#include <osg/Matrixd>
#include <osg/Quat>
#include <algorithm>
#include <cmath>

namespace RobotSimulationMath {
namespace
{

bool matrixFromNodeWorldImpl(osg::Node* node, osg::Matrixd& outWorld)
{
	if (!node)
	{
		return false;
	}
	osg::NodePathList paths = node->getParentalNodePaths();
	if (paths.empty())
	{
		return false;
	}
	const osg::NodePath& path = paths.front();
	outWorld = osg::computeLocalToWorld(path);
	if (path.empty() || path.back() != node)
	{
		if (const auto* mt = dynamic_cast<osg::MatrixTransform*>(node))
		{
			outWorld = outWorld * mt->getMatrix();
		}
	}
	return true;
}

struct ParsedUrdfJoint
{
	QString name;
	QString type;
	QString parent;
	QString child;
	double x = 0.0; // meters in URDF
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0; // radians
	double pitch = 0.0;
	double yaw = 0.0;
	double ax = 0.0;
	double ay = 0.0;
	double az = 1.0;
};

bool parseThreeDoubles(const QString& src, double& a, double& b, double& c)
{
	const QString text = src.trimmed();
	if (text.isEmpty())
	{
		a = b = c = 0.0;
		return true;
	}
	const QStringList parts = text.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
	if (parts.size() < 3)
	{
		return false;
	}
	bool ok = false;
	a = parts[0].toDouble(&ok);
	if (!ok) return false;
	b = parts[1].toDouble(&ok);
	if (!ok) return false;
	c = parts[2].toDouble(&ok);
	return ok;
}

bool loadUrdfJointList(const QString& urdfPath, QVector<ParsedUrdfJoint>& outJoints, QString* errMsg)
{
	outJoints.clear();
	QFile file(urdfPath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("无法打开URDF文件：%1").arg(urdfPath);
		}
		return false;
	}
	QXmlStreamReader xml(&file);
	while (!xml.atEnd())
	{
		xml.readNext();
		if (!xml.isStartElement() || xml.name() != QLatin1String("joint"))
		{
			continue;
		}
		ParsedUrdfJoint joint;
		joint.name = xml.attributes().value(QStringLiteral("name")).toString().trimmed();
		joint.type = xml.attributes().value(QStringLiteral("type")).toString().trimmed().toLower();
		while (xml.readNextStartElement())
		{
			if (xml.name() == QLatin1String("origin"))
			{
				(void)parseThreeDoubles(xml.attributes().value(QStringLiteral("xyz")).toString(), joint.x, joint.y, joint.z);
				(void)parseThreeDoubles(
					xml.attributes().value(QStringLiteral("rpy")).toString(), joint.roll, joint.pitch, joint.yaw);
				xml.skipCurrentElement();
			}
			else if (xml.name() == QLatin1String("parent"))
			{
				joint.parent = xml.attributes().value(QStringLiteral("link")).toString().trimmed();
				xml.skipCurrentElement();
			}
			else if (xml.name() == QLatin1String("child"))
			{
				joint.child = xml.attributes().value(QStringLiteral("link")).toString().trimmed();
				xml.skipCurrentElement();
			}
			else if (xml.name() == QLatin1String("axis"))
			{
				(void)parseThreeDoubles(
					xml.attributes().value(QStringLiteral("xyz")).toString(), joint.ax, joint.ay, joint.az);
				xml.skipCurrentElement();
			}
			else
			{
				xml.skipCurrentElement();
			}
		}
		if (!joint.parent.isEmpty() && !joint.child.isEmpty())
		{
			outJoints.push_back(joint);
		}
	}
	if (xml.hasError())
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("解析URDF失败：%1").arg(xml.errorString());
		}
		return false;
	}
	if (outJoints.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("URDF中没有可用的joint定义。");
		}
		return false;
	}
	return true;
}

bool decomposeDhFromOriginXyzRpy(
	double txMm,
	double tyMm,
	double tzMm,
	double roll,
	double pitch,
	double yaw,
	robot_kinematics::DhRow& out,
	QString* errMsg)
{
	const double cr = std::cos(roll);
	const double sr = std::sin(roll);
	const double cp = std::cos(pitch);
	const double sp = std::sin(pitch);
	const double cy = std::cos(yaw);
	const double sy = std::sin(yaw);

	const double r00 = cy * cp;
	const double r01 = cy * sp * sr - sy * cr;
	const double r02 = cy * sp * cr + sy * sr;
	const double r10 = sy * cp;
	const double r11 = sy * sp * sr + cy * cr;
	const double r12 = sy * sp * cr - cy * sr;
	const double r20 = -sp;
	const double r21 = cp * sr;
	const double r22 = cp * cr;

	if (std::abs(r20) > 1e-3)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("关节origin不满足DH可分解条件（pitch过大，r20=%1）。").arg(r20, 0, 'g', 6);
		}
		return false;
	}

	const double theta = std::atan2(r10, r00);
	const double alpha = std::atan2(r21, r22);
	const double ct = std::cos(theta);
	const double st = std::sin(theta);

	double a = 0.0;
	if (std::abs(ct) >= std::abs(st) && std::abs(ct) > 1e-6)
	{
		a = txMm / ct;
	}
	else if (std::abs(st) > 1e-6)
	{
		a = tyMm / st;
	}
	else
	{
		a = std::sqrt(txMm * txMm + tyMm * tyMm);
	}
	const double d = tzMm;

	const double txFit = a * ct;
	const double tyFit = a * st;
	if (std::abs(txMm - txFit) > 1e-2 || std::abs(tyMm - tyFit) > 1e-2)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("joint origin平移项与DH模型不一致（可能不是标准串联DH链）。");
		}
		return false;
	}

	out.a = a;
	out.alpha = alpha;
	out.d = d;
	out.thetaOffset = theta;
	return true;
}

bool buildDhRowsFromUrdfImpl(
	const QString& urdfPath,
	std::vector<robot_kinematics::DhRow>& outRows,
	QString* errMsg)
{
	outRows.clear();
	QVector<ParsedUrdfJoint> joints;
	if (!loadUrdfJointList(urdfPath, joints, errMsg))
	{
		return false;
	}

	QHash<QString, QVector<int>> parentToJointIdx;
	QSet<QString> allLinks;
	QSet<QString> childLinks;
	for (int i = 0; i < joints.size(); ++i)
	{
		const ParsedUrdfJoint& j = joints[i];
		parentToJointIdx[j.parent].push_back(i);
		allLinks.insert(j.parent);
		allLinks.insert(j.child);
		childLinks.insert(j.child);
	}
	QSet<QString> rootCandidates = allLinks;
	for (const QString& c : childLinks)
	{
		rootCandidates.remove(c);
	}
	if (rootCandidates.size() != 1)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("URDF不是单根串联结构（root候选=%1）。").arg(rootCandidates.size());
		}
		return false;
	}

	QString curLink = *rootCandidates.constBegin();
	QSet<QString> visitedLinks;
	QVector<ParsedUrdfJoint> serialChain;
	while (true)
	{
		if (visitedLinks.contains(curLink))
		{
			if (errMsg)
			{
				*errMsg = QStringLiteral("URDF关节链存在环路。");
			}
			return false;
		}
		visitedLinks.insert(curLink);
		const QVector<int> children = parentToJointIdx.value(curLink);
		if (children.isEmpty())
		{
			break;
		}
		if (children.size() != 1)
		{
			if (errMsg)
			{
				*errMsg = QStringLiteral("URDF存在分支，当前版本仅支持单链机器人。");
			}
			return false;
		}
		const ParsedUrdfJoint& j = joints[children.front()];
		serialChain.push_back(j);
		curLink = j.child;
	}
	if (serialChain.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("没有可用于构建DH的关节链。");
		}
		return false;
	}

	int revoluteIndex = 0;
	for (const ParsedUrdfJoint& j : serialChain)
	{
		robot_kinematics::DhRow row{};
		QString rowErr;
		if (!decomposeDhFromOriginXyzRpy(
				j.x * 1000.0, j.y * 1000.0, j.z * 1000.0, j.roll, j.pitch, j.yaw, row, &rowErr))
		{
			if (errMsg)
			{
				*errMsg = QStringLiteral("joint '%1' 无法转换为DH：%2").arg(j.name, rowErr);
			}
			return false;
		}
		if (j.type == QLatin1String("revolute") || j.type == QLatin1String("continuous"))
		{
			const double norm = std::sqrt(j.ax * j.ax + j.ay * j.ay + j.az * j.az);
			const double nx = (norm > 1e-9) ? (j.ax / norm) : 0.0;
			const double ny = (norm > 1e-9) ? (j.ay / norm) : 0.0;
			const double nz = (norm > 1e-9) ? (j.az / norm) : 1.0;
			if (norm <= 1e-9 || std::abs(nx) > 1e-3 || std::abs(ny) > 1e-3 || nz < 0.999)
			{
				if (errMsg)
				{
					*errMsg = QStringLiteral(
						"joint '%1' 旋转轴不是 +Z（axis=%2,%3,%4），当前DH求解链不支持。")
								  .arg(j.name)
								  .arg(nx, 0, 'g', 6)
								  .arg(ny, 0, 'g', 6)
								  .arg(nz, 0, 'g', 6);
				}
				return false;
			}
			row.jointIndex = revoluteIndex++;
		}
		else if (j.type == QLatin1String("fixed"))
		{
			row.jointIndex = -1;
		}
		else
		{
			if (errMsg)
			{
				*errMsg = QStringLiteral("joint '%1' 类型 '%2' 暂不支持DH自动建链。").arg(j.name, j.type);
			}
			return false;
		}
		outRows.push_back(row);
	}
	if (revoluteIndex <= 0)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("未找到可运动关节，无法进行IK。");
		}
		return false;
	}
	return true;
}

double matrixTranslationErrorMm(const osg::Matrixd& a, const osg::Matrixd& b)
{
	const osg::Vec3d ta = a.getTrans();
	const osg::Vec3d tb = b.getTrans();
	const osg::Vec3d d = ta - tb;
	return std::sqrt(d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
}

double quaternionAngularErrorDeg(const osg::Quat& qaIn, const osg::Quat& qbIn)
{
	osg::Quat qa = qaIn;
	osg::Quat qb = qbIn;
	auto normalizeQuat = [](osg::Quat& q) {
		const double n = std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
		if (n <= 1e-12)
		{
			q.set(0.0, 0.0, 0.0, 1.0);
			return;
		}
		const double inv = 1.0 / n;
		q.set(q.x() * inv, q.y() * inv, q.z() * inv, q.w() * inv);
	};
	normalizeQuat(qa);
	normalizeQuat(qb);
	double dot = qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z() + qa.w() * qb.w();
	dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
	const double angleRad = 2.0 * std::acos(dot);
	return angleRad * 180.0 / 3.14159265358979323846;
}

QString matrix4ToLog(const osg::Matrixd& m)
{
	QString out;
	for (int r = 0; r < 4; ++r)
	{
		if (r > 0)
		{
			out += QStringLiteral(" | ");
		}
		out += QStringLiteral("[%1,%2,%3,%4]")
				   .arg(m(r, 0), 0, 'f', 3)
				   .arg(m(r, 1), 0, 'f', 3)
				   .arg(m(r, 2), 0, 'f', 3)
				   .arg(m(r, 3), 0, 'f', 3);
	}
	return out;
}

std::string encodeMatrix4CsvImpl(const osg::Matrixd& m)
{
	osg::Matrixd o;
	// This matches the legacy matrix convention used by instruction render metadata.
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (r == 3 || c == 3)
			{
				o(r, c) = m(r, c);
			}
			else
			{
				o(r, c) = m(c, r);
			}
		}
	}

	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (r != 0 || c != 0)
			{
				oss << ",";
			}
			oss << o(r, c);
		}
	}
	return oss.str();
}

bool decodeMatrix4CsvImpl(const std::string& text, osg::Matrixd& out)
{
	std::stringstream ss(text);
	ss.imbue(std::locale::classic());
	std::string token;
	double values[16]{};
	int n = 0;
	while (std::getline(ss, token, ','))
	{
		if (n >= 16)
		{
			return false;
		}
		try
		{
			values[n++] = std::stod(token);
		}
		catch (...)
		{
			return false;
		}
	}
	if (n != 16)
	{
		return false;
	}
	osg::Matrixd in;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			in(r, c) = values[r * 4 + c];
		}
	}
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (r == 3 || c == 3)
			{
				out(r, c) = in(r, c);
			}
			else
			{
				out(r, c) = in(c, r);
			}
		}
	}
	return true;
}

} // anonymous namespace

ROBOTWIDGET_EXPORT bool matrixFromNodeWorld(osg::Node* node, osg::Matrixd& outWorld)
{
	return matrixFromNodeWorldImpl(node, outWorld);
}

ROBOTWIDGET_EXPORT std::string encodeMatrix4Csv(const osg::Matrixd& m)
{
	return encodeMatrix4CsvImpl(m);
}

ROBOTWIDGET_EXPORT bool decodeMatrix4Csv(const std::string& text, osg::Matrixd& out)
{
	return decodeMatrix4CsvImpl(text, out);
}

ROBOTWIDGET_EXPORT bool buildDhRowsFromUrdf(
	const QString& urdfPath,
	std::vector<robot_kinematics::DhRow>& outRows,
	QString* errMsg)
{
	return buildDhRowsFromUrdfImpl(urdfPath, outRows, errMsg);
}

} // namespace RobotSimulationMath
