#include "MainWindow.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <locale>
#include <unordered_set>

#include <QAbstractItemView>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QFile>
#include <QHeaderView>
#include <QList>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QRegularExpression>
#include <QSet>
#include <QSignalBlocker>
#include <QStringList>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTabWidget>
#include <QVector>
#include <QVBoxLayout>
#include <QWidget>
#include <QXmlStreamReader>

#include <osg/Vec3f>

#include "ApplicationStyle.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendHierarchyModel.h"
#include "BackendFollowMath.h"
#include "BackendFollowTransformSolver.h"
#include "DocumentPage.h"
#include "FollowAttachmentComponent.h"
#include "DevicePageWidget.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "IRobotBackendPoseSink.h"
#include "RobotInstructionProgram.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include "RunInfoPage.h"
#include "SimulationCommandWidget.h"

#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/Quat>

#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;
using namespace RobotSimulation;

namespace
{
bool matrixFromNodeWorld(osg::Node* node, osg::Matrixd& outWorld)
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

bool buildDhRowsFromUrdf(
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
	return angleRad * 180.0 / RobotSimulation::kPi;
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

std::string encodeMatrix4Csv(const osg::Matrixd& m)
{
	osg::Matrixd o;
	// Keep translation terms in-place, but map 3x3 rotation with swapped indices.
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

bool decodeMatrix4Csv(const std::string& text, osg::Matrixd& out)
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
	// Inverse of encodeMatrix4Csv mapping (same operation: involution).
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

struct MotionPoseBackup
{
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	bool active = false;
};

MotionPoseBackup backupInstructionPose(const RobotInstruction::Base& ins)
{
	MotionPoseBackup backup;
	if (ins.hasPoseProperty() && ins.hasEulerProperty())
	{
		backup.pose = ins.pose();
		backup.euler = ins.eulerDeg();
		backup.active = true;
	}
	return backup;
}

void restoreInstructionPose(RobotInstruction::Base& ins, const MotionPoseBackup& backup)
{
	if (!backup.active)
	{
		return;
	}
	ins.setPose(backup.pose);
	ins.setEulerDeg(backup.euler);
}

void mapInstructionPoseRenderToFk(RobotInstruction::Base& ins)
{
	if (!ins.hasPoseProperty() || !ins.hasEulerProperty())
	{
		return;
	}
	const RobotInstruction::Vec3 pose = ins.pose();
	const RobotInstruction::Vec3 euler = ins.eulerDeg();
	const auto& extMap = ins.extensionProperties();
	const auto itRenderToFk = extMap.find("context.renderWorldToFkMat4");
	bool mappedByMatrix = false;
	if (itRenderToFk != extMap.end() && !itRenderToFk->second.empty())
	{
		osg::Matrixd renderToFk;
		if (decodeMatrix4Csv(itRenderToFk->second, renderToFk))
		{
			bool shouldMapRenderToFk = true;
			osg::Matrixd capRender;
			osg::Matrixd capFk;
			const auto itCapturedRender = extMap.find("render.tcpWorldMat4");
			const auto itCapturedFk = extMap.find("render.tcpLocalMat4");
			if (itCapturedRender != extMap.end() && !itCapturedRender->second.empty()
				&& itCapturedFk != extMap.end() && !itCapturedFk->second.empty())
			{
				if (decodeMatrix4Csv(itCapturedRender->second, capRender)
					&& decodeMatrix4Csv(itCapturedFk->second, capFk))
				{
					osg::Matrixd currentPoseMat;
					currentPoseMat.makeRotate(OsgScene::eulerDegToQuat(osg::Vec3f(
						static_cast<float>(euler.x),
						static_cast<float>(euler.y),
						static_cast<float>(euler.z))));
					currentPoseMat.setTrans(pose.x, pose.y, pose.z);
					const double fkScore = matrixTranslationErrorMm(currentPoseMat, capFk)
						+ quaternionAngularErrorDeg(currentPoseMat.getRotate(), capFk.getRotate());
					const double renderScore = matrixTranslationErrorMm(currentPoseMat, capRender)
						+ quaternionAngularErrorDeg(currentPoseMat.getRotate(), capRender.getRotate());
					shouldMapRenderToFk = (renderScore + 1e-6 < fkScore);
				}
			}
			osg::Matrixd targetRender;
			targetRender.makeRotate(OsgScene::eulerDegToQuat(osg::Vec3f(
				static_cast<float>(euler.x),
				static_cast<float>(euler.y),
				static_cast<float>(euler.z))));
			targetRender.setTrans(pose.x, pose.y, pose.z);
			const osg::Matrixd targetFk = shouldMapRenderToFk ? (renderToFk * targetRender) : targetRender;
			const osg::Vec3d mappedPos = targetFk.getTrans();
			const osg::Vec3f mappedEuler = OsgScene::quatToEulerDeg(targetFk.getRotate());
			ins.setPose(RobotInstruction::Vec3{ mappedPos.x(), mappedPos.y(), mappedPos.z() });
			ins.setEulerDeg(RobotInstruction::Vec3{
				static_cast<double>(mappedEuler.x()),
				static_cast<double>(mappedEuler.y()),
				static_cast<double>(mappedEuler.z()) });
			mappedByMatrix = true;
		}
	}
	if (!mappedByMatrix)
	{
		const auto itDeltaPos = extMap.find("context.renderToFkDeltaPosMmCsv");
		if (itDeltaPos != extMap.end() && !itDeltaPos->second.empty())
		{
			std::stringstream dss(itDeltaPos->second);
			dss.imbue(std::locale::classic());
			std::string tok;
			double dv[3] = { 0.0, 0.0, 0.0 };
			int dn = 0;
			while (std::getline(dss, tok, ',') && dn < 3)
			{
				try
				{
					dv[dn++] = std::stod(tok);
				}
				catch (...)
				{
					dn = 0;
					break;
				}
			}
			if (dn == 3)
			{
				ins.setPose(RobotInstruction::Vec3{ pose.x + dv[0], pose.y + dv[1], pose.z + dv[2] });
			}
		}
	}
}

std::string encodeJointCsv(const QVector<double>& q)
{
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	for (int i = 0; i < q.size(); ++i)
	{
		if (i > 0)
		{
			oss << ",";
		}
		oss << q[i];
	}
	return oss.str();
}

void attachMotionPlanningContext(
	RobotInstruction::Base& ins,
	const QVector<double>& rollingQ,
	const QString& urdfPath,
	const std::string& defaultTcpLinkName)
{
	ins.setExtensionProperty("context.currentJointRadCsv", encodeJointCsv(rollingQ));
	ins.setExtensionProperty("context.urdfPath", urdfPath.toStdString());
	std::string tcpRef = defaultTcpLinkName;
	const auto& ext = ins.extensionProperties();
	const auto itCapturedTcp = ext.find("context.capturedTcpLinkName");
	if (itCapturedTcp != ext.end() && !itCapturedTcp->second.empty())
	{
		tcpRef = itCapturedTcp->second;
	}
	ins.setExtensionProperty("context.tcpLinkName", tcpRef);
}

QString defaultTcpLinkNameForUrdf(const QString& urdfPath, const QString& comboTcpLink)
{
	QString defaultTcpLinkName;
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	if (!childLinks.isEmpty())
	{
		defaultTcpLinkName = childLinks.back();
	}
	if (defaultTcpLinkName.isEmpty() && !comboTcpLink.isEmpty())
	{
		defaultTcpLinkName = comboTcpLink;
	}
	if (defaultTcpLinkName.isEmpty())
	{
		(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, defaultTcpLinkName, nullptr);
	}
	return defaultTcpLinkName;
}
} // namespace

QString MainWindow::i18n(const QString& en, const QString& zh) const
{
	return m_useChinese ? zh : en;
}

void MainWindow::applyLanguage()
{
	setWindowTitle(i18n(QStringLiteral("PointCloudProcess - MainWindow"), QStringLiteral("点云处理 - 主窗口")));
	if (m_fileMenu) m_fileMenu->setTitle(i18n(QStringLiteral("File"), QStringLiteral("文件")));
	if (m_newDocumentAction)
	{
		m_newDocumentAction->setText(i18n(QStringLiteral("New"), QStringLiteral("新建")));
	}
	if (m_openModelAction) m_openModelAction->setText(i18n(QStringLiteral("Open Model..."), QStringLiteral("打开模型...")));
	if (m_openPointCloudAction) m_openPointCloudAction->setText(i18n(QStringLiteral("Open Point Cloud..."), QStringLiteral("打开点云...")));
	if (m_openProjectAction) m_openProjectAction->setText(i18n(QStringLiteral("Open Project..."), QStringLiteral("打开工程...")));
	if (m_saveAction) m_saveAction->setText(i18n(QStringLiteral("Save Project..."), QStringLiteral("保存工程...")));
	if (m_exitAction) m_exitAction->setText(i18n(QStringLiteral("Exit"), QStringLiteral("退出")));
	if (m_viewMenu) m_viewMenu->setTitle(i18n(QStringLiteral("View"), QStringLiteral("视图")));
	if (m_resetLayoutAction)
	{
		m_resetLayoutAction->setText(i18n(QStringLiteral("Reset Layout"), QStringLiteral("重置布局")));
	}
	if (m_settingsMenu) m_settingsMenu->setTitle(i18n(QStringLiteral("Settings"), QStringLiteral("设置")));
	if (m_appearanceMenu)
	{
		m_appearanceMenu->setTitle(i18n(QStringLiteral("Theme"), QStringLiteral("风格")));
	}
	if (m_lightThemeAction)
	{
		m_lightThemeAction->setText(i18n(QStringLiteral("Light"), QStringLiteral("浅色")));
	}
	if (m_darkThemeAction)
	{
		m_darkThemeAction->setText(i18n(QStringLiteral("Dark"), QStringLiteral("深色")));
	}
	if (m_languageMenu) m_languageMenu->setTitle(i18n(QStringLiteral("Language"), QStringLiteral("语言")));
	if (m_languageEnglishAction) m_languageEnglishAction->setText(QStringLiteral("English"));
	if (m_languageChineseAction) m_languageChineseAction->setText(QStringLiteral("中文"));
	if (m_languageEnglishAction) m_languageEnglishAction->setChecked(!m_useChinese);
	if (m_languageChineseAction) m_languageChineseAction->setChecked(m_useChinese);

	if (m_viewModeAction) m_viewModeAction->setText(i18n(QStringLiteral("View Mode"), QStringLiteral("视图模式")));
	if (m_objectModeAction) m_objectModeAction->setText(i18n(QStringLiteral("Object Select"), QStringLiteral("对象选择")));
	if (m_pointPickModeAction) m_pointPickModeAction->setText(i18n(QStringLiteral("Point Pick"), QStringLiteral("点选模式")));
	if (m_meshLinePickModeAction) m_meshLinePickModeAction->setText(i18n(QStringLiteral("Line Pick"), QStringLiteral("线选择模式")));
	if (m_meshFacePickModeAction) m_meshFacePickModeAction->setText(i18n(QStringLiteral("Face Pick"), QStringLiteral("面选择模式")));
	if (m_gizmoLocalFrameAction)
	{
		m_gizmoLocalFrameAction->setText(i18n(QStringLiteral("Transform: Local (object axes)"),
			QStringLiteral("变换：物体系（罗盘轴）")));
	}
	if (m_gizmoWorldFrameAction)
	{
		m_gizmoWorldFrameAction->setText(i18n(QStringLiteral("Transform: World"), QStringLiteral("变换：世界系")));
	}
	if (m_simulationStartAction)
	{
		m_simulationStartAction->setText(i18n(QStringLiteral("Start Simulation"), QStringLiteral("开始仿真")));
	}

	if (m_propertyDock) m_propertyDock->setWindowTitle(i18n(QStringLiteral("Property / Devices"), QStringLiteral("属性 / 设备")));
	if (m_propertyDockTabs && m_propertyDockTabs->count() >= 2)
	{
		m_propertyDockTabs->setTabText(0, i18n(QStringLiteral("Property"), QStringLiteral("属性")));
		m_propertyDockTabs->setTabText(1, i18n(QStringLiteral("Devices"), QStringLiteral("设备")));
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Units / Simulation / Scene"),
			QStringLiteral("单元 / 仿真 / 场景")));
	}
	if (m_unitDockTabs && m_unitDockTabs->count() >= 3)
	{
		m_unitDockTabs->setTabText(0, i18n(QStringLiteral("Units"), QStringLiteral("单元部件")));
		m_unitDockTabs->setTabText(1, i18n(QStringLiteral("Simulation"), QStringLiteral("指令仿真")));
		m_unitDockTabs->setTabText(2, i18n(QStringLiteral("Scene graph"), QStringLiteral("场景层级")));
	}
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->setUseChinese(m_useChinese);
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setUseChinese(m_useChinese);
	}
	if (m_simulationDockTabs && m_simulationDockTabs->count() >= 2)
	{
		m_simulationDockTabs->setTabText(0, i18n(QStringLiteral("Instructions"), QStringLiteral("指令")));
		m_simulationDockTabs->setTabText(1, i18n(QStringLiteral("Axis control"), QStringLiteral("轴控制")));
	}
	refreshSimulationJointListFromCurrentDoc();
	if (m_runDock) m_runDock->setWindowTitle(i18n(QStringLiteral("Runtime Output"), QStringLiteral("运行信息")));
	if (m_runInfoPage) m_runInfoPage->setUiLanguage(m_useChinese);

	if (m_propertyBrowser)
	{
		if (QTreeWidget* tw = m_propertyBrowser->findChild<QTreeWidget*>())
		{
			tw->setHeaderLabels(QStringList()
				<< i18n(QStringLiteral("Property"), QStringLiteral("属性"))
				<< i18n(QStringLiteral("Value"), QStringLiteral("值")));
			tw->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked
				| QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed);
		}
	}
	if (m_osgSceneTree)
	{
		m_osgSceneTree->setHeaderLabels(QStringList()
			<< i18n(QStringLiteral("Node"), QStringLiteral("节点"))
			<< i18n(QStringLiteral("Local transform"), QStringLiteral("本地变换矩阵")));
	}
	if (m_backendRootItem)
	{
		m_backendRootItem->setText(0, i18n(QStringLiteral("BackendDataManager"), QStringLiteral("后端数据管理器")));
	}
	if (m_annotationRootItem)
	{
		m_annotationRootItem->setText(0, i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));
	}
	refreshBackendTree();
}

void MainWindow::onSelectedObjectPoseChanged(float x, float y, float z)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	BackendVec3 euler{};
	if (osgW)
	{
		const osg::Vec3f er = osgW->selectedRotationEulerDeg();
		euler.x = static_cast<double>(er.x());
		euler.y = static_cast<double>(er.y());
		euler.z = static_cast<double>(er.z());
	}
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data);
	if (pointCloud)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		if (!osgW)
		{
			euler = pointCloud->rotation();
		}
		if (pointCloud->supportsBackendTransform())
		{
			pointCloud->applyBackendWorldPose(pose, euler);
		}
		else
		{
			pointCloud->setPose(pose);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data);
	if (mesh)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		if (!osgW)
		{
			euler = mesh->rotation();
		}
		if (mesh->supportsBackendTransform())
		{
			mesh->applyBackendWorldPose(pose, euler);
		}
		else
		{
			mesh->setPose(pose);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onSelectedObjectRotationChanged(float rx, float ry, float rz)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	BackendVec3 pos{};
	if (osgW)
	{
		const osg::Vec3f p = osgW->selectedPosition();
		pos.x = static_cast<double>(p.x());
		pos.y = static_cast<double>(p.y());
		pos.z = static_cast<double>(p.z());
	}
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data);
	if (pointCloud)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		if (!osgW)
		{
			pos = pointCloud->pose();
		}
		if (pointCloud->supportsBackendTransform())
		{
			pointCloud->applyBackendWorldPose(pos, rot);
		}
		else
		{
			pointCloud->setPose(pos);
			pointCloud->setRotation(rot);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data);
	if (mesh)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		if (!osgW)
		{
			pos = mesh->pose();
		}
		if (mesh->supportsBackendTransform())
		{
			mesh->applyBackendWorldPose(pos, rot);
		}
		else
		{
			mesh->setPose(pos);
			mesh->setRotation(rot);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onSelectedObjectColorChanged(float r, float g, float b, float a)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		pc->setColor(c);
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pc);
		return;
	}
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		mesh->setColor(c);
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onTransformGizmoCommitted()
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data)
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	if (osgW)
	{
		(void)osgW->writeActiveBackendPoseFromOsg(*data);
	}
	DocumentPage* doc = currentPage();
	if (doc)
	{
		doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
	}
	updatePropertyPanel(data);
}

void MainWindow::refreshFollowSolveAndPropertyPanelFromOsgWrite(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data)
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = doc ? doc->osgWidget() : nullptr;
	if (doc && osg)
	{
		doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
		if (!osg->isTransformGizmoDragging())
		{
			runBackendFollowSolveAndSync(*doc, *osg);
		}
	}
	if (!osg || !osg->isTransformGizmoDragging())
	{
		updatePropertyPanel(data);
	}
}

void MainWindow::schedulePropertyPanelCommitRefresh(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data)
	{
		return;
	}
	m_propertyPanelCommitPendingBackendId = QString::fromStdString(data->id());
	m_propertyPanelCommitTimer.start(220);
}

void MainWindow::onPropertyPanelCommitTimer()
{
	const QString want = m_propertyPanelCommitPendingBackendId;
	m_propertyPanelCommitPendingBackendId.clear();
	if (want.isEmpty())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection() || m_selectionState.selectedBackendId() != want)
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> d = MainWindowSelectionService::selectedBackendData(*this);
	if (!d || QString::fromStdString(d->id()) != want)
	{
		return;
	}
	updatePropertyPanel(d);
}

void MainWindow::onActiveAxisChanged(const QString& axisName)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	m_activeAxisName = axisName;
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	updatePropertyPanel(MainWindowSelectionService::currentSelection(*this).data);
}

void MainWindow::onViewModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
}

void MainWindow::onObjectModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(true);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(true);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);

	// Allow gizmo / transform whenever the scene has a loaded object; tree refresh (e.g. language)
	// can clear the selection without unloading the scene.
	if (osg->hasImportedContent())
	{
		osg->setSelectionActive(true);
	}
}

void MainWindow::onPointPickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(true);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(true);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
}

void MainWindow::onMeshLinePickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(true);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(true);
	osg->setMeshFacePickMode(false);
}

void MainWindow::onMeshFacePickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(true);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
}

void MainWindow::onSelectionCanceledByEsc()
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	// OsgWidget emits this from ESC to leave object-select / point-pick; camera stays put (see OsgWidget manipulator attach).
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
	MainWindowSelectionService::clearSelection(*this, true);
}

void MainWindow::onLanguageEnglishTriggered()
{
	m_useChinese = false;
	applyLanguage();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("UI language switched to English."));
	}
}

void MainWindow::onLanguageChineseTriggered()
{
	m_useChinese = true;
	applyLanguage();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("界面语言已切换为中文。"));
	}
}

void MainWindow::onThemeActionGroupTriggered(QAction* action)
{
	if (!action)
	{
		return;
	}
	if (action == m_lightThemeAction)
	{
		ApplicationStyle::applyTheme(qApp, ApplicationStyle::Theme::Light);
		ApplicationStyle::saveTheme(ApplicationStyle::Theme::Light);
		setAllDocumentViewerDarkBackground(false);
	}
		else if (action == m_darkThemeAction)
	{
		ApplicationStyle::applyTheme(qApp, ApplicationStyle::Theme::Dark);
		ApplicationStyle::saveTheme(ApplicationStyle::Theme::Dark);
		setAllDocumentViewerDarkBackground(true);
	}
}

void MainWindow::setAllDocumentViewerDarkBackground(bool dark)
{
	if (!m_documentTabs)
	{
		return;
	}
	for (int i = 0; i < m_documentTabs->count(); ++i)
	{
		auto* p = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
		if (p && p->osgWidget())
		{
			p->osgWidget()->setViewerBackgroundForDarkUi(dark);
		}
	}
}

bool MainWindow::viewerUsesDarkBackground() const
{
	if (m_darkThemeAction && m_lightThemeAction)
	{
		return m_darkThemeAction->isChecked();
	}
	return ApplicationStyle::loadSavedTheme() == ApplicationStyle::Theme::Dark;
}

DocumentPage* MainWindow::currentPage() const
{
	if (!m_documentTabs)
	{
		return nullptr;
	}
	return qobject_cast<DocumentPage*>(m_documentTabs->currentWidget());
}

OsgWidget* MainWindow::currentOsgWidget() const
{
	DocumentPage* p = currentPage();
	return p ? p->osgWidget() : nullptr;
}

BackendDataManager& MainWindow::activeBackend()
{
	static BackendDataManager s_unused;
	DocumentPage* p = currentPage();
	return p ? p->backend() : s_unused;
}

BackendHierarchyModel* MainWindow::activeHierarchyModel()
{
	DocumentPage* p = currentPage();
	return p ? &p->hierarchyModel() : nullptr;
}

const BackendHierarchyModel* MainWindow::activeHierarchyModel() const
{
	DocumentPage* p = currentPage();
	return p ? &p->hierarchyModel() : nullptr;
}

void MainWindow::wireDocumentPageSignals(DocumentPage* page)
{
	if (!page || !page->osgWidget())
	{
		return;
	}
	OsgWidget* o = page->osgWidget();
	connect(o, &OsgWidget::selectedObjectPoseChanged, this, &MainWindow::onSelectedObjectPoseChanged);
	connect(o, &OsgWidget::selectedObjectRotationChanged, this, &MainWindow::onSelectedObjectRotationChanged);
	connect(o, &OsgWidget::selectedObjectColorChanged, this, &MainWindow::onSelectedObjectColorChanged);
	connect(o, &OsgWidget::transformGizmoCommitted, this, &MainWindow::onTransformGizmoCommitted);
	connect(o, &OsgWidget::activeAxisChanged, this, &MainWindow::onActiveAxisChanged);
	connect(o, &OsgWidget::selectionCanceledByEsc, this, &MainWindow::onSelectionCanceledByEsc);
	connect(o, &OsgWidget::annotationCreated, this, &MainWindow::onAnnotationCreated);
	connect(o, &OsgWidget::annotationRemoved, this, &MainWindow::onAnnotationRemoved);
	connect(o, &OsgWidget::annotationVisibilityChanged, this, &MainWindow::onAnnotationVisibilityChanged);
	connect(o, &OsgWidget::pointPickFeedback, this, &MainWindow::onPointPickFeedback);
	connect(o, &OsgWidget::backendObjectPicked, this, &MainWindow::onOsgBackendObjectPicked);
	installBackendFollowFrameHook(page);
}

void MainWindow::installBackendFollowFrameHook(DocumentPage* page)
{
	if (!page || !page->osgWidget())
	{
		return;
	}
	OsgWidget* osg = page->osgWidget();
	osg->setPerFrameHook([this, page](OsgWidget* o) {
		if (!page || !o || !m_documentTabs || m_documentTabs->currentWidget() != page)
		{
			return;
		}
		if (page->followDirtyBackendIds().empty() && !page->followSolveForcedPending() && !o->isTransformGizmoDragging())
		{
			return;
		}
		runBackendFollowSolveAndSync(*page, *o);
	});
}

void MainWindow::runBackendFollowSolveAndSync(DocumentPage& page, OsgWidget& osg,
	const std::string* manualPoseAuthorityBackendId)
{
	BackendDataManager& mgr = page.backend();
	const bool forced = page.takeFollowSolveForced();
	auto& dirty = page.followDirtyBackendIds();
	const bool gizmoDrag = osg.isTransformGizmoDragging();
	if (!forced && dirty.empty() && !gizmoDrag)
	{
		return;
	}
	std::string skipId;
	std::string gizmoDragSelectedId;
	std::string manualAuthorityId;
	if (manualPoseAuthorityBackendId && !manualPoseAuthorityBackendId->empty())
	{
		manualAuthorityId = *manualPoseAuthorityBackendId;
		skipId = manualAuthorityId;
	}
	if (gizmoDrag && m_selectionState.hasBackendSelection())
	{
		gizmoDragSelectedId = m_selectionState.selectedBackendId().toStdString();
		skipId = gizmoDragSelectedId;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&osg](const std::string& bid, BackendMat4& out) -> bool {
		osg::Matrixd om;
		if (!osg.getBackendRootWorldMatrix(bid, om))
		{
			return false;
		}
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				out.v[c * 4 + r] = om(r, c);
			}
		}
		return true;
	};
	const bool usePoseLimit = !forced && !gizmoDrag && !dirty.empty();
	const std::unordered_set<std::string>* limitPtr = usePoseLimit ? &dirty : nullptr;
	BackendFollowTransformSolver::solve(mgr, worldQuery, skipId, limitPtr);
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp || !comp->enabled() || comp->targetBackendId().empty())
		{
			continue;
		}
		const std::string fid = d->id();
		if (!gizmoDragSelectedId.empty() && fid == gizmoDragSelectedId)
		{
			continue;
		}
		if (!manualAuthorityId.empty() && fid == manualAuthorityId)
		{
			continue;
		}
		if (usePoseLimit && !dirty.count(fid))
		{
			continue;
		}
		page.sceneFacade().bridge().syncOuterPatFromBackend(*d);
	}
	dirty.clear();
}

void MainWindow::applyHierarchyFollowBinding(DocumentPage* doc, const std::string& childId, const std::string& parentId)
{
	if (!doc)
	{
		return;
	}
	OsgWidget* osg = doc->osgWidget();
	const std::shared_ptr<BackendDataBase> child = doc->backend().getData(childId);
	if (!child || !child->hasPoseProperty())
	{
		return;
	}
	if (parentId.empty())
	{
		const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			child->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (f && f->hierarchyDriven())
		{
			child->removeComponent(FollowAttachmentComponent::typeKeyStatic());
		}
		if (osg)
		{
			doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), childId);
			runBackendFollowSolveAndSync(*doc, *osg);
		}
		doc->invalidateFollowReverseIndex();
		return;
	}
	if (!doc->backend().contains(parentId))
	{
		return;
	}
	if (const auto f0 = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			child->getComponent(FollowAttachmentComponent::typeKeyStatic())))
	{
		if (f0->enabled() && !f0->hierarchyDriven() && !f0->targetBackendId().empty())
		{
			return;
		}
	}
	if (!child->getComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		child->addComponent(std::make_shared<FollowAttachmentComponent>());
	}
	const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		child->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	f->setHierarchyDriven(true);
	f->setEnabled(true);
	f->setTargetBackendId(parentId);
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [osg](const std::string& bid, BackendMat4& out) -> bool {
		if (osg)
		{
			osg::Matrixd om;
			if (osg->getBackendRootWorldMatrix(bid, om))
			{
				for (int c = 0; c < 4; ++c)
				{
					for (int r = 0; r < 4; ++r)
					{
						out.v[c * 4 + r] = om(r, c);
					}
				}
				return true;
			}
		}
		return false;
	};
	(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(doc->backend(), worldQuery, *child, nullptr);
	if (osg)
	{
		doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), childId);
		runBackendFollowSolveAndSync(*doc, *osg);
	}
	doc->invalidateFollowReverseIndex();
}

void MainWindow::afterBackendFollowPropertyEdited(const QString& propertyKey, const QString& valueText)
{
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !osg || !m_selectionState.hasBackendSelection())
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data)
	{
		return;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [osg](const std::string& bid, BackendMat4& out) -> bool {
		osg::Matrixd om;
		if (!osg->getBackendRootWorldMatrix(bid, om))
		{
			return false;
		}
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				out.v[c * 4 + r] = om(r, c);
			}
		}
		return true;
	};
	if (propertyKey == QStringLiteral("follow.targetId")
		|| propertyKey == QStringLiteral("follow.targetName")
		|| (propertyKey == QStringLiteral("follow.enabled")
			&& (valueText == QStringLiteral("1") || valueText.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0)))
	{
		(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(doc->backend(), worldQuery, *data, nullptr);
	}
	doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
	runBackendFollowSolveAndSync(*doc, *osg);
	doc->invalidateFollowReverseIndex();
}

void MainWindow::onNewDocument()
{
	if (!m_documentTabs)
	{
		return;
	}
	auto* page = new DocumentPage(m_documentTabs);
	wireDocumentPageSignals(page);
	if (OsgWidget* osg = page->osgWidget())
	{
		osg->setViewerBackgroundForDarkUi(viewerUsesDarkBackground());
	}
	const QString title = i18n(QStringLiteral("Untitled"), QStringLiteral("未命名"));
	m_documentTabs->addTab(page, title);
	m_documentTabs->setCurrentWidget(page);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("New document."), QStringLiteral("新建文档。")));
	}
	onDocumentTabChanged(m_documentTabs->currentIndex());
}

void MainWindow::onDocumentTabChanged(int)
{
	stopRobotSimulation();
	MainWindowSelectionService::clearSelection(*this, true);
	refreshBackendTree();
	syncViewModeActionsFromCurrentOsg();
	refreshSimulationJointListFromCurrentDoc();
}

void MainWindow::syncViewModeActionsFromCurrentOsg()
{
	OsgWidget* o = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction)
	{
		return;
	}
	if (!o)
	{
		m_viewModeAction->setChecked(true);
		m_objectModeAction->setChecked(false);
		m_pointPickModeAction->setChecked(false);
		m_meshLinePickModeAction->setChecked(false);
		m_meshFacePickModeAction->setChecked(false);
		if (m_gizmoFrameGroup && m_gizmoLocalFrameAction && m_gizmoWorldFrameAction)
		{
			const QSignalBlocker bg(m_gizmoFrameGroup);
			const QSignalBlocker b1(m_gizmoLocalFrameAction);
			const QSignalBlocker b2(m_gizmoWorldFrameAction);
			m_gizmoLocalFrameAction->setChecked(true);
			m_gizmoWorldFrameAction->setChecked(false);
		}
		return;
	}
	const bool view = !o->objectSelectionMode() && !o->pointPickMode() && !o->meshLinePickMode() && !o->meshFacePickMode();
	m_viewModeAction->setChecked(view);
	m_objectModeAction->setChecked(o->objectSelectionMode());
	m_pointPickModeAction->setChecked(o->pointPickMode());
	m_meshLinePickModeAction->setChecked(o->meshLinePickMode());
	m_meshFacePickModeAction->setChecked(o->meshFacePickMode());
	if (m_gizmoFrameGroup && m_gizmoLocalFrameAction && m_gizmoWorldFrameAction)
	{
		const QSignalBlocker bg(m_gizmoFrameGroup);
		const QSignalBlocker b1(m_gizmoLocalFrameAction);
		const QSignalBlocker b2(m_gizmoWorldFrameAction);
		if (o->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::Local)
		{
			m_gizmoLocalFrameAction->setChecked(true);
			m_gizmoWorldFrameAction->setChecked(false);
		}
		else
		{
			m_gizmoLocalFrameAction->setChecked(false);
			m_gizmoWorldFrameAction->setChecked(true);
		}
	}
}

void MainWindow::onPointPickFeedback(const QString& text)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (statusBar())
	{
		statusBar()->showMessage(text);
	}
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(text);
	}
}

void MainWindow::stopRobotSimulation()
{
	const QVector<double> lastJointAngles = m_robotProgramExecutor.jointAnglesRad();
	m_robotProgramExecutor.stop();
	m_robotSimTimer.stop();
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->setSimulationRunning(false);
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setInteractionEnabled(true);
		DocumentPage* doc = currentPage();
		const int instIdx = m_simulationCommandPage ? m_simulationCommandPage->currentRobotInstanceIndex() : 0;
		if (doc && doc->hasRobotSimulationContext() && instIdx >= 0 && !lastJointAngles.isEmpty())
		{
			const int offset = doc->robotJointOffsetInAggregatedVector(instIdx);
			const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
			if (nj > 0 && m_robotAxisControlPage->jointCount() == nj
				&& offset + nj <= lastJointAngles.size())
			{
				m_robotAxisControlPage->setJointAnglesRad(lastJointAngles.mid(offset, nj));
			}
		}
	}
}

void MainWindow::refreshSimulationJointListFromCurrentDoc()
{
	if (!m_simulationCommandPage || !m_robotAxisControlPage)
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (doc)
	{
		m_simulationCommandPage->setProgramStore(&doc->robotProgramStore());
	}
	if (doc && doc->hasRobotSimulationContext())
	{
		QStringList labels;
		QStringList backendIds;
		const int n = doc->robotKinematicInstanceCount();
		for (int i = 0; i < n; ++i)
		{
			labels.append(doc->robotDisplayLabelForInstance(i));
			backendIds.append(doc->robotSceneBackendIdForInstance(i));
		}
		m_simulationCommandPage->setRobotInstances(labels, backendIds);

		const int instIdx = m_simulationCommandPage->currentRobotInstanceIndex() >= 0
			? m_simulationCommandPage->currentRobotInstanceIndex()
			: 0;
		const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
		m_simulationCommandPage->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instIdx));

		QStringList tcpLinks;
		QString preferredTcp;
		(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
		QStringList childLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
		QSet<QString> uniq;
		if (!preferredTcp.isEmpty())
		{
			uniq.insert(preferredTcp);
			tcpLinks.push_back(preferredTcp);
		}
		for (const QString& l : childLinks)
		{
			if (l.isEmpty() || uniq.contains(l))
			{
				continue;
			}
			uniq.insert(l);
			tcpLinks.push_back(l);
		}
		m_simulationCommandPage->setTcpLinkOptions(tcpLinks, preferredTcp);

		QVector<double> lower;
		QVector<double> upper;
		doc->robotJointLimitsForInstance(instIdx, lower, upper);
		const QStringList jn = doc->robotRevoluteJointNamesForInstance(instIdx);
		if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
		{
			m_robotAxisControlPage->setJoints(jn, lower, upper);
		}
		else
		{
			m_robotAxisControlPage->clearJoints();
		}

		m_aggregatedJointAnglesRad = QVector<double>(doc->robotRevoluteJointNames().size(), 0.0);
		captureMotionPreviewProgramStartJoints();
	}
	else
	{
		m_simulationCommandPage->setRobotInstances(QStringList(), QStringList());
		m_simulationCommandPage->setRevoluteJointNames(QStringList());
		m_simulationCommandPage->setTcpLinkOptions(QStringList(), QString());
		m_robotAxisControlPage->clearJoints();
		m_aggregatedJointAnglesRad.clear();
		m_motionPreviewProgramStartJointRad.clear();
	}
}

void MainWindow::captureMotionPreviewProgramStartJoints()
{
	DocumentPage* doc = currentPage();
	if (!doc || !m_simulationCommandPage || !doc->hasRobotSimulationContext())
	{
		m_motionPreviewProgramStartJointRad.clear();
		return;
	}
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	m_motionPreviewProgramStartJointRad = QVector<double>(jnamesAll.size(), 0.0);
	const int instIdx = m_simulationCommandPage->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == nj)
	{
		const QVector<double> local = m_robotAxisControlPage->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			m_motionPreviewProgramStartJointRad[jointOffset + j] = local[j];
		}
	}
}

QVector<double> MainWindow::motionPreviewProgramStartJointsLocal(const int nj, const int jointOffset) const
{
	QVector<double> rollingQ(nj, 0.0);
	if (nj <= 0)
	{
		return rollingQ;
	}
	if (m_motionPreviewProgramStartJointRad.size() > jointOffset)
	{
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			rollingQ[j] = m_motionPreviewProgramStartJointRad[jointOffset + j];
		}
		return rollingQ;
	}
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == nj)
	{
		return m_robotAxisControlPage->jointAnglesRad();
	}
	return rollingQ;
}

void MainWindow::onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId)
{
	(void)sceneBackendId;
	DocumentPage* doc = currentPage();
	if (!doc || !m_simulationCommandPage || !m_robotAxisControlPage || instanceIndex < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	m_simulationCommandPage->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instanceIndex));

	QStringList tcpLinks;
	QString preferredTcp;
	(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	QSet<QString> uniq;
	if (!preferredTcp.isEmpty())
	{
		uniq.insert(preferredTcp);
		tcpLinks.push_back(preferredTcp);
	}
	for (const QString& l : childLinks)
	{
		if (l.isEmpty() || uniq.contains(l))
		{
			continue;
		}
		uniq.insert(l);
		tcpLinks.push_back(l);
	}
	m_simulationCommandPage->setTcpLinkOptions(tcpLinks, preferredTcp);

	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instanceIndex, lower, upper);
	const QStringList jn = doc->robotRevoluteJointNamesForInstance(instanceIndex);
	if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
	{
		m_robotAxisControlPage->setJoints(jn, lower, upper);
	}
	captureMotionPreviewProgramStartJoints();
	invalidateFeasibleAxisConfigurationCache();
}

void MainWindow::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_robotProgramExecutor.isRunning())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	IRobotBackendPoseSink* poseSink = doc ? doc->sceneFacade().poseSink() : nullptr;
	if (!doc || !poseSink)
	{
		return;
	}
	const int instIdx = m_simulationCommandPage ? m_simulationCommandPage->currentRobotInstanceIndex() : 0;
	if (instIdx < 0)
	{
		return;
	}
	if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
	{
		m_aggregatedJointAnglesRad = QVector<double>(doc->robotRevoluteJointNames().size(), 0.0);
	}
	(void)RobotSceneKinematics::applyJointAnglesForInstance(
		doc, poseSink, instIdx, jointAnglesRad, m_aggregatedJointAnglesRad);
	if (!m_suppressMotionPreviewStartCapture)
	{
		captureMotionPreviewProgramStartJoints();
		invalidateFeasibleAxisConfigurationCache();
	}
}

void MainWindow::onSimulationStopRequested()
{
	stopRobotSimulation();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Simulation stopped."), QStringLiteral("仿真已停止。")));
	}
}

void MainWindow::onSimulationRunRequested()
{
	onSimulationStartTriggered();
}

bool MainWindow::tryCaptureCurrentRobotTcpPose(
	RobotInstruction::Vec3& outPoseMm,
	RobotInstruction::Vec3& outEulerDeg,
	osg::Matrixd* outTcpLocalMat,
	osg::Matrixd* outTcpRenderWorldMat,
	QString* outTcpLinkName,
	QString* errMsg) const
{
	DocumentPage* doc = currentPage();
	if (!doc || !doc->hasRobotSimulationContext())
	{
		if (errMsg)
		{
			*errMsg = i18n(QStringLiteral("Robot simulation context is not ready."),
				QStringLiteral("机器人仿真上下文尚未就绪。"));
		}
		return false;
	}
	const int instIdx = m_simulationCommandPage && m_simulationCommandPage->currentRobotInstanceIndex() >= 0
		? m_simulationCommandPage->currentRobotInstanceIndex()
		: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = i18n(QStringLiteral("URDF path is empty."),
				QStringLiteral("URDF 路径为空。"));
		}
		return false;
	}
	const QString jointPrefix = doc->robotJointKeyPrefixForInstance(instIdx);
	const QStringList jointsLocal = doc->robotRevoluteJointNamesForInstance(instIdx);
	QVector<double> q(jointsLocal.size(), 0.0);
	QString qSource = QStringLiteral("zero-fallback");
	osg::Matrixd robotRootWorld;
	robotRootWorld.makeIdentity();
	if (OsgWidget* osg = currentOsgWidget())
	{
		const QString robotRootId = doc->robotSceneBackendIdForInstance(instIdx);
		if (!robotRootId.isEmpty())
		{
			osg::Matrixd m;
			if (osg->getBackendRootWorldMatrix(robotRootId.toStdString(), m))
			{
				robotRootWorld = m;
			}
		}
	}
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == jointsLocal.size())
	{
		q = m_robotAxisControlPage->jointAnglesRad();
		qSource = QStringLiteral("slider");
	}
	(void)qSource;
	const QString lastJointName = jointsLocal.isEmpty() ? QString() : (jointPrefix + jointsLocal.back());
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString computeErr;
	const bool hasLinkFk = UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, q, linkWorldByName, &computeErr);

	QString tcpLinkName;
	{
		// Link frame fallback only (primary path uses last revolute joint frame).
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			tcpLinkName = revoluteChildLinks.back();
		}
	}
	if (tcpLinkName.isEmpty() && m_simulationCommandPage)
	{
		// Fallback only when URDF does not expose revolute child links.
		tcpLinkName = m_simulationCommandPage->selectedTcpLink();
	}
	osg::Matrixd tcpLocal;
	bool hasTcpLocal = false;
	QString tcpSource = QStringLiteral("None");
	// Hierarchy pose of last revolute joint (OSG scene graph); may differ from URDF child link frame.
	osg::Matrixd tcpLocalFromHierarchy;
	bool hasHierarchyLocal = false;
	if (!lastJointName.isEmpty())
	{
		if (osg::MatrixTransform* jointMt = doc->robotJointMatrixTransform(lastJointName))
		{
			osg::Matrixd jointWorld;
			if (matrixFromNodeWorld(jointMt, jointWorld))
			{
				tcpLocalFromHierarchy = osg::Matrixd::inverse(robotRootWorld) * jointWorld;
				hasHierarchyLocal = true;
			}
		}
	}
	// Primary: URDF link FK for tcpLinkName — same frame as RobotInstructionController URDF IK / FK.
	// Using the joint MatrixTransform first caused "add at current pose then run moves" when joint ≠ link frame.
	if (hasLinkFk && !tcpLinkName.isEmpty() && linkWorldByName.contains(tcpLinkName))
	{
		tcpLocal = linkWorldByName.value(tcpLinkName);
		hasTcpLocal = true;
		tcpSource = QStringLiteral("LinkFK");
	}
	else if (hasHierarchyLocal)
	{
		tcpLocal = tcpLocalFromHierarchy;
		hasTcpLocal = true;
		tcpSource = QStringLiteral("HierarchyJoint");
	}
	else if (!lastJointName.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = i18n(QStringLiteral("Cannot evaluate TCP world transform."),
				QStringLiteral("无法获取末端世界坐标。"));
		}
		return false;
	}
	if (!hasTcpLocal)
	{
		if (errMsg)
		{
			*errMsg = i18n(QStringLiteral("No link transform is available for TCP."),
				QStringLiteral("当前没有可用的末端关节/连杆位姿。"));
		}
		return false;
	}

	// Keep captured TCP matrix directly. In current URDF->OSG pipeline this matrix is already
	// in the same scene frame used by FK comparison and overlay rendering; multiplying rootWorld
	// here introduces an extra transform and shifts the marker.
	const osg::Matrixd tcpWorld = tcpLocal;
	osg::Matrixd renderWorld = tcpWorld;
	if (hasHierarchyLocal)
	{
		// Rendering should align with the visible robot scene graph in world space.
		renderWorld = robotRootWorld * tcpLocalFromHierarchy;
	}
	const osg::Vec3d t = tcpWorld.getTrans();
	const osg::Vec3f euler = OsgScene::quatToEulerDeg(tcpWorld.getRotate());
	outPoseMm.x = t.x();
	outPoseMm.y = t.y();
	outPoseMm.z = t.z();
	outEulerDeg.x = euler.x();
	outEulerDeg.y = euler.y();
	outEulerDeg.z = euler.z();
	if (outTcpLocalMat)
	{
		*outTcpLocalMat = tcpWorld;
	}
	if (outTcpRenderWorldMat)
	{
		*outTcpRenderWorldMat = renderWorld;
	}
	if (outTcpLinkName)
	{
		// Planning/IK reference should be a link name (URDF link frame key).
		*outTcpLinkName = tcpLinkName;
	}
	return true;
}

void MainWindow::onSimulationAddInstructionRequested(RobotInstruction::Type type)
{
	if (!m_simulationCommandPage)
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	osg::Matrixd tcpLocalMat;
	osg::Matrixd tcpRenderWorldMat;
	QString tcpLinkName;
	QString err;
	if (!tryCaptureCurrentRobotTcpPose(pose, euler, &tcpLocalMat, &tcpRenderWorldMat, &tcpLinkName, &err))
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(err);
		}
		return;
	}
	const std::shared_ptr<RobotInstruction::Base> ins = m_simulationCommandPage->appendInstructionFromCurrentPose(type, pose, euler);
	invalidateFeasibleAxisConfigurationCache();
	if (ins)
	{
		const std::string matCsv = encodeMatrix4Csv(tcpLocalMat);
		const std::string renderMatCsv = encodeMatrix4Csv(tcpRenderWorldMat);
		const osg::Matrixd renderWorldToFk = tcpLocalMat * osg::Matrixd::inverse(tcpRenderWorldMat);
		const std::string renderToFkCsv = encodeMatrix4Csv(renderWorldToFk);
		const osg::Vec3d deltaPosMm = tcpLocalMat.getTrans() - tcpRenderWorldMat.getTrans();
		std::ostringstream deltaOss;
		deltaOss.imbue(std::locale::classic());
		deltaOss << deltaPosMm.x() << "," << deltaPosMm.y() << "," << deltaPosMm.z();
		ins->setExtensionProperty("render.tcpWorldMat4", renderMatCsv);
		ins->setExtensionProperty("render.tcpLocalMat4", matCsv);
		ins->setExtensionProperty("render.tcpLinkName", tcpLinkName.toStdString());
		ins->setExtensionProperty("context.renderWorldToFkMat4", renderToFkCsv);
		ins->setExtensionProperty("context.renderToFkDeltaPosMmCsv", deltaOss.str());
		ins->setExtensionProperty("context.capturedTcpLinkName", tcpLinkName.toStdString());
		if (ins->hasMotionAxisConfigurationProperty() && m_robotAxisControlPage
			&& m_robotAxisControlPage->jointCount() > 0)
		{
			QVector<double> seedQ = m_robotAxisControlPage->jointAnglesRad();
			RobotInstruction::FeasibleMotionAxisConfigurationOptions feasible =
				feasibleMotionAxisConfigurationOptionsForInstruction(ins, &seedQ);
			applySuggestedAxisPresetFromSeedIfNeeded(ins, seedQ, feasible);
			ins->setExtensionProperty("context.axisConfigSeeded", "1");
		}
	}
	refreshInstructionPoseAxes();
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions MainWindow::feasibleMotionAxisConfigurationOptionsForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	QVector<double>* outSeedJointRad)
{
	RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
	if (!instruction || !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return out;
	}
	DocumentPage* doc = currentPage();
	if (!doc || !m_simulationCommandPage || !doc->hasRobotSimulationContext())
	{
		return out;
	}
	const int instIdx = m_simulationCommandPage->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return out;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return out;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return out;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_robotInstructionController.setDhRows(dhRows);
		}
		else
		{
			m_robotInstructionController.clearDhRows();
		}
	}
	const QString robotBackendId = m_simulationCommandPage->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_simulationCommandPage->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const std::string targetId = instruction->id();
	int targetMotionIndex = -1;
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i] && motions[i]->id() == targetId)
		{
			targetMotionIndex = static_cast<int>(i);
			break;
		}
	}
	if (targetMotionIndex < 0)
	{
		return out;
	}
	const QString defaultTcpLinkName = defaultTcpLinkNameForUrdf(
		urdfPath,
		m_simulationCommandPage ? m_simulationCommandPage->selectedTcpLink() : QString());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	for (int mi = 0; mi < targetMotionIndex; ++mi)
	{
		RobotInstruction::Base* motionIns = const_cast<RobotInstruction::Base*>(motions[static_cast<size_t>(mi)]);
		if (!motionIns)
		{
			continue;
		}
		const MotionPoseBackup backup = backupInstructionPose(*motionIns);
		mapInstructionPoseRenderToFk(*motionIns);
		attachMotionPlanningContext(
			*motionIns,
			rollingQ,
			urdfPath,
			defaultTcpLinkName.toStdString());
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		if (m_robotInstructionController.plan(*motionIns, plan, &planErr)
			&& !plan.jointTargetsRad.empty()
			&& plan.jointTargetsRad.size() == static_cast<size_t>(nj))
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		restoreInstructionPose(*motionIns, backup);
	}
	QString fingerprint = QString::fromStdString(instruction->id());
	if (instruction->hasPoseProperty())
	{
		const RobotInstruction::Vec3 p = instruction->pose();
		const RobotInstruction::Vec3 e = instruction->eulerDeg();
		fingerprint += QStringLiteral("|%1,%2,%3|%4,%5,%6")
							.arg(p.x, 0, 'g', 8)
							.arg(p.y, 0, 'g', 8)
							.arg(p.z, 0, 'g', 8)
							.arg(e.x, 0, 'g', 8)
							.arg(e.y, 0, 'g', 8)
							.arg(e.z, 0, 'g', 8);
	}
	fingerprint += QStringLiteral("|mi=%1").arg(targetMotionIndex);
	for (int j = 0; j < rollingQ.size(); ++j)
	{
		fingerprint += QLatin1Char(',') + QString::number(rollingQ[j], 'g', 8);
	}
	if (m_cachedFeasibleAxisInstructionId == QString::fromStdString(instruction->id())
		&& m_cachedFeasibleAxisFingerprint == fingerprint && !m_cachedFeasibleAxisOptions.presetTokens.empty())
	{
		if (outSeedJointRad)
		{
			*outSeedJointRad = m_cachedFeasibleAxisSeedJointRad;
		}
		return m_cachedFeasibleAxisOptions;
	}

	const MotionPoseBackup targetBackup = backupInstructionPose(*instruction);
	mapInstructionPoseRenderToFk(*instruction);
	attachMotionPlanningContext(
		*instruction,
		rollingQ,
		urdfPath,
		defaultTcpLinkName.toStdString());
	out = m_robotInstructionController.queryFeasibleMotionAxisConfigurationOptions(*instruction);
	restoreInstructionPose(*instruction, targetBackup);

	m_cachedFeasibleAxisInstructionId = QString::fromStdString(instruction->id());
	m_cachedFeasibleAxisFingerprint = fingerprint;
	m_cachedFeasibleAxisOptions = out;
	m_cachedFeasibleAxisSeedJointRad = rollingQ;
	if (outSeedJointRad)
	{
		*outSeedJointRad = rollingQ;
	}
	return out;
}

void MainWindow::onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	m_activeInstructionForProperty = instruction;
	if (instruction)
	{
		MainWindowSelectionService::clearBackendObjectSelection(*this, true);
	}
	updateInstructionPropertyPanel(instruction);
	if (instruction && instruction->hasMotionAxisConfigurationProperty())
	{
		const auto& ext = instruction->extensionProperties();
		if (instruction->motionAxisConfiguration().preset == "AUTO"
			&& ext.find("context.axisConfigSeeded") == ext.end())
		{
			QVector<double> seedQ;
			const RobotInstruction::FeasibleMotionAxisConfigurationOptions feasible =
				feasibleMotionAxisConfigurationOptionsForInstruction(instruction, &seedQ);
			applySuggestedAxisPresetFromSeedIfNeeded(instruction, seedQ, feasible);
			instruction->setExtensionProperty("context.axisConfigSeeded", "1");
			updateInstructionPropertyPanel(instruction, false);
		}
	}
	refreshInstructionPoseAxes();
	applyRobotPoseForInstructionPreview(instruction);
}

void MainWindow::applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_robotProgramExecutor.isRunning() || !instruction
		|| !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return;
	}
	DocumentPage* doc = currentPage();
	IRobotBackendPoseSink* poseSink = doc ? doc->sceneFacade().poseSink() : nullptr;
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !poseSink || !osg || !m_simulationCommandPage || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_simulationCommandPage->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_robotInstructionController.setDhRows(dhRows);
		}
		else
		{
			m_robotInstructionController.clearDhRows();
		}
	}
	const QString robotBackendId = m_simulationCommandPage->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_simulationCommandPage->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const std::string targetId = instruction->id();
	int targetMotionIndex = -1;
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i] && motions[i]->id() == targetId)
		{
			targetMotionIndex = static_cast<int>(i);
			break;
		}
	}
	if (targetMotionIndex < 0)
	{
		return;
	}
	const QString defaultTcpLinkName = defaultTcpLinkNameForUrdf(
		urdfPath,
		m_simulationCommandPage ? m_simulationCommandPage->selectedTcpLink() : QString());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	for (int mi = 0; mi <= targetMotionIndex; ++mi)
	{
		RobotInstruction::Base* motionIns = const_cast<RobotInstruction::Base*>(motions[static_cast<size_t>(mi)]);
		if (!motionIns)
		{
			continue;
		}
		const MotionPoseBackup backup = backupInstructionPose(*motionIns);
		mapInstructionPoseRenderToFk(*motionIns);
		attachMotionPlanningContext(
			*motionIns,
			rollingQ,
			urdfPath,
			defaultTcpLinkName.toStdString());
		std::string planErr;
		if (!m_robotInstructionController.validate(*motionIns, &planErr))
		{
			restoreInstructionPose(*motionIns, backup);
			if (m_runInfoPage && mi == targetMotionIndex)
			{
				const QString pointTag = QString::fromStdString(
					RobotInstruction::formatMotionPointName(RobotInstruction::motionPointIndex(*motionIns)));
				m_runInfoPage->appendWarning(
					pointTag.isEmpty()
						? QString::fromStdString(planErr)
						: QStringLiteral("%1: %2").arg(pointTag, QString::fromStdString(planErr)));
			}
			return;
		}
		RobotInstruction::PlanResult plan{};
		if (!m_robotInstructionController.plan(*motionIns, plan, &planErr))
		{
			restoreInstructionPose(*motionIns, backup);
			if (m_runInfoPage && mi == targetMotionIndex)
			{
				const QString pointTag = QString::fromStdString(
					RobotInstruction::formatMotionPointName(RobotInstruction::motionPointIndex(*motionIns)));
				m_runInfoPage->appendWarning(
					pointTag.isEmpty()
						? QString::fromStdString(planErr)
						: QStringLiteral("%1: %2").arg(pointTag, QString::fromStdString(planErr)));
			}
			return;
		}
		restoreInstructionPose(*motionIns, backup);
		if (!plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(nj))
		{
			for (int j = 0; j < nj; ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
	}
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	if (m_aggregatedJointAnglesRad.size() != jnamesAll.size())
	{
		m_aggregatedJointAnglesRad = QVector<double>(jnamesAll.size(), 0.0);
	}
	for (int j = 0; j < nj && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
	{
		m_aggregatedJointAnglesRad[jointOffset + j] = rollingQ[j];
	}
	(void)RobotSceneKinematics::applyJointAnglesForInstance(
		doc, poseSink, instIdx, rollingQ, m_aggregatedJointAnglesRad);
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == nj)
	{
		m_suppressMotionPreviewStartCapture = true;
		const QSignalBlocker blocker(m_robotAxisControlPage);
		m_robotAxisControlPage->setJointAnglesRad(rollingQ);
		m_suppressMotionPreviewStartCapture = false;
	}
	osg->requestRedraw();
}

void MainWindow::refreshInstructionPoseAxes()
{
	OsgWidget* osg = currentOsgWidget();
	DocumentPage* doc = currentPage();
	if (!osg || !m_simulationCommandPage)
	{
		if (osg)
		{
			osg->clearInstructionPoseAxes();
		}
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> insList = m_simulationCommandPage->instructionList();
	std::vector<OsgWidget::InstructionPoseAxis> axes;
	axes.reserve(insList.size());
	std::string robotBackendId;
	if (m_simulationCommandPage && !m_simulationCommandPage->currentRobotBackendId().isEmpty())
	{
		robotBackendId = m_simulationCommandPage->currentRobotBackendId().toStdString();
	}
	else if (doc && !doc->robotSceneBackendId().isEmpty())
	{
		robotBackendId = doc->robotSceneBackendId().toStdString();
	}
	for (const auto& ins : insList)
	{
		if (!ins || !ins->hasPoseProperty() || !ins->hasEulerProperty())
		{
			continue;
		}
		const RobotInstruction::Vec3 p = ins->pose();
		const RobotInstruction::Vec3 e = ins->eulerDeg();
		OsgWidget::InstructionPoseAxis a;
		a.positionMm = osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
		a.eulerDeg = osg::Vec3f(static_cast<float>(e.x), static_cast<float>(e.y), static_cast<float>(e.z));
		a.lineMotion = (ins->type() == RobotInstruction::Type::LINE);
		// Keep instruction markers in world frame and parent under overlay root.
		a.robotBackendId.clear();
		a.urdfTcpAttachLinkName.clear();
		// Render target axes from current editable pose/euler to keep display
		// consistent with what planner/IK receives.
		a.hasLocalMatrix = false;
		axes.push_back(a);
	}
	if (axes.empty())
	{
		osg->clearInstructionPoseAxes();
		return;
	}
	osg->setInstructionPoseAxes(axes);
}

void MainWindow::onSimulationStartTriggered()
{
	if (m_robotProgramExecutor.isRunning())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !osg || !m_simulationCommandPage)
	{
		return;
	}
	if (!doc->hasRobotSimulationContext())
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(
				QStringLiteral("Import a robot (URDF) first, then add simulation commands."),
				QStringLiteral("请先导入机器人(URDF)，再添加仿真指令。")));
		}
		return;
	}
	const int instIdx = m_simulationCommandPage->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotBackendId = m_simulationCommandPage->currentRobotBackendId();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	{
		std::vector<robot_kinematics::DhRow> dhRows;
		QString dhErr;
		if (buildDhRowsFromUrdf(urdfPath, dhRows, &dhErr))
		{
			m_robotInstructionController.setDhRows(dhRows);
		}
		else
		{
			m_robotInstructionController.clearDhRows();
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(i18n(
					QStringLiteral("无DH上下文（切换URDF等效运动学求解）：%1").arg(dhErr),
					QStringLiteral("无DH上下文（切换URDF等效运动学求解）：%1").arg(dhErr)));
			}
		}
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(
				QStringLiteral("No revolute joints in URDF (joints need type=\"revolute\" or \"continuous\" and an axis)."),
				QStringLiteral("URDF中无可旋转关节（需 type=“revolute/continuous” 及 axis）。")));
		}
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> instructions =
		m_simulationCommandPage->instructions(robotBackendId);
	if (instructions.empty())
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(QStringLiteral("Add at least one instruction row."),
				QStringLiteral("请至少添加一条指令。")));
		}
		return;
	}
	const std::vector<const RobotInstruction::Base*> motions =
		RobotInstruction::collectMotionInstructions(instructions);
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = defaultTcpLinkNameForUrdf(
		urdfPath,
		m_simulationCommandPage ? m_simulationCommandPage->selectedTcpLink() : QString());
	QVector<double> initialAngles(jnamesAll.size(), 0.0);
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == nj)
	{
		const QVector<double> local = m_robotAxisControlPage->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < initialAngles.size(); ++j)
		{
			initialAngles[jointOffset + j] = local[j];
		}
	}
	m_aggregatedJointAnglesRad = initialAngles;
	m_motionPreviewProgramStartJointRad = initialAngles;

	std::vector<RobotInstruction::PlanResult> planResults;
	planResults.reserve(motions.size());
	QVector<double> rollingQ(nj, 0.0);
	for (int j = 0; j < nj; ++j)
	{
		rollingQ[j] = initialAngles[jointOffset + j];
	}
	for (const RobotInstruction::Base* motionPtr : motions)
	{
		if (!motionPtr)
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(i18n(
					QStringLiteral("Instruction row is invalid."),
					QStringLiteral("存在无效指令行。")));
			}
			return;
		}
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const MotionPoseBackup poseBackup = backupInstructionPose(*ins);
		mapInstructionPoseRenderToFk(*ins);
		attachMotionPlanningContext(*ins, rollingQ, urdfPath, defaultTcpLinkName.toStdString());
		std::string planErr;
		if (!m_robotInstructionController.validate(*ins, &planErr))
		{
			restoreInstructionPose(*ins, poseBackup);
			if (m_runInfoPage)
			{
				const QString msg = !planErr.empty() ? QString::fromStdString(planErr)
													 : i18n(QStringLiteral("Instruction validation failed."),
														 QStringLiteral("指令校验失败。"));
				m_runInfoPage->appendWarning(msg);
			}
			return;
		}
		RobotInstruction::PlanResult plan{};
		if (!m_robotInstructionController.plan(*ins, plan, &planErr))
		{
			restoreInstructionPose(*ins, poseBackup);
			if (m_runInfoPage)
			{
				const QString msg = !planErr.empty() ? QString::fromStdString(planErr)
													 : i18n(QStringLiteral("Instruction planning failed."),
														 QStringLiteral("指令规划失败。"));
				m_runInfoPage->appendWarning(msg);
			}
			return;
		}
		restoreInstructionPose(*ins, poseBackup);
		if (plan.durationSec > 1e-6)
		{
			ins->setExtensionProperty(
				"motion.durationSec",
				QString::number(plan.durationSec, 'f', 3).toStdString());
		}
		if (!plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
				initialAngles[jointOffset + j] = rollingQ[j];
			}
		}
		planResults.push_back(std::move(plan));
	}
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->refreshInstructionList();
	}
	QString err;
	if (!m_robotProgramExecutor.tryStart(
			doc, osg, &m_simulationIoSink, instIdx, instructions, planResults, initialAngles, &err))
	{
		if (m_runInfoPage)
		{
			if (err.contains(QLatin1String("Invalid joint index")))
			{
				m_runInfoPage->appendWarning(i18n(QStringLiteral("Invalid joint index in simulation command."),
					QStringLiteral("仿真指令关节索引无效。")));
			}
			else if (!err.isEmpty())
			{
				m_runInfoPage->appendWarning(err);
			}
		}
		return;
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setInteractionEnabled(false);
	}
	m_simulationCommandPage->setSimulationRunning(true);
	m_robotSimTimer.start();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Simulation started."), QStringLiteral("仿真已开始。")));
	}
}

void MainWindow::logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad)
{
	if (!m_runInfoPage)
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !osg || !m_simulationCommandPage || finalJointAnglesRad.isEmpty())
	{
		return;
	}
	const auto insList = m_simulationCommandPage->instructionList();
	std::shared_ptr<RobotInstruction::Base> targetIns;
	for (auto it = insList.rbegin(); it != insList.rend(); ++it)
	{
		if (*it && (*it)->hasPoseProperty())
		{
			targetIns = *it;
			break;
		}
	}
	if (!targetIns)
	{
		return;
	}

	const RobotInstruction::Vec3 targetPose = targetIns->pose();
	QString tcpLinkName;
	{
		const auto& ext = targetIns->extensionProperties();
		const auto itCaptured = ext.find("context.capturedTcpLinkName");
		if (itCaptured != ext.end() && !itCaptured->second.empty())
		{
			tcpLinkName = QString::fromStdString(itCaptured->second);
		}
		if (tcpLinkName.isEmpty())
		{
			const auto itTcp = ext.find("context.tcpLinkName");
			if (itTcp != ext.end() && !itTcp->second.empty())
			{
				tcpLinkName = QString::fromStdString(itTcp->second);
			}
		}
	}

	const auto distToTarget = [&](double x, double y, double z) {
		const double dx = x - targetPose.x;
		const double dy = y - targetPose.y;
		const double dz = z - targetPose.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	};
	const auto fmtPos = [](double x, double y, double z) {
		return QStringLiteral("(%1, %2, %3)")
			.arg(x, 0, 'f', 3)
			.arg(y, 0, 'f', 3)
			.arg(z, 0, 'f', 3);
	};

	const QString urdfPath = doc->robotUrdfAbsolutePath();
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, finalJointAnglesRad, linkWorldByName, &fkErr))
	{
		m_runInfoPage->appendWarning(QStringLiteral("[Playback对比] FK计算失败：%1").arg(fkErr));
		return;
	}

	m_runInfoPage->appendInfo(
		QStringLiteral("[Playback对比] targetPose=%1")
			.arg(fmtPos(targetPose.x, targetPose.y, targetPose.z)));

	if (!tcpLinkName.isEmpty() && linkWorldByName.contains(tcpLinkName))
	{
		const osg::Vec3d t = linkWorldByName.value(tcpLinkName).getTrans();
		m_runInfoPage->appendInfo(
			QStringLiteral("[Playback对比] FK(tcpLink=%1)=%2, errMm=%3")
				.arg(tcpLinkName)
				.arg(fmtPos(t.x(), t.y(), t.z()))
				.arg(distToTarget(t.x(), t.y(), t.z()), 0, 'f', 6));
	}
	else
	{
		m_runInfoPage->appendInfo(
			QStringLiteral("[Playback对比] FK(tcpLink=%1)不可用")
				.arg(tcpLinkName.isEmpty() ? QStringLiteral("<empty>") : tcpLinkName));
	}

	QString primaryLink;
	(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, primaryLink, nullptr);
	if (!primaryLink.isEmpty() && linkWorldByName.contains(primaryLink))
	{
		const osg::Vec3d t = linkWorldByName.value(primaryLink).getTrans();
		m_runInfoPage->appendInfo(
			QStringLiteral("[Playback对比] FK(primaryLink=%1)=%2, errMm=%3")
				.arg(primaryLink)
				.arg(fmtPos(t.x(), t.y(), t.z()))
				.arg(distToTarget(t.x(), t.y(), t.z()), 0, 'f', 6));
	}

	const QStringList joints = doc->robotRevoluteJointNames();
	if (!joints.isEmpty())
	{
		const QString lastJoint = joints.back();
		if (osg::MatrixTransform* jointMt = doc->robotJointMatrixTransform(lastJoint))
		{
			osg::Matrixd jointWorld;
			if (matrixFromNodeWorld(jointMt, jointWorld))
			{
				osg::Matrixd robotRootWorld;
				robotRootWorld.makeIdentity();
				const QString robotRootId = doc->robotSceneBackendId();
				if (!robotRootId.isEmpty())
				{
					osg::Matrixd m;
					if (osg->getBackendRootWorldMatrix(robotRootId.toStdString(), m))
					{
						robotRootWorld = m;
					}
				}
				const osg::Matrixd jointLocal = osg::Matrixd::inverse(robotRootWorld) * jointWorld;
				const osg::Vec3d jt = jointLocal.getTrans();
				m_runInfoPage->appendInfo(
					QStringLiteral("[Playback对比] Hierarchy(lastJoint=%1)=%2, errMm=%3")
						.arg(lastJoint)
						.arg(fmtPos(jt.x(), jt.y(), jt.z()))
						.arg(distToTarget(jt.x(), jt.y(), jt.z()), 0, 'f', 6));
			}
		}
	}
}

void MainWindow::onRobotSimulationTick()
{
	if (!m_robotProgramExecutor.isRunning())
	{
		m_robotSimTimer.stop();
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	IRobotBackendPoseSink* poseSink = doc ? doc->sceneFacade().poseSink() : nullptr;
	const RobotInstructionPlaybackTickResult r = m_robotProgramExecutor.tick(doc, poseSink);
	m_aggregatedJointAnglesRad = m_robotProgramExecutor.jointAnglesRad();
	if (doc && osg)
	{
		doc->requestFollowSolveForced();
		runBackendFollowSolveAndSync(*doc, *osg);
	}
	switch (r)
	{
	case RobotInstructionPlaybackTickResult::Continue:
		break;
	case RobotInstructionPlaybackTickResult::Finished:
		logPlaybackFrameComparison(m_robotProgramExecutor.jointAnglesRad());
		stopRobotSimulation();
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(
				i18n(QStringLiteral("Simulation finished."), QStringLiteral("仿真已结束。")));
		}
		break;
	case RobotInstructionPlaybackTickResult::Aborted:
		stopRobotSimulation();
		break;
	}
}
