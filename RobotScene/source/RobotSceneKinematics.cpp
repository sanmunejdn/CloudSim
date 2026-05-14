#include "RobotSceneKinematics.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "MeshBackendData.h"
#include "UrdfRobotLoader.h"
#include "RunLogger.h"

#include <iomanip>
#include <memory>
#include <sstream>
#include <osg/Matrixd>
#include <osg/MatrixTransform>

#include <QByteArray>
#include <QHash>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <algorithm>
#include <vector>

namespace
{
std::string qToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}

/// 0=off, 1=compact (translation + quat + order), 2=full 4x4 matrices. Set \c ROBOT_KINEMATICS_DEBUG=1 or 2/full.
int robotKinematicsDebugLevel()
{
	const QByteArray v = qgetenv("ROBOT_KINEMATICS_DEBUG");
	if (v.isEmpty() || v == "0")
	{
		return 0;
	}
	if (v == "2" || qstricmp(v.constData(), "full") == 0)
	{
		return 2;
	}
	return 1;
}

void appendOsgMatrixBlock(std::ostringstream& o, const char* name, const osg::Matrixd& m, int level)
{
	o << ' ' << name << ':';
	osg::Vec3d t;
	osg::Quat r;
	osg::Vec3d s;
	osg::Quat so;
	m.decompose(t, r, s, so);
	o << " T(" << std::setprecision(6) << t.x() << ',' << t.y() << ',' << t.z() << ") quat(" << r.x() << ',' << r.y() << ',' << r.z()
	  << ',' << r.w() << ") scale(" << s.x() << ',' << s.y() << ',' << s.z() << ')';
	if (level >= 2)
	{
		o << " mat4 row-major:\n";
		o << std::setprecision(9);
		for (int row = 0; row < 4; ++row)
		{
			o << "    [";
			for (int col = 0; col < 4; ++col)
			{
				o << (col ? " " : "") << m(row, col);
			}
			o << "]\n";
		}
	}
}

std::string joinUtf8List(const QVector<QString>& v)
{
	std::ostringstream o;
	for (int i = 0; i < v.size(); ++i)
	{
		if (i)
		{
			o << ',';
		}
		o << qToUtf8Std(v[i]);
	}
	return o.str();
}

BackendMat4 osgMatToBackendColMajor(const osg::Matrixd& m)
{
	BackendMat4 b{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			b.v[c * 4 + r] = m(r, c);
		}
	}
	return b;
}

BackendVec3 meshBoundsCenterMm(const MeshBackendData& mesh)
{
	const BackendBoundingBox bb = mesh.geometryBounds();
	if (!bb.valid)
	{
		return BackendVec3{};
	}
	BackendVec3 c{};
	c.x = (bb.min.x + bb.max.x) * 0.5;
	c.y = (bb.min.y + bb.max.y) * 0.5;
	c.z = (bb.min.z + bb.max.z) * 0.5;
	return c;
}

/// When link meshes are parented in OSG, \ref IRobotBackendPoseSink::setBackendRootWorldMatrixFromWorld uses each
/// node's current parent world matrix. Updates must run **parent link before child link**; arbitrary QHash iteration
/// was valid only while every outer PAT hung directly under the same scene group (identical parent world).
void resolveRobotLinkUpdateOrder(
	BackendDataManager* mgr,
	const QHash<QString, QString>& linkNameToBackendId,
	QVector<QString>& outLinkNames)
{
	outLinkNames.clear();
	if (linkNameToBackendId.isEmpty())
	{
		return;
	}

	outLinkNames.reserve(linkNameToBackendId.size());
	if (!mgr)
	{
		outLinkNames = linkNameToBackendId.keys().toVector();
		std::sort(outLinkNames.begin(), outLinkNames.end());
		return;
	}

	QHash<QString, QString> backendToLinkName;
	QSet<QString> robotBackendIds;
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		backendToLinkName.insert(it.value(), it.key());
		robotBackendIds.insert(it.value());
	}

	QHash<QString, QVector<QString>> childrenByParentLink;
	QHash<QString, int> indegree;
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		indegree.insert(it.key(), 0);
	}
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& bid = it.value();
		const std::vector<std::string> ps = mgr->parentsOf(bid.toStdString());
		if (ps.empty())
		{
			continue;
		}
		const QString parentBid = QString::fromStdString(ps.front());
		if (!robotBackendIds.contains(parentBid))
		{
			continue;
		}
		const QString parentLink = backendToLinkName.value(parentBid);
		if (parentLink.isEmpty() || parentLink == linkName)
		{
			continue;
		}
		childrenByParentLink[parentLink].push_back(linkName);
		indegree[linkName] = indegree[linkName] + 1;
	}

	QVector<QString> queue;
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		if (indegree.value(it.key(), 0) == 0)
		{
			queue.push_back(it.key());
		}
	}
	std::sort(queue.begin(), queue.end());

	int qh = 0;
	while (qh < queue.size())
	{
		const QString u = queue[qh++];
		outLinkNames.push_back(u);
		const QVector<QString> ch = childrenByParentLink.value(u);
		for (const QString& v : ch)
		{
			int& d = indegree[v];
			--d;
			if (d == 0)
			{
				queue.push_back(v);
			}
		}
	}

	if (outLinkNames.size() != linkNameToBackendId.size())
	{
		QSet<QString> seen;
		for (const QString& s : outLinkNames)
		{
			seen.insert(s);
		}
		QVector<QString> tail;
		for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
		{
			if (!seen.contains(it.key()))
			{
				tail.push_back(it.key());
			}
		}
		std::sort(tail.begin(), tail.end());
		outLinkNames += tail;
	}
}
struct PendingLinkBackendPose
{
	std::shared_ptr<MeshBackendData> mesh;
	BackendVec3 pose{};
	BackendVec3 euler{};
};

} // namespace

namespace RobotSceneKinematics
{

bool applyJointAnglesViaLinkBackends(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	BackendDataManager& mgr,
	const QVector<double>& anglesRad)
{
	if (!doc || !osg)
	{
		return false;
	}
	const QHash<QString, QString>& linkToId = doc->robotLinkNameToBackendId();
	if (linkToId.isEmpty())
	{
		return false;
	}
	if (doc->robotKinematicInstanceCount() != 1)
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesViaLinkBackends: only a single robot instance is supported.");
		return false;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(0);
	if (nj < 0 || anglesRad.size() < nj)
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesViaLinkBackends: invalid joint vector size.");
		return false;
	}
	const QVector<double> slice = anglesRad.mid(0, nj);
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(0);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, slice, Tq, &fkErr, doc->robotUrdfMeshVerticesInLinkFrame()))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics::applyJointAnglesViaLinkBackends FK failed: %1").arg(fkErr)));
		return false;
	}
	const QHash<QString, osg::Matrixd>& T0 = doc->robotFkMeshWorldT0();
	const QHash<QString, osg::Matrixd>& m0bind = doc->robotOuterWorldAtBind();

	QVector<QString> linkOrder;
	resolveRobotLinkUpdateOrder(&mgr, linkToId, linkOrder);

	const int dbg = robotKinematicsDebugLevel();
	if (dbg > 0)
	{
		std::ostringstream hdr;
		hdr << "[RobotKinematicsDBG] applyJointAnglesViaLinkBackends urdf=" << qToUtf8Std(urdfPath) << " nj=" << nj
			<< " linkUpdateOrder=[" << joinUtf8List(linkOrder) << "]\n";
		hdr << "  jointAnglesRad (index : rad : deg):";
		const QStringList jnames = doc->robotRevoluteJointNames();
		for (int ji = 0; ji < nj; ++ji)
		{
			const double rad = slice[ji];
			const double deg = rad * (180.0 / 3.14159265358979323846);
			hdr << "\n    [" << ji << "] " << std::setprecision(9) << rad << " rad (" << deg << " deg)";
			if (ji < jnames.size())
			{
				hdr << "  jointKey=" << qToUtf8Std(jnames[ji]);
			}
		}
		RunLogger::info(hdr.str());
	}

	std::vector<PendingLinkBackendPose> pendingBackendPose;
	pendingBackendPose.reserve(static_cast<size_t>(linkOrder.size()));

	for (const QString& linkName : linkOrder)
	{
		const QString backendIdStr = linkToId.value(linkName);
		if (backendIdStr.isEmpty())
		{
			continue;
		}
		const std::string bid = backendIdStr.toStdString();
		const auto meshPtr = std::dynamic_pointer_cast<MeshBackendData>(mgr.getData(bid));
		if (!meshPtr)
		{
			continue;
		}
		if (!Tq.contains(linkName) || !T0.contains(linkName))
		{
			continue;
		}
		const auto m0It = m0bind.constFind(backendIdStr);
		if (m0It == m0bind.cend())
		{
			continue;
		}
		const osg::Matrixd& T0m = T0[linkName];
		const osg::Matrixd& Tqm = Tq[linkName];
		const osg::Matrixd& M0m = m0It.value();
		// UrdfRobotLoader::mat4ToOsg stores C(M)=M^T with C(A*B)=C(B)*C(A). Internal mesh world is Tq*T0^-1*M0;
		// in OSG multiply order that is M0_osg * inv(T0_osg) * Tq_osg.
		const osg::Matrixd Mnew =
			M0m * osg::Matrixd::inverse(T0m) * Tqm;

		if (dbg > 0)
		{
			std::ostringstream line;
			line << "[RobotKinematicsDBG] link=" << qToUtf8Std(linkName) << " backendId=" << bid;
			const std::vector<std::string> ps = mgr.parentsOf(bid);
			if (!ps.empty())
			{
				line << " parentBackendId=" << ps.front();
			}
			else
			{
				line << " parentBackendId=<none>";
			}
			osg::Matrixd parentWorldBefore;
			if (!ps.empty() && osg->getBackendRootWorldMatrix(ps.front(), parentWorldBefore))
			{
				appendOsgMatrixBlock(line, "parentWorld(OSG,before)", parentWorldBefore, dbg);
			}
			appendOsgMatrixBlock(line, "T0(bindFK_meshWorld)", T0m, dbg);
			appendOsgMatrixBlock(line, "Tq(currentFK_meshWorld)", Tqm, dbg);
			appendOsgMatrixBlock(line, "M0(outerWorldAtBind)", M0m, dbg);
			appendOsgMatrixBlock(line, "Mnew(target_outerWorld)", Mnew, dbg);
			RunLogger::info(line.str());
		}

		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		BackendVec3 center{};
		if (osg->tryGetBackendModelCenterMm(bid, cx, cy, cz))
		{
			center.x = cx;
			center.y = cy;
			center.z = cz;
		}
		else
		{
			center = meshBoundsCenterMm(*meshPtr);
		}

		osg->setBackendRootWorldMatrixFromWorld(bid, Mnew);

		if (dbg > 0)
		{
			osg::Matrixd worldAfter{};
			if (osg->getBackendRootWorldMatrix(bid, worldAfter))
			{
				std::ostringstream ver;
				ver << "[RobotKinematicsDBG] link=" << qToUtf8Std(linkName) << " backendId=" << bid;
				appendOsgMatrixBlock(ver, "outerWorld(OSG,after)", worldAfter, dbg);
				RunLogger::info(ver.str());
			}
			else
			{
				RunLogger::info(std::string("[RobotKinematicsDBG] link=") + qToUtf8Std(linkName)
					+ " getBackendRootWorldMatrix failed after setBackendRootWorldMatrixFromWorld");
			}
		}

		const BackendMat4 Wb = osgMatToBackendColMajor(Mnew);
		BackendVec3 pose{};
		BackendVec3 euler{};
		backend_pose_euler_from_world_mat(Wb, center, pose, euler);
		pendingBackendPose.push_back(PendingLinkBackendPose{meshPtr, pose, euler});
	}

	for (const PendingLinkBackendPose& pb : pendingBackendPose)
	{
		if (!pb.mesh)
		{
			continue;
		}
		pb.mesh->setPose(pb.pose);
		pb.mesh->setRotation(pb.euler);
	}
	doc->notifyRobotKinematicsAppliedToScene();
	if (dbg > 0)
	{
		RunLogger::info("[RobotKinematicsDBG] applyJointAnglesViaLinkBackends complete.");
		RunLogger::flush();
	}
	return true;
}

bool applyJointAnglesFromDocument(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad)
{
	if (!doc || !doc->hasRobotSimulationContext())
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument invalid document or missing simulation context.");
		return false;
	}

	if (!doc->robotLinkNameToBackendId().isEmpty())
	{
		if (BackendDataManager* mgr = doc->robotBackendManagerForKinematics())
		{
			return applyJointAnglesViaLinkBackends(doc, osg, *mgr, anglesRad);
		}
		RunLogger::warn(
			"RobotSceneKinematics::applyJointAnglesFromDocument per-link robot requires BackendDataManager access.");
		return false;
	}

	if (!osg)
	{
		return false;
	}
	const int nInst = doc->robotKinematicInstanceCount();
	int offset = 0;
	bool appliedHierarchy = false;
	for (int i = 0; i < nInst; ++i)
	{
		const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(i);
		const int nj = doc->robotRevoluteJointCountForInstance(i);
		if (nj <= 0)
		{
			continue;
		}
		if (offset + nj > anglesRad.size())
		{
			RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument angle vector size does not match joint count.");
			return false;
		}
		const QVector<double> slice = anglesRad.mid(offset, nj);
		offset += nj;
		if (urdfPath.isEmpty())
		{
			continue;
		}

		QHash<QString, osg::Matrixd> newJointMatrices;
		if (!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, slice, newJointMatrices, nullptr))
		{
			RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: computeJointTransformMatrices failed for URDF '%1'").arg(urdfPath)));
			return false;
		}

		const QString prefix = doc->robotJointKeyPrefixForInstance(i);
		for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
		{
			const QString key = prefix + it.key();
			osg::MatrixTransform* jointMT = doc->robotJointMatrixTransform(key);
			if (jointMT)
			{
				jointMT->setMatrix(it.value());
				appliedHierarchy = true;
			}
		}
	}

	if (appliedHierarchy)
	{
		return true;
	}
	// 已按多实例语义遍历过但未写入任何 MatrixTransform：视为失败（避免用错误的全向量再算一遍 FK）
	if (nInst > 0)
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument no joint matrix written for multi-instance context.");
		return false;
	}

	// nInst==0（例如仅传统烘焙数据）：回退到旧单 URDF 路径（无前缀键）
	const QString urdfPath = doc->robotUrdfAbsolutePath();
	QHash<QString, osg::Matrixd> newJointMatrices;
	if (urdfPath.isEmpty() ||
		!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, anglesRad, newJointMatrices, nullptr))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: fallback computeJointTransformMatrices failed for URDF '%1'").arg(urdfPath)));
		return false;
	}

	bool hasJointTransforms = false;
	const QStringList jointNames = doc->robotRevoluteJointNames();
	for (const QString& jointName : jointNames)
	{
		if (doc->robotJointMatrixTransform(jointName))
		{
			hasJointTransforms = true;
			break;
		}
	}

	if (hasJointTransforms)
	{
		for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
		{
			osg::MatrixTransform* jointMT = doc->robotJointMatrixTransform(it.key());
			if (jointMT)
			{
				jointMT->setMatrix(it.value());
			}
		}
		return true;
	}

	// 【中文】回退到旧架构（传统烘焙法）
	if (!doc->hasRobotKinematicsBind())
	{
		RunLogger::warn("RobotSceneKinematics: fallback path requires RobotKinematicsBind but it is missing.");
		return false;
	}

	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, anglesRad, Tq, nullptr, doc->robotUrdfMeshVerticesInLinkFrame()))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: computeMeshWorldMatrices failed for URDF '%1'").arg(urdfPath)));
		return false;
	}
	const QHash<QString, osg::Matrixd>& T0 = doc->robotFkMeshWorldT0();
	const QHash<QString, osg::Matrixd>& M0 = doc->robotOuterWorldAtBind();
	const QHash<QString, QString>& linkToId = doc->robotLinkNameToBackendId();
	BackendDataManager* mgr = doc->robotBackendManagerForKinematics();
	QVector<QString> linkOrder;
	resolveRobotLinkUpdateOrder(mgr, linkToId, linkOrder);
	for (const QString& linkName : linkOrder)
	{
		const QString backendId = linkToId.value(linkName);
		if (backendId.isEmpty())
		{
			continue;
		}
		if (!Tq.contains(linkName) || !T0.contains(linkName))
		{
			continue;
		}
		const auto m0It = M0.find(backendId);
		if (m0It == M0.end())
		{
			continue;
		}
		const osg::Matrixd Mnew =
			m0It.value() * osg::Matrixd::inverse(T0[linkName]) * Tq[linkName];
		osg->setBackendRootWorldMatrixFromWorld(backendId.toStdString(), Mnew);
	}
	return true;
}

void applyMeshWorldMatricesRelativeToBind(
	IRobotBackendPoseSink* osg,
	const QHash<QString, osg::Matrixd>& meshWorldByLink,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, QString>& linkNameToBackendId,
	const std::unordered_map<std::string, osg::Matrixd>& outerWorldAtBind)
{
	if (!osg)
	{
		return;
	}
	QVector<QString> linkOrder;
	resolveRobotLinkUpdateOrder(nullptr, linkNameToBackendId, linkOrder);
	for (const QString& linkName : linkOrder)
	{
		const QString backendId = linkNameToBackendId.value(linkName);
		if (backendId.isEmpty())
		{
			continue;
		}
		if (!meshWorldByLink.contains(linkName) || !fkMeshWorldT0.contains(linkName))
		{
			continue;
		}
		const auto m0It = outerWorldAtBind.find(backendId.toStdString());
		if (m0It == outerWorldAtBind.end())
		{
			continue;
		}
		const osg::Matrixd& T0 = fkMeshWorldT0[linkName];
		const osg::Matrixd Trel = osg::Matrixd::inverse(T0) * meshWorldByLink[linkName];
		const osg::Matrixd Mnew = m0It->second * Trel;
		osg->setBackendRootWorldMatrixFromWorld(backendId.toStdString(), Mnew);
	}
}

} // namespace RobotSceneKinematics
