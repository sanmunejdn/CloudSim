/// @file RobotSceneKinematics.cpp
/// @brief ROBOT_KINEMATICS_DEBUG：0 关，1 紧凑，2/full 全矩阵

#include "RobotSceneKinematics.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "MeshBackendData.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPerLinkKinematicsApply.h"
#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include <QByteArray>
#include <QHash>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include <osg/Matrixd>

namespace
{
std::string qToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}

bool getBackendRootWorldOsg(IRobotBackendPoseSink* sink, const std::string& backendId, osg::Matrixd& outWorld)
{
	if (!sink)
	{
		return false;
	}
	cloudsim::core::Mat4 mat;
	if (!sink->getBackendRootWorldMatrix(backendId, mat))
	{
		return false;
	}
	outWorld = RobotSceneKinematics::osgMatrixFromCoreMat4(mat);
	return true;
}

void setBackendRootWorldOsg(IRobotBackendPoseSink* sink, const std::string& backendId, const osg::Matrixd& worldMat)
{
	if (!sink)
	{
		return;
	}
	sink->setBackendRootWorldMatrixFromWorld(backendId, RobotSceneKinematics::coreMat4FromOsgMatrix(worldMat));
}

/// ROBOT_KINEMATICS_DEBUG：0 关，1 紧凑，2/full 全矩阵
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
	o << " T(" << std::setprecision(6) << t.x() << ',' << t.y() << ',' << t.z() << ") quat(" << r.x() << ',' << r.y()
	  << ',' << r.z() << ',' << r.w() << ") scale(" << s.x() << ',' << s.y() << ',' << s.z() << ')';
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
	return RobotMatrixOsg::backendColMajorFromMatrix(m);
}

/// When link meshes are parented in OSG, \ref IRobotBackendPoseSink::setBackendRootWorldMatrixFromWorld uses each
/// node's current parent world matrix. Updates must run **parent link before child link**; arbitrary QHash iteration
/// was valid only while every outer PAT hung directly under the same scene group (identical parent world).
} // namespace

namespace RobotSceneKinematics
{
void resolveRobotLinkUpdateOrder(BackendDataManager* mgr, const QHash<QString, QString>& linkNameToBackendId,
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

bool applyJointAnglesViaLinkBackends(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, BackendDataManager& mgr,
									 const QVector<double>& anglesRad, const RobotPerLinkKinematicsSlice& slice)
{
	if (!doc || !osg)
	{
		return false;
	}
	const QHash<QString, QString>& linkToId = slice.linkNameToBackendId;
	if (linkToId.isEmpty())
	{
		return false;
	}
	const QString urdfPath = slice.urdfAbsolutePath;
	if (urdfPath.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatricesViaKinematicCore(urdfPath, anglesRad, Tq, &fkErr,
																   slice.meshVerticesInLinkFrame))
	{
		RunLogger::warn(qToUtf8Std(
			QStringLiteral("RobotSceneKinematics::applyJointAnglesViaLinkBackends FK failed: %1").arg(fkErr)));
		return false;
	}
	return RobotPerLinkKinematicsApply::applyLinkWorldFromCoreFk(osg, mgr, slice, Tq);
}

bool computeBasePlacementFromAnchorLinkWorld(const RobotPerLinkKinematicsSlice& slice,
											 const QString& anchorLinkBackendId, const QVector<double>& jointAnglesRad,
											 const osg::Matrixd& anchorLinkWorld, osg::Matrixd& outBasePlacementWorld)
{
	if (anchorLinkBackendId.isEmpty() || slice.urdfAbsolutePath.isEmpty())
	{
		return false;
	}
	QString anchorLinkName;
	for (auto it = slice.linkNameToBackendId.constBegin(); it != slice.linkNameToBackendId.constEnd(); ++it)
	{
		if (it.value() == anchorLinkBackendId)
		{
			anchorLinkName = it.key();
			break;
		}
	}
	if (anchorLinkName.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(slice.urdfAbsolutePath, jointAnglesRad, Tq, &fkErr,
												   slice.meshVerticesInLinkFrame))
	{
		return false;
	}
	const auto t0It = slice.fkMeshWorldT0.constFind(anchorLinkName);
	const auto m0It = slice.outerWorldAtBindByBackendId.constFind(anchorLinkBackendId);
	if (!Tq.contains(anchorLinkName) || t0It == slice.fkMeshWorldT0.constEnd() ||
		m0It == slice.outerWorldAtBindByBackendId.constEnd())
	{
		return false;
	}
	outBasePlacementWorld = osg::Matrixd::inverse(Tq[anchorLinkName]) * t0It.value() *
							osg::Matrixd::inverse(m0It.value()) * anchorLinkWorld;
	return true;
}

bool applyPerLinkRobotBasePlacement(IRobotBackendPoseSink* osg, BackendDataManager& mgr,
									const RobotPerLinkKinematicsSlice& slice, const QVector<double>& jointAnglesRad,
									const osg::Matrixd& basePlacementWorld)
{
	if (!osg || slice.linkNameToBackendId.isEmpty() || slice.urdfAbsolutePath.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(slice.urdfAbsolutePath, jointAnglesRad, Tq, &fkErr,
												   slice.meshVerticesInLinkFrame))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("applyPerLinkRobotBasePlacement FK failed: %1").arg(fkErr)));
		return false;
	}
	const QHash<QString, osg::Matrixd>& T0 = slice.fkMeshWorldT0;
	const QHash<QString, osg::Matrixd>& m0bind = slice.outerWorldAtBindByBackendId;

	QVector<QString> linkOrder;
	resolveRobotLinkUpdateOrder(&mgr, slice.linkNameToBackendId, linkOrder);

	bool any = false;
	for (const QString& linkName : linkOrder)
	{
		const QString backendIdStr = slice.linkNameToBackendId.value(linkName);
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
		const osg::Matrixd Mnew =
			m0It.value() * osg::Matrixd::inverse(T0[linkName]) * Tq[linkName] * basePlacementWorld;
		meshPtr->setWorldMatrix(osgMatToBackendColMajor(Mnew));
		if (osg)
		{
			osg->syncRobotMeshBackendPoseAfterKinematics(*meshPtr);
		}
		any = true;
	}
	return any;
}

bool applyJointAnglesForInstance(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, int instanceIndex,
								 const QVector<double>& localAnglesRad, QVector<double>& aggregatedAnglesRad)
{
	if (!doc || instanceIndex < 0 || instanceIndex >= doc->robotKinematicInstanceCount())
	{
		return false;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instanceIndex);
	if (localAnglesRad.size() != nj)
	{
		return false;
	}
	const int total = doc->robotRevoluteJointNames().size();
	if (aggregatedAnglesRad.size() != total)
	{
		aggregatedAnglesRad.resize(total);
		aggregatedAnglesRad.fill(0.0);
	}
	int offset = 0;
	for (int i = 0; i < instanceIndex; ++i)
	{
		offset += doc->robotRevoluteJointCountForInstance(i);
	}
	for (int j = 0; j < nj; ++j)
	{
		aggregatedAnglesRad[offset + j] = localAnglesRad[j];
	}
	return applyJointAnglesFromDocument(doc, osg, aggregatedAnglesRad);
}

bool applyJointAnglesFromDocument(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
								  const QVector<double>& anglesRad)
{
	if (!doc || !doc->hasRobotSimulationContext())
	{
		RunLogger::warn(
			"RobotSceneKinematics::applyJointAnglesFromDocument invalid document or missing simulation context.");
		return false;
	}

	BackendDataManager* mgr = doc->robotBackendManagerForKinematics();
	const int nInst = doc->robotKinematicInstanceCount();
	if (nInst > 0)
	{
		int offset = 0;
		bool applied = false;
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
				RunLogger::warn(
					"RobotSceneKinematics::applyJointAnglesFromDocument angle vector size does not match joint count.");
				return false;
			}
			const QVector<double> jointSlice = anglesRad.mid(offset, nj);
			offset += nj;
			if (urdfPath.isEmpty())
			{
				continue;
			}

			cloudsim::core::RobotPerLinkKinematicsSliceDto plDto;
			if (doc->robotPerLinkKinematicsForInstance(i, plDto))
			{
				if (!mgr || !osg)
				{
					RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument per-link robot requires "
									"BackendDataManager and OSG.");
					return false;
				}
				const RobotPerLinkKinematicsSlice plSlice = robotPerLinkSliceFromDto(plDto);
				if (!applyJointAnglesViaLinkBackends(doc, osg, *mgr, jointSlice, plSlice))
				{
					return false;
				}
				applied = true;
				continue;
			}

			if (!osg)
			{
				continue;
			}
			QHash<QString, osg::Matrixd> newJointMatrices;
			if (!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, jointSlice, newJointMatrices, nullptr))
			{
				RunLogger::warn(qToUtf8Std(
					QStringLiteral("RobotSceneKinematics: computeJointTransformMatrices failed for URDF '%1'")
						.arg(urdfPath)));
				return false;
			}
			const QString prefix = doc->robotJointKeyPrefixForInstance(i);
			QHash<QString, cloudsim::core::Mat4> localByKey;
			for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
			{
				localByKey.insert(prefix + it.key(), coreMat4FromOsgMatrix(it.value()));
			}
			if (!localByKey.isEmpty())
			{
				doc->applyRobotJointLocalMatrices(localByKey);
				applied = true;
			}
		}
		if (applied)
		{
			doc->notifyRobotKinematicsAppliedToScene();
			return true;
		}
		if (nInst > 0)
		{
			RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument no joint matrix written for "
							"multi-instance context.");
			return false;
		}
	}

	if (!osg)
	{
		return false;
	}

	// nInst==0（例如仅传统烘焙数据）：回退到旧单 URDF 路径（无前缀键）
	const QString urdfPath = doc->robotUrdfAbsolutePath();
	QHash<QString, osg::Matrixd> newJointMatrices;
	if (urdfPath.isEmpty() ||
		!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, anglesRad, newJointMatrices, nullptr))
	{
		RunLogger::warn(qToUtf8Std(
			QStringLiteral("RobotSceneKinematics: fallback computeJointTransformMatrices failed for URDF '%1'")
				.arg(urdfPath)));
		return false;
	}

	bool hasJointTransforms = false;
	const QStringList jointNames = doc->robotRevoluteJointNames();
	for (const QString& jointName : jointNames)
	{
		if (doc->hasRobotJointLocalMatrix(jointName))
		{
			hasJointTransforms = true;
			break;
		}
	}

	if (hasJointTransforms)
	{
		QHash<QString, cloudsim::core::Mat4> localByKey;
		for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
		{
			localByKey.insert(it.key(), coreMat4FromOsgMatrix(it.value()));
		}
		doc->applyRobotJointLocalMatrices(localByKey);
		doc->notifyRobotKinematicsAppliedToScene();
		return true;
	}

	// 回退到旧架构（传统烘焙法）
	if (!doc->hasRobotKinematicsBind())
	{
		RunLogger::warn("RobotSceneKinematics: fallback path requires RobotKinematicsBind but it is missing.");
		return false;
	}

	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, anglesRad, Tq, nullptr,
												   doc->robotUrdfMeshVerticesInLinkFrame()))
	{
		RunLogger::warn(qToUtf8Std(
			QStringLiteral("RobotSceneKinematics: computeMeshWorldMatrices failed for URDF '%1'").arg(urdfPath)));
		return false;
	}
	const QHash<QString, cloudsim::core::Mat4> T0Dto = doc->robotFkMeshWorldT0();
	const QHash<QString, cloudsim::core::Mat4> M0Dto = doc->robotOuterWorldAtBind();
	QHash<QString, osg::Matrixd> T0;
	QHash<QString, osg::Matrixd> M0;
	for (auto it = T0Dto.constBegin(); it != T0Dto.constEnd(); ++it)
	{
		T0.insert(it.key(), osgMatrixFromCoreMat4(it.value()));
	}
	for (auto it = M0Dto.constBegin(); it != M0Dto.constEnd(); ++it)
	{
		M0.insert(it.key(), osgMatrixFromCoreMat4(it.value()));
	}
	const QHash<QString, QString>& linkToId = doc->robotLinkNameToBackendId();
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
		const osg::Matrixd Mnew = m0It.value() * osg::Matrixd::inverse(T0[linkName]) * Tq[linkName];
		if (mgr)
		{
			if (const auto meshPtr = std::dynamic_pointer_cast<MeshBackendData>(mgr->getData(backendId.toStdString())))
			{
				meshPtr->setWorldMatrix(osgMatToBackendColMajor(Mnew));
				if (osg)
				{
					osg->syncRobotMeshBackendPoseAfterKinematics(*meshPtr);
				}
			}
		}
		else
		{
			setBackendRootWorldOsg(osg, backendId.toStdString(), Mnew);
		}
	}
	doc->notifyRobotKinematicsAppliedToScene();
	return true;
}

void applyMeshWorldMatricesRelativeToBind(IRobotBackendPoseSink* osg,
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
		setBackendRootWorldOsg(osg, backendId.toStdString(), Mnew);
	}
}

} // namespace RobotSceneKinematics
