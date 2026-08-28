/// @file RobotProjectKinematicsRestore.cpp
/// @brief 工程运动学恢复

#include "RobotProjectKinematicsRestore.h"

#include "BackendProjectObjectIo.h"
#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "CoreTypes.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotUrdfImportContext.h"
#include "MeshBackendData.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "UrdfRobotLoader.h"

#include "RunLogger.h"

#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>

#include <json.hpp>
#include <osg/MatrixTransform>
#include <osg/Matrixd>

namespace cloudsim::host
{
namespace
{
void collectRobotLinkIdsFromKinematicsObject(const QJsonObject& rkObj, QSet<QString>& outIds)
{
	if (rkObj.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink"))
	{
		return;
	}
	const QJsonObject linksHint = rkObj.value(QStringLiteral("links")).toObject();
	for (auto it = linksHint.constBegin(); it != linksHint.constEnd(); ++it)
	{
		outIds.insert(it.value().toString());
	}
}

void collectRobotKinematicsMetaFromObject(const QJsonObject& rkObj, QSet<QString>& outLinkIds,
										  QSet<QString>& outSceneRoots,
										  QHash<QString, RobotLinkUrdfReloadHint>& outReloadHints)
{
	if (rkObj.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink"))
	{
		return;
	}
	const QString urdf = rkObj.value(QStringLiteral("urdf")).toString();
	const QString sceneRoot = rkObj.value(QStringLiteral("sceneRootBackendId")).toString();
	if (!sceneRoot.isEmpty())
	{
		outSceneRoots.insert(sceneRoot);
	}
	const QJsonObject linksJ = rkObj.value(QStringLiteral("links")).toObject();
	for (auto it = linksJ.constBegin(); it != linksJ.constEnd(); ++it)
	{
		const QString backendId = it.value().toString();
		if (backendId.isEmpty())
		{
			continue;
		}
		outLinkIds.insert(backendId);
		RobotLinkUrdfReloadHint hint;
		hint.urdfPath = urdf;
		hint.linkName = it.key();
		outReloadHints.insert(backendId, hint);
	}
}

} // namespace

QSet<QString> collectRobotLinkMeshBackendIds(const QJsonObject& projectRoot)
{
	QSet<QString> ids;
	const QJsonArray instances = projectRoot.value(QStringLiteral("robotKinematicsInstances")).toArray();
	for (const QJsonValue& rv : instances)
	{
		if (rv.isObject())
		{
			collectRobotLinkIdsFromKinematicsObject(rv.toObject(), ids);
		}
	}
	const QJsonObject legacy = projectRoot.value(QStringLiteral("robotKinematics")).toObject();
	if (!legacy.isEmpty())
	{
		collectRobotLinkIdsFromKinematicsObject(legacy, ids);
	}
	return ids;
}

QSet<QString> collectRobotSceneRootBackendIds(const QJsonObject& projectRoot)
{
	QSet<QString> roots;
	QSet<QString> unusedLinks;
	QHash<QString, RobotLinkUrdfReloadHint> unusedHints;
	const QJsonArray instances = projectRoot.value(QStringLiteral("robotKinematicsInstances")).toArray();
	for (const QJsonValue& rv : instances)
	{
		if (rv.isObject())
		{
			collectRobotKinematicsMetaFromObject(rv.toObject(), unusedLinks, roots, unusedHints);
		}
	}
	const QJsonObject legacy = projectRoot.value(QStringLiteral("robotKinematics")).toObject();
	if (!legacy.isEmpty())
	{
		collectRobotKinematicsMetaFromObject(legacy, unusedLinks, roots, unusedHints);
	}
	return roots;
}

QHash<QString, RobotLinkUrdfReloadHint> collectRobotLinkUrdfReloadHints(const QJsonObject& projectRoot)
{
	QHash<QString, RobotLinkUrdfReloadHint> hints;
	QSet<QString> unusedLinks;
	QSet<QString> unusedRoots;
	const QJsonArray instances = projectRoot.value(QStringLiteral("robotKinematicsInstances")).toArray();
	for (const QJsonValue& rv : instances)
	{
		if (rv.isObject())
		{
			collectRobotKinematicsMetaFromObject(rv.toObject(), unusedLinks, unusedRoots, hints);
		}
	}
	const QJsonObject legacy = projectRoot.value(QStringLiteral("robotKinematics")).toObject();
	if (!legacy.isEmpty())
	{
		collectRobotKinematicsMetaFromObject(legacy, unusedLinks, unusedRoots, hints);
	}
	return hints;
}

bool reloadRobotLinkMeshFromUrdfHint(MeshBackendData& mesh, const RobotLinkUrdfReloadHint& hint, QString* outError)
{
	if (mesh.hasGeometry())
	{
		return true;
	}
	if (hint.urdfPath.isEmpty() || hint.linkName.isEmpty() || !QFileInfo::exists(hint.urdfPath))
	{
		if (outError)
		{
			*outError = QStringLiteral("robot link reload: missing URDF or link name");
		}
		return false;
	}
	QHash<QString, QString> linkMeshes;
	QString rootLink;
	QString urdfErr;
	if (!UrdfRobotLoader::enumerateLinkVisualMeshes(hint.urdfPath, rootLink, linkMeshes, &urdfErr))
	{
		if (outError)
		{
			*outError = urdfErr.isEmpty() ? QStringLiteral("enumerateLinkVisualMeshes failed") : urdfErr;
		}
		return false;
	}
	const QString meshPath = linkMeshes.value(hint.linkName);
	if (meshPath.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("robot link reload: no visual mesh for '%1'").arg(hint.linkName);
		}
		return false;
	}
	std::string loadErr;
	if (!mesh.loadFromFile(meshPath.toStdString(), &loadErr))
	{
		if (outError)
		{
			*outError = loadErr.empty() ? QStringLiteral("loadFromFile failed") : QString::fromStdString(loadErr);
		}
		return false;
	}
	return true;
}

void reapplyAllRobotHierarchyFromProjectJson(DocumentHost& host, const QJsonObject& projectRoot)
{
	const auto applyOne = [&host](const QJsonObject& rk)
	{
		if (rk.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink"))
		{
			return;
		}
		const QString urdf = rk.value(QStringLiteral("urdf")).toString();
		const QString sceneRoot = rk.value(QStringLiteral("sceneRootBackendId")).toString();
		const QJsonObject linksJ = rk.value(QStringLiteral("links")).toObject();
		QHash<QString, QString> linkMap;
		for (auto it = linksJ.constBegin(); it != linksJ.constEnd(); ++it)
		{
			linkMap.insert(it.key(), it.value().toString());
		}
		if (sceneRoot.isEmpty() || linkMap.isEmpty())
		{
			return;
		}
		reapplyUrdfRobotHierarchyEdges(host.backend(), urdf, sceneRoot, linkMap);
	};
	for (const QJsonValue& rv : projectRoot.value(QStringLiteral("robotKinematicsInstances")).toArray())
	{
		if (rv.isObject())
		{
			applyOne(rv.toObject());
		}
	}
	const QJsonObject legacy = projectRoot.value(QStringLiteral("robotKinematics")).toObject();
	if (!legacy.isEmpty())
	{
		applyOne(legacy);
	}
	rebuildBackendParentIdMirror(host);
}

void reapplyUrdfRobotHierarchyEdges(BackendDataManager& backend, const QString& urdfPath,
									const QString& sceneRootBackendId, const QHash<QString, QString>& linkNameToBackendId)
{
	if (urdfPath.isEmpty() || sceneRootBackendId.isEmpty() || linkNameToBackendId.isEmpty())
	{
		return;
	}
	if (!backend.contains(sceneRootBackendId.toStdString()))
	{
		return;
	}
	QHash<QString, QString> urdfChildToParent;
	if (!UrdfRobotLoader::loadLinkChildToParentMap(urdfPath, urdfChildToParent, nullptr))
	{
		return;
	}
	const auto nearestMeshedAncestor = [&](const QString& linkName) -> QString
	{
		QString p = urdfChildToParent.value(linkName);
		while (!p.isEmpty() && !linkNameToBackendId.contains(p))
		{
			p = urdfChildToParent.value(p);
		}
		return p;
	};
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		const std::string childId = it.value().toStdString();
		if (backend.contains(childId))
		{
			backend.detachAllParents(childId);
		}
	}
	for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& childBid = it.value();
		const QString parentLink = nearestMeshedAncestor(linkName);
		const QString parentBid = parentLink.isEmpty() ? sceneRootBackendId : linkNameToBackendId.value(parentLink);
		if (parentBid.isEmpty())
		{
			continue;
		}
		const std::string parentStd = parentBid.toStdString();
		const std::string childStd = childBid.toStdString();
		if (backend.contains(parentStd) && backend.contains(childStd))
		{
			backend.attachChild(parentStd, childStd);
		}
	}
}

bool restorePerLinkRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx, const QJsonObject& rk,
												  QVector<double>& outAllJointAnglesRad, QString* outWarning)
{
	if (rk.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink"))
	{
		return false;
	}
	const QString urdf = rk.value(QStringLiteral("urdf")).toString();
	const QString sceneRoot = rk.value(QStringLiteral("sceneRootBackendId")).toString();
	const QString jointRoot = rk.value(QStringLiteral("jointPrefixRoot")).toString();
	const QString importKey = rk.value(QStringLiteral("importKey")).toString();
	const QJsonObject linksJ = rk.value(QStringLiteral("links")).toObject();
	QHash<QString, QString> linkMap;
	for (auto it = linksJ.constBegin(); it != linksJ.constEnd(); ++it)
	{
		linkMap.insert(it.key(), it.value().toString());
	}
	BackendDataManager& backend = ctx.urdfImportBackend();
	for (auto it = linkMap.constBegin(); it != linkMap.constEnd(); ++it)
	{
		if (!backend.contains(it.value().toStdString()))
		{
			return false;
		}
	}
	if (urdf.isEmpty() || !QFileInfo::exists(urdf) || sceneRoot.isEmpty() || jointRoot.isEmpty() || linkMap.isEmpty())
	{
		return false;
	}
	QStringList jn;
	QVector<double> lo;
	QVector<double> hi;
	(void)UrdfRobotLoader::loadRevoluteJointMeta(urdf, jn, lo, hi, nullptr);
	QVector<double> q0(jn.size(), 0.0);
	const QJsonArray savedJoints = rk.value(QStringLiteral("jointAnglesRad")).toArray();
	if (savedJoints.size() == jn.size())
	{
		for (int i = 0; i < jn.size(); ++i)
		{
			q0[i] = savedJoints.at(i).toDouble(0.0);
		}
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdf, q0, Tq, &fkErr, false))
	{
		if (outWarning)
		{
			*outWarning = QStringLiteral("robotKinematics: FK restore failed: %1").arg(fkErr);
		}
		return false;
	}
	QHash<QString, osg::Matrixd> fkT0;
	QHash<QString, osg::Matrixd> outer;
	auto maxMatAbsDiff = [](const osg::Matrixd& a, const osg::Matrixd& b) -> double
	{
		double m = 0.0;
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				const double d = std::abs(static_cast<double>(a(r, c)) - static_cast<double>(b(r, c)));
				if (d > m)
				{
					m = d;
				}
			}
		}
		return m;
	};
	IRobotBackendPoseSink* sink = ctx.urdfImportScenePoseSink();
	for (auto it = linkMap.constBegin(); it != linkMap.constEnd(); ++it)
	{
		const QString& lname = it.key();
		if (!Tq.contains(lname))
		{
			continue;
		}
		const osg::Matrixd& meshWorld0 = Tq[lname];
		fkT0.insert(lname, meshWorld0);
		const QString& bid = it.value();
		(void)sink;
		(void)maxMatAbsDiff;
		// 从已加载的 backend 读取 worldMatrix（project.json 恢复时已还原），
		// 而非用 identity；否则 applyJointAnglesViaLinkBackends 的公式
		// Mnew = M0 * inv(T0) * Tq * P 会使所有 link 坍缩到原点。
		const auto meshPtr = std::dynamic_pointer_cast<MeshBackendData>(backend.getData(bid.toStdString()));
		if (meshPtr)
		{
			outer.insert(bid, RobotMatrixOsg::matrixFromBackendColMajor(meshPtr->worldMatrix()));
		}
		else
		{
			outer.insert(bid, osg::Matrixd::identity());
		}
	}
	// 无 OSG 关节节点：perLink 模式靠 backend 位姿驱动
	ctx.appendHierarchicalRobotSimulationContext(urdf, jn, lo, hi, QHash<QString, osg::MatrixTransform*>(), sceneRoot,
												 jointRoot);
	QHash<QString, cloudsim::core::Mat4> fkT0Mat4;
	QHash<QString, cloudsim::core::Mat4> outerMat4;
	for (auto it = fkT0.constBegin(); it != fkT0.constEnd(); ++it)
	{
		fkT0Mat4.insert(it.key(), RobotSceneKinematics::coreMat4FromOsgMatrix(it.value()));
	}
	for (auto it = outer.constBegin(); it != outer.constEnd(); ++it)
	{
		outerMat4.insert(it.key(), RobotSceneKinematics::coreMat4FromOsgMatrix(it.value()));
	}
	ctx.setRobotPerLinkKinematicsBinding(importKey, linkMap, fkT0Mat4, outerMat4, false);
	reapplyUrdfRobotHierarchyEdges(backend, urdf, sceneRoot, linkMap);

	// 恢复机器人基座放置位姿 P（JSON 列主序 16 元 ≡ core::Mat4）
	const QJsonArray basePlacementArr = rk.value(QStringLiteral("basePlacementWorld")).toArray();
	if (basePlacementArr.size() == 16)
	{
		cloudsim::core::Mat4 basePlacement{};
		for (int i = 0; i < 16; ++i)
		{
			basePlacement[static_cast<size_t>(i)] =
				basePlacementArr.at(i).toDouble((i % 5 == 0) ? 1.0 : 0.0);
		}
		const int instIdx = ctx.robotKinematicInstanceCount() - 1;
		if (instIdx >= 0)
		{
			ctx.setRobotBasePlacementWorldForInstance(instIdx, basePlacement);
		}
	}

	const QJsonObject cfObj = rk.value(QStringLiteral("coordinateFrames")).toObject();
	if (!cfObj.isEmpty())
	{
		const QByteArray raw = QJsonDocument(cfObj).toJson(QJsonDocument::Compact);
		try
		{
			const nlohmann::json cfJ = nlohmann::json::parse(raw.constData(), raw.constData() + raw.size());
			RobotCoordinate::RobotCoordinateFrameSet frames;
			if (RobotCoordinate::readCoordinateFrameSetFromJson(cfJ, frames))
			{
				const int instIdx = ctx.robotKinematicInstanceCount() - 1;
				if (instIdx >= 0)
				{
					ctx.robotCoordinateFramesForInstance(instIdx) = std::move(frames);
				}
			}
		}
		catch (...)
		{
			RunLogger::warn("kinematics restore: coordinateFrames JSON parse failed; frames skipped");
		}
	}
	else
	{
		const int instIdx = ctx.robotKinematicInstanceCount() - 1;
		if (instIdx >= 0)
		{
			QString defaultFlange;
			QStringList childLinks;
			(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdf, childLinks, nullptr);
			if (!childLinks.isEmpty())
			{
				defaultFlange = childLinks.back();
			}
			ctx.robotCoordinateFramesForInstance(instIdx) =
				RobotCoordinate::makeDefaultFrameSet(defaultFlange.toStdString());
		}
	}

	const QJsonObject eaObj = rk.value(QStringLiteral("externalAxes")).toObject();
	if (!eaObj.isEmpty())
	{
		const QByteArray raw = QJsonDocument(eaObj).toJson(QJsonDocument::Compact);
		try
		{
			const nlohmann::json eaJ = nlohmann::json::parse(raw.constData(), raw.constData() + raw.size());
			RobotExternal::RobotExternalAxisConfigSet axes;
			if (RobotExternal::readExternalAxisConfigSetFromJson(eaJ, axes))
			{
				const int instIdx = ctx.robotKinematicInstanceCount() - 1;
				if (instIdx >= 0)
				{
					ctx.robotExternalAxesForInstance(instIdx) = std::move(axes);
				}
			}
		}
		catch (...)
		{
			RunLogger::warn("kinematics restore: externalAxes JSON parse failed; ext-axis config skipped");
		}
	}

	outAllJointAnglesRad += q0; // 多机实例关节角拼接
	return true;
}

} // namespace cloudsim::host
