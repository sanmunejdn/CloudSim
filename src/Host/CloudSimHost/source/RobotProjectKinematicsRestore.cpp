/// @file RobotProjectKinematicsRestore.cpp
/// @brief RobotProjectKinematicsRestore 实现

#include "RobotProjectKinematicsRestore.h"

#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotUrdfImportContext.h"
#include "MeshBackendData.h"
#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"
#include "UrdfRobotLoader.h"

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
			outer.insert(bid, RobotMatrixOsg::matrixFromBackendColMajor(meshPtr->worldMatrix(&backend)));
		}
		else
		{
			outer.insert(bid, osg::Matrixd::identity());
		}
	}
	// 无 OSG 关节节点：perLink 模式靠 backend 位姿驱动
	ctx.appendHierarchicalRobotSimulationContext(urdf, jn, lo, hi, QHash<QString, osg::MatrixTransform*>(), sceneRoot,
												 jointRoot);
	ctx.setRobotPerLinkKinematicsBinding(importKey, linkMap, fkT0, outer, false);

	// 恢复机器人基座放置位姿 P
	const QJsonArray basePlacementArr = rk.value(QStringLiteral("basePlacementWorld")).toArray();
	if (basePlacementArr.size() == 16)
	{
		osg::Matrixd basePlacement;
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				basePlacement(r, c) = basePlacementArr.at(c * 4 + r).toDouble(c == r ? 1.0 : 0.0);
			}
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
	outAllJointAnglesRad += q0; // 多机实例关节角拼接
	return true;
}

} // namespace cloudsim::host
