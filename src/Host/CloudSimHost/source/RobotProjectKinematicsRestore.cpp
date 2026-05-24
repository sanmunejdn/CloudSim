#include "RobotProjectKinematicsRestore.h"

#include "IRobotUrdfImportContext.h"

#include "BackendDataManager.h"
#include "RobotCoordinateFrames.h"
#include "UrdfRobotLoader.h"

#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>

#include <json.hpp>

#include <osg/Matrixd>
#include <osg/MatrixTransform>

namespace cloudsim::host {

namespace {

void collectRobotLinkIdsFromKinematicsObject(const QJsonObject& rkObj, QSet<QString>& outIds)
{
	if (rkObj.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink")
		|| !rkObj.value(QStringLiteral("meshInLinkFrame")).toBool())
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
	const bool meshInLinkFrame = rk.value(QStringLiteral("meshInLinkFrame")).toBool();
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
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdf, q0, Tq, &fkErr, meshInLinkFrame))
	{
		if (outWarning)
		{
			*outWarning = QStringLiteral("robotKinematics: FK restore failed: %1").arg(fkErr);
		}
		return false;
	}
	QHash<QString, osg::Matrixd> fkT0;
	QHash<QString, osg::Matrixd> outer;
	for (auto it = linkMap.constBegin(); it != linkMap.constEnd(); ++it)
	{
		const QString& lname = it.key();
		if (Tq.contains(lname))
		{
			const osg::Matrixd& meshWorld0 = Tq[lname];
			fkT0.insert(lname, meshWorld0);
			outer.insert(it.value(), meshWorld0);
		}
	}
	// 无 OSG 关节节点：perLink 模式靠 backend 位姿驱动
	ctx.appendHierarchicalRobotSimulationContext(
		urdf, jn, lo, hi, QHash<QString, osg::MatrixTransform*>(), sceneRoot, jointRoot);
	ctx.setRobotPerLinkKinematicsBinding(importKey, linkMap, fkT0, outer, meshInLinkFrame);
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
