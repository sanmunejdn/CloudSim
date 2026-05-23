#include "RobotProjectIoAdapter.h"
#include "IRobotDocumentHost.h"
#include "RobotInstructionFactory.h"
#include "RobotProgramStore.h"

#include <QJsonDocument>

#include <json.hpp>

namespace RobotProjectIo
{

void writeRobotKinematicsAndPrograms(
	QJsonObject& root,
	IRobotDocumentHost* doc,
	const QVector<double>* aggregatedJointAnglesRad)
{
	if (!doc)
	{
		return;
	}
	if (doc->robotKinematicInstanceCount() > 0)
	{
		QJsonArray robotsArr;
		for (int ri = 0; ri < doc->robotKinematicInstanceCount(); ++ri)
		{
			RobotPerLinkKinematicsSlice pl;
			if (!doc->robotPerLinkKinematicsForInstance(ri, pl))
			{
				continue;
			}
			QJsonObject rk;
			rk.insert(QStringLiteral("mode"), QStringLiteral("perLink"));
			rk.insert(QStringLiteral("urdf"), pl.urdfAbsolutePath);
			rk.insert(QStringLiteral("sceneRootBackendId"), pl.sceneRootBackendId);
			const QString prefix = doc->robotJointKeyPrefixForInstance(ri);
			QString jointRoot = prefix;
			if (jointRoot.endsWith(QStringLiteral("::")))
			{
				jointRoot.chop(2);
			}
			rk.insert(QStringLiteral("jointPrefixRoot"), jointRoot);
			rk.insert(QStringLiteral("importKey"), jointRoot + QStringLiteral("_ctx"));
			QJsonObject linksObj;
			for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
			{
				linksObj.insert(it.key(), it.value());
			}
			rk.insert(QStringLiteral("links"), linksObj);
			if (pl.meshVerticesInLinkFrame)
			{
				rk.insert(QStringLiteral("meshInLinkFrame"), true);
			}
			nlohmann::json cfJ;
			RobotCoordinate::writeCoordinateFrameSetToJson(doc->robotCoordinateFramesForInstance(ri), cfJ);
			const QByteArray cfRaw = QByteArray::fromStdString(cfJ.dump());
			const QJsonDocument cfDoc = QJsonDocument::fromJson(cfRaw);
			if (cfDoc.isObject())
			{
				rk.insert(QStringLiteral("coordinateFrames"), cfDoc.object());
			}
			if (aggregatedJointAnglesRad && !aggregatedJointAnglesRad->isEmpty())
			{
				const int offset = doc->robotJointOffsetInAggregatedVector(ri);
				const int nj = doc->robotRevoluteJointCountForInstance(ri);
				if (nj > 0 && aggregatedJointAnglesRad->size() >= offset + nj)
				{
					QJsonArray ja;
					for (int j = 0; j < nj; ++j)
					{
						ja.append((*aggregatedJointAnglesRad)[offset + j]);
					}
					rk.insert(QStringLiteral("jointAnglesRad"), ja);
				}
			}
			robotsArr.push_back(rk);
		}
		if (!robotsArr.isEmpty())
		{
			root.insert(QStringLiteral("robotKinematicsInstances"), robotsArr);
		}
	}
	if (root.value(QStringLiteral("robotKinematicsInstances")).isUndefined()
		&& !doc->robotLinkNameToBackendId().isEmpty())
	{
		QJsonObject rk;
		rk.insert(QStringLiteral("mode"), QStringLiteral("perLink"));
		rk.insert(QStringLiteral("urdf"), doc->robotUrdfAbsolutePath());
		rk.insert(QStringLiteral("sceneRootBackendId"), doc->robotSceneBackendIdForInstance(0));
		QJsonObject linksObj;
		for (auto it = doc->robotLinkNameToBackendId().constBegin(); it != doc->robotLinkNameToBackendId().constEnd();
			 ++it)
		{
			linksObj.insert(it.key(), it.value());
		}
		rk.insert(QStringLiteral("links"), linksObj);
		if (doc->robotUrdfMeshVerticesInLinkFrame())
		{
			rk.insert(QStringLiteral("meshInLinkFrame"), true);
		}
		root.insert(QStringLiteral("robotKinematics"), rk);
	}

	QJsonArray programsArr;
	for (auto it = doc->robotProgramStore().allPrograms().constBegin();
		 it != doc->robotProgramStore().allPrograms().constEnd();
		 ++it)
	{
		if (it.value().empty())
		{
			continue;
		}
		QJsonObject entry;
		entry.insert(QStringLiteral("sceneBackendId"), it.key());
		QJsonArray insArr;
		for (const auto& ins : it.value())
		{
			if (!ins)
			{
				continue;
			}
			const nlohmann::json j = RobotInstruction::toJson(*ins);
			const QByteArray raw = QByteArray::fromStdString(j.dump());
			const QJsonDocument jdoc = QJsonDocument::fromJson(raw);
			if (jdoc.isObject())
			{
				insArr.append(jdoc.object());
			}
		}
		entry.insert(QStringLiteral("instructions"), insArr);
		programsArr.append(entry);
	}
	if (!programsArr.isEmpty())
	{
		root.insert(QStringLiteral("robotPrograms"), programsArr);
	}
}

void loadRobotPrograms(
	const QJsonObject& root,
	IRobotDocumentHost* doc,
	const std::function<void(const QString&)>& appendWarning)
{
	if (!doc)
	{
		return;
	}
	const QJsonArray robotPrograms = root.value(QStringLiteral("robotPrograms")).toArray();
	for (const QJsonValue& pv : robotPrograms)
	{
		if (!pv.isObject())
		{
			continue;
		}
		const QJsonObject progObj = pv.toObject();
		const QString sceneBackendId = progObj.value(QStringLiteral("sceneBackendId")).toString();
		if (sceneBackendId.isEmpty())
		{
			continue;
		}
		if (doc->robotInstanceIndexForSceneBackendId(sceneBackendId) < 0)
		{
			if (appendWarning)
			{
				appendWarning(
					QStringLiteral("robotPrograms: unknown sceneBackendId %1").arg(sceneBackendId));
			}
			continue;
		}
		const QJsonArray insArr = progObj.value(QStringLiteral("instructions")).toArray();
		nlohmann::json arr = nlohmann::json::array();
		for (const QJsonValue& iv : insArr)
		{
			if (!iv.isObject())
			{
				continue;
			}
			const QByteArray raw = QJsonDocument(iv.toObject()).toJson(QJsonDocument::Compact);
			arr.push_back(nlohmann::json::parse(std::string(raw.constData(), static_cast<size_t>(raw.size()))));
		}
		std::string parseErr;
		std::vector<std::shared_ptr<RobotInstruction::Base>> program =
			RobotInstruction::createListFromJson(arr, &parseErr);
		if (!parseErr.empty() && appendWarning)
		{
			appendWarning(QString::fromStdString(parseErr));
		}
		doc->robotProgramStore().setProgramFor(sceneBackendId, std::move(program));
	}
}

}
