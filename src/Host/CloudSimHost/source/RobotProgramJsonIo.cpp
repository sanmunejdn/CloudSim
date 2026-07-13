#include "RobotProgramJsonIo.h"

#include "IRobotUrdfImportContext.h"

#include "RobotInstructionFactory.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"

#include <QJsonDocument>
#include <QJsonObject>

#include <json.hpp>

namespace cloudsim::host {

QJsonArray robotProgramsToJson(const RobotProgramStore& store)
{
	QJsonArray programsArr;
	for (auto it = store.allCatalogs().constBegin(); it != store.allCatalogs().constEnd(); ++it)
	{
		const RobotInstruction::RobotProgramCatalog& catalog = it.value();
		bool hasContent = false;
		for (const RobotInstruction::RobotProgram& prog : catalog.programs())
		{
			if (!prog.steps.empty() || !prog.groups.empty() || prog.isMain)
			{
				hasContent = true;
				break;
			}
		}
		if (!hasContent)
		{
			continue;
		}
		QJsonObject entry;
		entry.insert(QStringLiteral("sceneBackendId"), it.key());
		const nlohmann::json catalogJson = catalog.toJson();
		const QByteArray raw = QByteArray::fromStdString(catalogJson.dump());
		const QJsonDocument jdoc = QJsonDocument::fromJson(raw);
		if (jdoc.isObject())
		{
			const QJsonObject obj = jdoc.object();
			if (obj.contains(QStringLiteral("activeProgramId")))
			{
				entry.insert(QStringLiteral("activeProgramId"), obj.value(QStringLiteral("activeProgramId")));
			}
			if (obj.contains(QStringLiteral("programs")))
			{
				entry.insert(QStringLiteral("programs"), obj.value(QStringLiteral("programs")));
			}
			if (obj.contains(QStringLiteral("pathPlanRaws")))
			{
				entry.insert(QStringLiteral("pathPlanRaws"), obj.value(QStringLiteral("pathPlanRaws")));
			}
		}
		programsArr.append(entry);
	}
	return programsArr;
}

bool robotProgramsFromJson(RobotProgramStore& store, const QJsonArray& programs, IRobotUrdfImportContext& ctx,
	QString* outError)
{
	// 整表替换会使 UI 持有的 steps/groups 裸指针失效；调用方须在刷新前 bindProgramTree
	for (const QJsonValue& pv : programs)
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
		if (ctx.robotInstanceIndexForSceneBackendId(sceneBackendId) < 0)
		{
			if (outError)
			{
				*outError = QStringLiteral("robotPrograms: unknown sceneBackendId %1").arg(sceneBackendId);
			}
			return false;
		}
		RobotInstruction::RobotProgramCatalog catalog;
		if (progObj.contains(QStringLiteral("programs")))
		{
			const QByteArray raw = QJsonDocument(progObj).toJson(QJsonDocument::Compact);
			nlohmann::json j = nlohmann::json::parse(std::string(raw.constData(), static_cast<size_t>(raw.size())));
			std::string parseErr;
			if (!RobotInstruction::RobotProgramCatalog::fromJson(j, catalog, &parseErr))
			{
				if (outError)
				{
					*outError = QString::fromStdString(parseErr);
				}
				return false;
			}
		}
		else
		{
			catalog = RobotInstruction::RobotProgramCatalog::withDefaultMain();
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
			if (RobotInstruction::RobotProgram* mainProg = catalog.mainProgram())
			{
				mainProg->steps = RobotInstruction::createListFromJson(arr, &parseErr);
			}
			if (!parseErr.empty())
			{
				if (outError)
				{
					*outError = QString::fromStdString(parseErr);
				}
				return false;
			}
		}
		store.catalogFor(sceneBackendId) = std::move(catalog);
	}
	return true;
}

} // namespace cloudsim::host
