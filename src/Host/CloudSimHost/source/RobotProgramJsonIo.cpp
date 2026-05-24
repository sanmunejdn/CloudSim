#include "RobotProgramJsonIo.h"

#include "IRobotUrdfImportContext.h"

#include "RobotInstructionFactory.h"
#include "RobotProgramStore.h"

#include <QJsonDocument>
#include <QJsonObject>

#include <json.hpp>

namespace cloudsim::host {

QJsonArray robotProgramsToJson(const RobotProgramStore& store)
{
	QJsonArray programsArr;
	for (auto it = store.allPrograms().constBegin(); it != store.allPrograms().constEnd(); ++it)
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
	return programsArr;
}

bool robotProgramsFromJson(RobotProgramStore& store, const QJsonArray& programs, IRobotUrdfImportContext& ctx,
	QString* outError)
{
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
		// 须先恢复 robotKinematics，再加载程序
		if (ctx.robotInstanceIndexForSceneBackendId(sceneBackendId) < 0)
		{
			if (outError)
			{
				*outError = QStringLiteral("robotPrograms: unknown sceneBackendId %1").arg(sceneBackendId);
			}
			return false;
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
		if (!parseErr.empty())
		{
			if (outError)
			{
				*outError = QString::fromStdString(parseErr);
			}
			return false;
		}
		store.setProgramFor(sceneBackendId, std::move(program));
	}
	return true;
}

} // namespace cloudsim::host
