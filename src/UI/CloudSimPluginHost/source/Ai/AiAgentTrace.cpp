/// @file AiAgentTrace.cpp
/// @brief 追加一行 JSONL 便于复盘

#include "Ai/AiAgentTrace.h"

#include <QDateTime>
#include <QFile>
#include <QMutex>
#include <QMutexLocker>

#include <json.hpp>

namespace AiAgentTrace
{
namespace
{
QMutex g_mu;
}

QString traceFilePath(const QString& applicationDirPath)
{
	return applicationDirPath + QStringLiteral("/ai_agent_trace.jsonl");
}

void append(const QString& applicationDirPath, const QString& state, const QString& toolId, bool ok,
			const QByteArray& detailUtf8)
{
	if (applicationDirPath.isEmpty())
		return;
	nlohmann::json row;
	row["ts"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString();
	row["state"] = state.toStdString();
	row["tool"] = toolId.toStdString();
	row["ok"] = ok;
	if (!detailUtf8.isEmpty())
	{
		try
		{
			row["detail"] = nlohmann::json::parse(detailUtf8.constData(), nullptr, true);
		}
		catch (...)
		{
			row["detail"] = detailUtf8.constData();
		}
	}
	QMutexLocker lock(&g_mu);
	QFile f(traceFilePath(applicationDirPath));
	if (!f.open(QIODevice::WriteOnly | QIODevice::Append))
		return;
	f.write(QByteArray::fromStdString(row.dump()));
	f.write("\n");
}
} // namespace AiAgentTrace
