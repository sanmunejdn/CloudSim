/// @file AiAgentMemory.cpp
/// @brief Agent 偏好（全局 + 按文档）与会话摘要

#include "Ai/AiAgentMemory.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMutex>
#include <QMutexLocker>
#include <QVector>

namespace AiAgentMemory
{
namespace
{
QMutex g_mu;
QVector<QString> g_steps;

QJsonObject readRoot(const QString& path)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
		return {};
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
	return doc.isObject() ? doc.object() : QJsonObject();
}

void writeRoot(const QString& path, const QJsonObject& root)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
		return;
	f.write(QJsonDocument(root).toJson(QJsonDocument::Compact));
}

QVariantMap objectToMap(const QJsonObject& o)
{
	QVariantMap out;
	for (auto it = o.begin(); it != o.end(); ++it)
		out.insert(it.key(), it.value().toVariant());
	return out;
}
} // namespace

QString memoryFilePath(const QString& applicationDirPath)
{
	return applicationDirPath + QStringLiteral("/ai_agent_memory.json");
}

QVariantMap loadPrefs(const QString& applicationDirPath, const QString& documentId)
{
	const QJsonObject root = readRoot(memoryFilePath(applicationDirPath));
	QVariantMap out = objectToMap(root.value(QStringLiteral("prefs")).toObject());
	if (!documentId.isEmpty())
	{
		const QJsonObject byDoc = root.value(QStringLiteral("prefs_by_doc")).toObject();
		const QJsonObject docPrefs = byDoc.value(documentId).toObject();
		// 文档级覆盖全局
		const QVariantMap docMap = objectToMap(docPrefs);
		for (auto it = docMap.begin(); it != docMap.end(); ++it)
			out.insert(it.key(), it.value());
	}
	return out;
}

void savePrefForApi(const QString& applicationDirPath, const QString& apiId, const QVariantMap& args,
					const QString& documentId)
{
	if (apiId.isEmpty())
		return;
	const QString path = memoryFilePath(applicationDirPath);
	QJsonObject root = readRoot(path);
	if (documentId.isEmpty())
	{
		QJsonObject prefs = root.value(QStringLiteral("prefs")).toObject();
		prefs.insert(apiId, QJsonObject::fromVariantMap(args));
		root.insert(QStringLiteral("prefs"), prefs);
	}
	else
	{
		QJsonObject byDoc = root.value(QStringLiteral("prefs_by_doc")).toObject();
		QJsonObject docPrefs = byDoc.value(documentId).toObject();
		docPrefs.insert(apiId, QJsonObject::fromVariantMap(args));
		byDoc.insert(documentId, docPrefs);
		root.insert(QStringLiteral("prefs_by_doc"), byDoc);
	}
	writeRoot(path, root);
}

QByteArray sessionSummaryUtf8()
{
	QMutexLocker lock(&g_mu);
	QString s;
	const int n = g_steps.size();
	const int from = n > 12 ? n - 12 : 0;
	for (int i = from; i < n; ++i)
	{
		if (!s.isEmpty())
			s += QLatin1Char('\n');
		s += g_steps[i];
	}
	return s.toUtf8();
}

void appendSessionStep(const QString& toolId, bool ok, const QString& summary)
{
	QMutexLocker lock(&g_mu);
	g_steps.push_back(QStringLiteral("[%1] %2 %3")
						  .arg(ok ? QStringLiteral("ok") : QStringLiteral("fail"), toolId, summary));
	if (g_steps.size() > 64)
		g_steps.remove(0, g_steps.size() - 64);
}

void clearSession()
{
	QMutexLocker lock(&g_mu);
	g_steps.clear();
}
} // namespace AiAgentMemory
