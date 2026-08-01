/// @file ScriptModelIo.cpp

#include "ScriptModelIo.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

ScriptModelParseResult parseScriptModelJson(const QByteArray& utf8)
{
	ScriptModelParseResult out;
	QJsonParseError pe{};
	const QJsonDocument doc = QJsonDocument::fromJson(utf8, &pe);
	if (pe.error != QJsonParseError::NoError || !doc.isObject())
	{
		out.error = QStringLiteral("Invalid JSON: %1").arg(pe.errorString());
		return out;
	}
	const QJsonObject root = doc.object();
	const QString domain = root.value(QStringLiteral("domain")).toString();
	if (domain == QLatin1String("feature.compose"))
	{
		if (!root.value(QStringLiteral("steps")).isArray())
		{
			out.error = QStringLiteral("feature.compose requires steps[]");
			return out;
		}
		out.kind = ScriptModelJsonKind::Compose;
		out.payloadUtf8 = utf8;
		return out;
	}
	if (root.contains(QStringLiteral("parametricHistory")))
	{
		const QJsonValue ph = root.value(QStringLiteral("parametricHistory"));
		if (!ph.isObject())
		{
			out.error = QStringLiteral("parametricHistory must be object");
			return out;
		}
		const QJsonObject hist = ph.toObject();
		if (!hist.value(QStringLiteral("features")).isArray())
		{
			out.error = QStringLiteral("parametricHistory.features must be array");
			return out;
		}
		out.kind = ScriptModelJsonKind::History;
		out.payloadUtf8 = QJsonDocument(hist).toJson(QJsonDocument::Compact);
		return out;
	}
	if (root.value(QStringLiteral("features")).isArray())
	{
		out.kind = ScriptModelJsonKind::History;
		out.payloadUtf8 = utf8;
		return out;
	}
	out.error = QStringLiteral("Unrecognized script JSON (need features[] or domain=feature.compose)");
	return out;
}
