#pragma once

#include "aibackend_global.h"

#include <QByteArray>
#include <QString>
#include <QUrl>

namespace AiHttpsPost
{
/// POST body 到 url，可选额外头行
AIBACKEND_EXPORT bool post(
	const QUrl& url,
	const QByteArray& body,
	const QList<QPair<QByteArray, QByteArray>>& headers,
	QByteArray& responseBody,
	QString& errorMessage,
	int timeoutMs);
}
