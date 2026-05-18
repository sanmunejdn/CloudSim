#pragma once

#include "aibackend_global.h"

#include <QByteArray>
#include <QString>
#include <QUrl>

namespace AiHttpsPost
{
/// POST \a body to \a url with optional extra \a headerLines (each "Name: value", no CRLF).
AIBACKEND_EXPORT bool post(
	const QUrl& url,
	const QByteArray& body,
	const QList<QPair<QByteArray, QByteArray>>& headers,
	QByteArray& responseBody,
	QString& errorMessage,
	int timeoutMs);
}
