#ifndef CLOUDSIMPLUGINHOST_AIHTTPSPOST_H
#define CLOUDSIMPLUGINHOST_AIHTTPSPOST_H

/// @file AiHttpsPost.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief POST body 到 url，可选额外头行

#include "aibackend_global.h"

#include <QByteArray>
#include <QString>
#include <QUrl>

namespace AiHttpsPost
{
/// POST body 到 url，可选额外头行
AIBACKEND_EXPORT bool post(const QUrl& url, const QByteArray& body, const QList<QPair<QByteArray, QByteArray>>& headers,
						   QByteArray& responseBody, QString& errorMessage, int timeoutMs);
} // namespace AiHttpsPost

#endif // CLOUDSIMPLUGINHOST_AIHTTPSPOST_H
