#include "AiHttpsPost.h"

#include <algorithm>

#ifdef Q_OS_WIN
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#endif

#include <QSslSocket>
#include <QEventLoop>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>

namespace AiHttpsPost
{
namespace
{
#ifdef Q_OS_WIN
bool postWinHttp(
	const QUrl& url,
	const QByteArray& body,
	const QList<QPair<QByteArray, QByteArray>>& headers,
	QByteArray& responseBody,
	QString& errorMessage,
	int timeoutMs)
{
	responseBody.clear();
	errorMessage.clear();

	if (!url.isValid() || url.host().isEmpty())
	{
		errorMessage = QStringLiteral("Invalid URL.");
		return false;
	}

	const bool secure = url.scheme().compare(QStringLiteral("https"), Qt::CaseInsensitive) == 0;
	const INTERNET_PORT port = static_cast<INTERNET_PORT>(
		url.port(secure ? 443 : 80));

	const QString path = url.path(QUrl::FullyEncoded).isEmpty()
		? QStringLiteral("/")
		: url.path(QUrl::FullyEncoded);
	const QString pathAndQuery = url.hasQuery() ? path + QLatin1Char('?') + url.query(QUrl::FullyEncoded) : path;

	const std::wstring hostW = url.host().toStdWString();
	const std::wstring pathW = pathAndQuery.toStdWString();

	HINTERNET session = WinHttpOpen(L"PointCloudProcess/1.0",
		WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
		WINHTTP_NO_PROXY_NAME,
		WINHTTP_NO_PROXY_BYPASS,
		0);
	if (!session)
	{
		errorMessage = QStringLiteral("WinHttpOpen failed (%1).").arg(GetLastError());
		return false;
	}

	WinHttpSetTimeouts(session, timeoutMs, timeoutMs, timeoutMs, timeoutMs);

	HINTERNET connect = WinHttpConnect(session, hostW.c_str(), port, 0);
	if (!connect)
	{
		errorMessage = QStringLiteral("WinHttpConnect failed (%1).").arg(GetLastError());
		WinHttpCloseHandle(session);
		return false;
	}

	const DWORD flags = secure ? WINHTTP_FLAG_SECURE : 0;
	HINTERNET request = WinHttpOpenRequest(connect, L"POST", pathW.c_str(),
		nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
	if (!request)
	{
		errorMessage = QStringLiteral("WinHttpOpenRequest failed (%1).").arg(GetLastError());
		WinHttpCloseHandle(connect);
		WinHttpCloseHandle(session);
		return false;
	}

	auto addHeader = [&](const QString& line) {
		QString h = line;
		if (!h.endsWith(QStringLiteral("\r\n")))
			h += QStringLiteral("\r\n");
		const std::wstring w = h.toStdWString();
		WinHttpAddRequestHeaders(request, w.c_str(), static_cast<DWORD>(-1), WINHTTP_ADDREQ_FLAG_ADD);
	};
	addHeader(QStringLiteral("Content-Type: application/json"));
	for (const auto& h : headers)
	{
		if (h.first.isEmpty())
			continue;
		const QString name = QString::fromUtf8(h.first);
		const QString value = QString::fromUtf8(h.second);
		addHeader(name + QLatin1Char(':') + QLatin1Char(' ') + value);
	}

	LPVOID bodyPtr = WINHTTP_NO_REQUEST_DATA;
	DWORD bodyLen = 0;
	QByteArray bodyCopy;
	if (!body.isEmpty())
	{
		bodyCopy = body;
		bodyPtr = bodyCopy.data();
		bodyLen = static_cast<DWORD>(bodyCopy.size());
	}

	const BOOL sendOk = WinHttpSendRequest(request,
		WINHTTP_NO_ADDITIONAL_HEADERS,
		0,
		bodyPtr,
		bodyLen,
		bodyLen,
		0);
	if (!sendOk)
	{
		errorMessage = QStringLiteral("WinHttpSendRequest failed (%1).").arg(GetLastError());
		WinHttpCloseHandle(request);
		WinHttpCloseHandle(connect);
		WinHttpCloseHandle(session);
		return false;
	}

	if (!WinHttpReceiveResponse(request, nullptr))
	{
		errorMessage = QStringLiteral("WinHttpReceiveResponse failed (%1).").arg(GetLastError());
		WinHttpCloseHandle(request);
		WinHttpCloseHandle(connect);
		WinHttpCloseHandle(session);
		return false;
	}

	DWORD statusCode = 0;
	DWORD statusSize = sizeof(statusCode);
	WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		WINHTTP_HEADER_NAME_BY_INDEX, &statusCode, &statusSize, WINHTTP_NO_HEADER_INDEX);

	QByteArray accumulated;
	for (;;)
	{
		DWORD available = 0;
		if (!WinHttpQueryDataAvailable(request, &available))
			break;
		if (available == 0)
			break;
		const int prevSize = accumulated.size();
		accumulated.resize(prevSize + static_cast<int>(available));
		DWORD read = 0;
		if (!WinHttpReadData(request, accumulated.data() + prevSize, available, &read))
		{
			errorMessage = QStringLiteral("WinHttpReadData failed (%1).").arg(GetLastError());
			WinHttpCloseHandle(request);
			WinHttpCloseHandle(connect);
			WinHttpCloseHandle(session);
			return false;
		}
		accumulated.resize(prevSize + static_cast<int>(read));
	}

	WinHttpCloseHandle(request);
	WinHttpCloseHandle(connect);
	WinHttpCloseHandle(session);

	responseBody = accumulated;
	if (statusCode < 200 || statusCode >= 300)
	{
		errorMessage = QStringLiteral("HTTP %1: %2").arg(statusCode).arg(QString::fromUtf8(responseBody.left(512)));
		return false;
	}
	return true;
}
#endif

bool postQtNetwork(
	const QUrl& url,
	const QByteArray& body,
	const QList<QPair<QByteArray, QByteArray>>& headers,
	QByteArray& responseBody,
	QString& errorMessage,
	int timeoutMs)
{
	if (!QSslSocket::supportsSsl())
	{
		errorMessage = QStringLiteral(
			"Qt SSL is not available (missing OpenSSL DLLs). "
			"On Windows use HTTPS via the built-in WinHTTP backend, or copy libssl-1_1-x64.dll and libcrypto-1_1-x64.dll next to the executable.");
		return false;
	}

	QNetworkAccessManager nam;
	QNetworkRequest req(url);
	req.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
	for (const auto& h : headers)
		req.setRawHeader(h.first, h.second);

	QEventLoop loop;
	QTimer timer;
	timer.setSingleShot(true);
	QNetworkReply* reply = nam.post(req, body);
	QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
	QObject::connect(&timer, &QTimer::timeout, &loop, [&]() {
		if (reply->isRunning())
			reply->abort();
		loop.quit();
	});
	timer.start((std::max)(5000, timeoutMs));
	loop.exec();

	if (!timer.isActive() && reply->error() == QNetworkReply::OperationCanceledError)
	{
		errorMessage = QStringLiteral("HTTP request timed out.");
		reply->deleteLater();
		return false;
	}

	if (reply->error() != QNetworkReply::NoError)
	{
		errorMessage = QStringLiteral("Network error: %1").arg(reply->errorString());
		reply->deleteLater();
		return false;
	}

	const int httpStatus = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
	responseBody = reply->readAll();
	reply->deleteLater();

	if (httpStatus >= 200 && httpStatus < 300)
		return true;

	errorMessage = QStringLiteral("HTTP %1: %2").arg(httpStatus).arg(QString::fromUtf8(responseBody.left(512)));
	return false;
}
}

bool post(
	const QUrl& url,
	const QByteArray& body,
	const QList<QPair<QByteArray, QByteArray>>& headers,
	QByteArray& responseBody,
	QString& errorMessage,
	int timeoutMs)
{
#ifdef Q_OS_WIN
	return postWinHttp(url, body, headers, responseBody, errorMessage, timeoutMs);
#else
	return postQtNetwork(url, body, headers, responseBody, errorMessage, timeoutMs);
#endif
}

} // namespace AiHttpsPost
