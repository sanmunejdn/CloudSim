/// @file main.cpp
/// @brief CloudSimWeb 独立进程入口（与桌面 CloudSim.exe 互不影响）

#include "CloudSimBootstrap.h"
#include "ICloudSimContext.h"
#include "WebGateway.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QGuiApplication>
#include <QtWidgets/QApplication>
#include <cstring>

#ifdef Q_OS_WIN
#include <windows.h>

namespace
{
void prependPathIfExists(const QString& dirPath)
{
	if (dirPath.isEmpty() || !QFileInfo::exists(dirPath))
		return;
	QByteArray current = qgetenv("PATH");
	const QByteArray prefix = QDir::toNativeSeparators(dirPath).toLocal8Bit();
	if (current.startsWith(prefix + ';') || current == prefix)
		return;
	qputenv("PATH", current.isEmpty() ? prefix : prefix + ';' + current);
}

void configureWindowsDllSearchPath()
{
	const QString appDir = QCoreApplication::applicationDirPath();
	// 先本目录（Host/Data 等），再 OSG 运行时
	prependPathIfExists(appDir);
	for (const QString& c : {QDir(appDir).absoluteFilePath("../SDK/OSG3.6.5/bin"),
							 QDir(appDir).absoluteFilePath("../../SDK/OSG3.6.5/bin"),
							 QDir(appDir).absoluteFilePath("OSG3.6.5/bin")})
	{
		prependPathIfExists(QDir(c).absolutePath());
	}
}
} // namespace
#endif

static int parsePort(int argc, char* argv[], int fallback)
{
	for (int i = 1; i < argc; ++i)
	{
		if (std::strncmp(argv[i], "--port=", 7) == 0)
		{
			const int p = std::atoi(argv[i] + 7);
			if (p > 0 && p < 65536)
				return p;
		}
	}
	return fallback;
}

int main(int argc, char* argv[])
{
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
	QApplication a(argc, argv);
#ifdef Q_OS_WIN
	configureWindowsDllSearchPath();
#endif
	QCoreApplication::setOrganizationName(QStringLiteral("CloudSim"));
	QCoreApplication::setApplicationName(QStringLiteral("CloudSimWeb"));

	cloudsimSetApplicationContext(cloudsimCreateHeadlessApplicationContext());
	auto* ctx = cloudsimApplicationContext();
	if (!ctx)
		return 1;

	cloudsim::web::WebGatewayConfig cfg;
	cfg.port = parsePort(argc, argv, 8787);
	cfg.staticRoot = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("web"));

	cloudsim::web::WebGateway gateway(*ctx, cfg);
	QString err;
	if (!gateway.start(&err))
	{
		fprintf(stderr, "CloudSimWeb: %s\n", err.toLocal8Bit().constData());
		return 2;
	}
	fprintf(stdout, "CloudSimWeb listening on http://127.0.0.1:%d (role=web)\n", gateway.port());
	fflush(stdout);

	const int ret = a.exec();
	gateway.stop();
	cloudsimSetApplicationContext(nullptr);
	return ret;
}
