#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include "MainWindow.h"
#include "RunLogger.h"

#ifdef Q_OS_WIN
#include <windows.h>

namespace {

void prependPathIfExists(const QString& dirPath)
{
	if (dirPath.isEmpty() || !QFileInfo::exists(dirPath))
	{
		return;
	}
	QByteArray current = qgetenv("PATH");
	const QByteArray prefix = QDir::toNativeSeparators(dirPath).toLocal8Bit();
	if (current.startsWith(prefix + ';')
		|| current.startsWith(prefix + ':')
		|| current == prefix)
	{
		return;
	}
	if (current.isEmpty())
	{
		qputenv("PATH", prefix);
	}
	else
	{
		qputenv("PATH", prefix + ';' + current);
	}
}

void configureWindowsDllSearchPath()
{
	// Ensure project OSG runtime is searched before system PATH entries (e.g. osg161-*.dll).
	const QString appDir = QCoreApplication::applicationDirPath();
	const QStringList candidates{
		QDir(appDir).absoluteFilePath("../OSG3.6.5/bin"),
		QDir(appDir).absoluteFilePath("../../OSG3.6.5/bin"),
		QDir(appDir).absoluteFilePath("OSG3.6.5/bin")
	};
	for (const QString& c : candidates)
	{
		prependPathIfExists(QDir(c).absolutePath());
	}
}

} // namespace
#endif

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
#ifdef Q_OS_WIN
	configureWindowsDllSearchPath();
#endif
	QCoreApplication::setOrganizationName(QStringLiteral("PointCloudProcess"));
	QCoreApplication::setApplicationName(QStringLiteral("PointCloudProcess"));
	const QString logDir = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("logs"));
	(void)RunLogger::initialize(logDir.toStdString(), "PointCloudProcess");
	RunLogger::info("Application bootstrap started.");
	MainWindow mainWindow;
	mainWindow.showMaximized();
	const int ret = a.exec();
	RunLogger::info("Application shutdown.");
	RunLogger::shutdown();
	return ret;
}
