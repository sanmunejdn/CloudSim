#include "MainWindow.h"

#include <QtWidgets/QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <cstring>

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
		QDir(appDir).absoluteFilePath("../SDK/OSG3.6.5/bin"),
		QDir(appDir).absoluteFilePath("../SDK/OSG3.6.5/bin"),
		QDir(appDir).absoluteFilePath("OSG3.6.5/bin")
	};
	for (const QString& c : candidates)
	{
		prependPathIfExists(QDir(c).absolutePath());
	}
}

} // namespace
#endif

/// Sets \c ROBOT_KINEMATICS_DEBUG from argv so debugging works without pre-setting system env (Windows GUI).
static void applyRobotKinematicsDebugFromArgv(int argc, char* argv[])
{
	for (int i = 1; i < argc; ++i)
	{
		const char* arg = argv[i];
		if (std::strcmp(arg, "--robot-kinematics-debug") == 0)
		{
			const char* val = (i + 1 < argc && argv[i + 1][0] != '-') ? argv[++i] : "1";
			(void)qputenv("ROBOT_KINEMATICS_DEBUG", QByteArray(val));
		}
		else if (std::strncmp(arg, "--robot-kinematics-debug=", 26) == 0)
		{
			const char* val = arg + 26;
			if (val[0] != '\0')
			{
				(void)qputenv("ROBOT_KINEMATICS_DEBUG", QByteArray(val));
			}
		}
	}
}

int main(int argc, char* argv[])
{
	applyRobotKinematicsDebugFromArgv(argc, argv);
	// Default ON: per-link FK / OSG matrix dumps (Run info + logs). Turn off with env ROBOT_KINEMATICS_DEBUG=0
	// or e.g. --robot-kinematics-debug 0 before other args consume the value.
	if (qgetenv("ROBOT_KINEMATICS_DEBUG").isEmpty())
	{
		(void)qputenv("ROBOT_KINEMATICS_DEBUG", QByteArray("0"));
	}
	if (qgetenv("POINTCLOUD_GIZMO_PIVOT_DIAG").isEmpty())
	{
		(void)qputenv("POINTCLOUD_GIZMO_PIVOT_DIAG", QByteArray("0"));
	}
	if (qgetenv("POINTCLOUD_PROCESS_DEBUG").isEmpty())
	{
		(void)qputenv("POINTCLOUD_PROCESS_DEBUG", QByteArray("0"));
	}
	QApplication a(argc, argv);
#ifdef Q_OS_WIN
	configureWindowsDllSearchPath();
#endif
	QCoreApplication::setOrganizationName(QStringLiteral("CloudSim"));
	QCoreApplication::setApplicationName(QStringLiteral("CloudSim"));
	MainWindow mainWindow;
	mainWindow.showMaximized();
	const int ret = a.exec();
	MainWindow::shutdownApplicationLogging();
	return ret;
}
