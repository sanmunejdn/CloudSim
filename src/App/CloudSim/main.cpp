#include "CloudSimBootstrap.h"
#include "ICloudSimContext.h"
#include "MainWindow.h"

#include <QtWidgets/QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <cstring>
#include <memory>

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
	// OSG 运行时优先于系统 PATH（如 osg161-*.dll）
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

/// 从 argv 设 ROBOT_KINEMATICS_DEBUG（Windows GUI 无预置 env）
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
	// 默认开 per-link FK/OSG 矩阵 dump；ROBOT_KINEMATICS_DEBUG=0 关
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
	// 组合根：后续 MainWindow / DocumentPage 注入 g_appContext
	cloudsimSetApplicationContext(cloudsimCreateApplicationContext());
	MainWindow mainWindow(cloudsimApplicationContext()->events());
	mainWindow.showMaximized();
	const int ret = a.exec();
	MainWindow::shutdownApplicationLogging();
	return ret;
}
