#include <QtWidgets/QApplication>
#include <QCoreApplication>

#include "MainWindow.h"

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	QCoreApplication::setOrganizationName(QStringLiteral("PointCloudProcess"));
	QCoreApplication::setApplicationName(QStringLiteral("PointCloudProcess"));
	MainWindow mainWindow;
	mainWindow.showMaximized();
	return a.exec();
}
