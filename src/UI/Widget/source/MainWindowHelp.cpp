/// @file MainWindowHelp.cpp
/// @brief 帮助菜单：打开本地 HTML 文档与关于对话框

#include "HelpBrowserDialog.h"
#include "MainWindow.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>

void MainWindow::onOpenHelpDocumentation()
{
	const QString langDir = m_useChinese ? QStringLiteral("zh") : QStringLiteral("en");
	const QString htmlPath =
		QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("help/%1/index.html").arg(langDir));

	if (!QFileInfo::exists(htmlPath))
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Help"), QStringLiteral("帮助")),
			i18n(QStringLiteral("Help documentation was not found:\n%1").arg(htmlPath),
				 QStringLiteral("未找到帮助文档：\n%1").arg(htmlPath)));
		return;
	}

	auto* dialog = new HelpBrowserDialog(this, i18n(QStringLiteral("Help Documentation"), QStringLiteral("帮助文档")),
										 htmlPath, m_useChinese);
	dialog->show();
}

void MainWindow::onAboutCloudSim()
{
	QMessageBox::about(
		this, i18n(QStringLiteral("About CloudSim"), QStringLiteral("关于 CloudSim")),
		i18n(QStringLiteral("<b>CloudSim</b><br/>Desktop application for industrial robot simulation.<br/><br/>"
							"See Help &rarr; Documentation for more information."),
			 QStringLiteral("<b>CloudSim</b><br/>面向工业机器人仿真的桌面应用。<br/><br/>"
							"更多说明见「帮助 → 帮助文档」。")));
}
