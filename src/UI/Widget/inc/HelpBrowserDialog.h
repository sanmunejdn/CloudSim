#ifndef WIDGET_HELPBROWSERDIALOG_H
#define WIDGET_HELPBROWSERDIALOG_H

/// @file HelpBrowserDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 内嵌 HTML 帮助浏览器（左侧分级目录）

#include <QDialog>
#include <QString>

class QTextBrowser;
class QTreeWidget;
class QTreeWidgetItem;

/// 本地 HTML 帮助阅读器（不依赖 QHelp）
class HelpBrowserDialog : public QDialog
{
	Q_OBJECT

public:
	HelpBrowserDialog(QWidget* parent, const QString& title, const QString& htmlFilePath, bool useChinese);

private:
	void buildTocTree();
	void loadPage(const QString& fileName);
	void selectTocByFileName(const QString& fileName);
	void onTocItemActivated(QTreeWidgetItem* item, int column);
	void onTocCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);

	QTextBrowser* m_browser = nullptr;
	QTreeWidget* m_tocTree = nullptr;
	QString m_helpLangDir;
	bool m_useChinese = true;
	bool m_syncingToc = false;
};

#endif // WIDGET_HELPBROWSERDIALOG_H
