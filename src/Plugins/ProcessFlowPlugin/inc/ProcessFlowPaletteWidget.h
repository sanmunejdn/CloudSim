#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWPALETTEWIDGET_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWPALETTEWIDGET_H

/// @file ProcessFlowPaletteWidget.h
/// @brief 左侧：节点库（拖放/双击）+ 属性面板

#include <QColor>
#include <QWidget>

class ProcessFlowPropertyPanel;
class QLabel;
class QListWidget;

class ProcessFlowPaletteWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowPaletteWidget(QWidget* parent = nullptr);

	void applyLanguage(bool useChinese);
	ProcessFlowPropertyPanel* propertyPanel() const { return m_propertyPanel; }

signals:
	void addNodeRequested(const QString& kind, const QString& title, const QString& subtitle, const QColor& color);

private:
	void emitSelectedType();

	QLabel* m_title = nullptr;
	QLabel* m_hint = nullptr;
	QListWidget* m_list = nullptr;
	ProcessFlowPropertyPanel* m_propertyPanel = nullptr;
};

#endif
