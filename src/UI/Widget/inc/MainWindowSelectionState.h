#ifndef WIDGET_MAINWINDOWSELECTIONSTATE_H
#define WIDGET_MAINWINDOWSELECTIONSTATE_H

/// @file MainWindowSelectionState.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief MainWindow 层的轻量选择状态容器，避免选择字段散落在窗口成员中。

#include <QString>

/// MainWindow 层的轻量选择状态容器，避免选择字段散落在窗口成员中。
class MainWindowSelectionState
{
public:
	bool hasBackendSelection() const { return !m_selectedBackendId.isEmpty(); }
	const QString& selectedBackendId() const { return m_selectedBackendId; }

	void setSelectedBackendId(const QString& backendId) { m_selectedBackendId = backendId; }
	void clearBackendSelection() { m_selectedBackendId.clear(); }

private:
	QString m_selectedBackendId;
};

#endif // WIDGET_MAINWINDOWSELECTIONSTATE_H
