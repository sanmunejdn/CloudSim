#pragma once

#include <QWidget>

class IPluginHostContext;

class HelloDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit HelloDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void refreshBackendCount();

private:
	IPluginHostContext* m_host = nullptr;
	class QLabel* m_label = nullptr;
};
