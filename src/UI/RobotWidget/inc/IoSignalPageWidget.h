#ifndef ROBOTWIDGET_IOSIGNALPAGEWIDGET_H
#define ROBOTWIDGET_IOSIGNALPAGEWIDGET_H

/// @file IoSignalPageWidget.h
/// @brief 信号定义/监控页（Property Dock，与设备同级）

#include "robotwidget_global.h"

#include "NamedSignalTable.h"

#include <QWidget>

class NamedSignalIoSink;
class QPushButton;
class QTableWidget;
class QVBoxLayout;

/// 信号定义/监控页（Property Dock，与设备同级）
class ROBOTWIDGET_EXPORT IoSignalPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit IoSignalPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setSignalTable(RobotIo::NamedSignalTable* table);
	void setIoSink(NamedSignalIoSink* sink);
	void refreshFromModel();

signals:
	void signalTableEdited();

private slots:
	void onAddClicked();
	void onRemoveClicked();
	void onResetDefaultsClicked();
	void onCellChanged(int row, int column);
	void onSinkValuesChanged();

private:
	void setupUi();
	void updateUiLabels();
	void rebuildTable();
	void applyRowToModel(int row);
	void applyValueCellToSink(int row);
	void syncForceItemFlags(int row);
	void syncValueColumnsFromSink();
	RobotIo::SignalKind kindFromComboText(const QString& text) const;
	static bool parseBoolText(const QString& text);

	bool m_useChinese = true;
	bool m_updating = false;
	RobotIo::NamedSignalTable* m_table = nullptr;
	NamedSignalIoSink* m_sink = nullptr;

	QTableWidget* m_tableWidget = nullptr;
	QPushButton* m_addBtn = nullptr;
	QPushButton* m_removeBtn = nullptr;
	QPushButton* m_resetBtn = nullptr;
};

#endif // ROBOTWIDGET_IOSIGNALPAGEWIDGET_H
