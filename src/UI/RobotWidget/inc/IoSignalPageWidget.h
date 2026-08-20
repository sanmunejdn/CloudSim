#ifndef ROBOTWIDGET_IOSIGNALPAGEWIDGET_H
#define ROBOTWIDGET_IOSIGNALPAGEWIDGET_H

/// @file IoSignalPageWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 按 Owner 编辑自持信号表（Property Dock）

#include "robotwidget_global.h"

#include "NamedSignalTable.h"

#include <QPointer>
#include <QWidget>

class IoSignalNetworkService;
class NamedSignalIoSink;
class QComboBox;
class QDialog;
class QPushButton;
class QTableWidget;
class SignalConnectionStationWidget;

class ROBOTWIDGET_EXPORT IoSignalPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit IoSignalPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setNetwork(IoSignalNetworkService* network);
	void setCurrentOwnerId(const QString& ownerId);
	QString currentOwnerId() const;
	void refreshOwners();
	void refreshFromModel();

signals:
	void signalTableEdited();
	void currentOwnerChanged(const QString& ownerId);

private slots:
	void onOwnerChanged(int index);
	void onAddClicked();
	void onRemoveClicked();
	void onResetDefaultsClicked();
	void onOpenStationClicked();
	void onCellChanged(int row, int column);
	void onSinkValuesChanged();
	void onNetworkChanged();

private:
	void setupUi();
	void updateUiLabels();
	void bindCurrentOwner();
	void rebuildTable();
	void applyRowToModel(int row);
	void applyValueCellToSink(int row);
	void syncForceItemFlags(int row);
	void syncValueColumnsFromSink();
	void flushDeviceIfNeeded();
	RobotIo::SignalKind kindFromComboText(const QString& text) const;
	static bool parseBoolText(const QString& text);

	bool m_useChinese = true;
	bool m_updating = false;
	IoSignalNetworkService* m_network = nullptr;
	RobotIo::NamedSignalTable* m_table = nullptr;
	QPointer<NamedSignalIoSink> m_sink;

	QComboBox* m_ownerCombo = nullptr;
	QTableWidget* m_tableWidget = nullptr;
	QPushButton* m_addBtn = nullptr;
	QPushButton* m_removeBtn = nullptr;
	QPushButton* m_resetBtn = nullptr;
	QPushButton* m_stationBtn = nullptr;
	QPointer<QDialog> m_stationDialog;
	SignalConnectionStationWidget* m_stationWidget = nullptr;
};

#endif // ROBOTWIDGET_IOSIGNALPAGEWIDGET_H
