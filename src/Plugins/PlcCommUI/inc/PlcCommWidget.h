#ifndef PLCCOMMUI_PLCCOMMWIDGET_H
#define PLCCOMMUI_PLCCOMMWIDGET_H

/// @file PlcCommWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PlcCommWidget 接口

#include "plc_comm_ui_global.h"

#include "PlcCommTypes.h"

#include <QByteArray>
#include <QHash>
#include <QWidget>

class PlcCommController;
class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTimer;

class PLCCOMM_UI_EXPORT PlcCommWidget : public QWidget
{
	Q_OBJECT

public:
	explicit PlcCommWidget(QWidget* parent = nullptr);
	~PlcCommWidget() override;

	void setUseChinese(bool chinese);
	void applyLanguage();

private slots:
	void onConnectClicked();
	void onDisconnectClicked();
	void onAddTagClicked();
	void onRemoveTagClicked();
	void onReadClicked();
	void onWriteClicked();
	void onPollTimeout();
	void onConnectedChanged(bool connected);
	void onTagAdded(int handle, const QString& name);
	void onTagRead(int handle, const QByteArray& data, bool ok);
	void onLogMessage(const QString& text);
	void onError(const QString& text);
	void onDisplayFormatChanged(int index);

private:
	enum class ValueFormat
	{
		Hex = 0,
		DecimalBytes,
		BinaryBytes,
		UInt16Le,
		Int32Le
	};

	QString i18n(const QString& en, const QString& zh) const;
	ValueFormat currentValueFormat() const;
	QString formatRawBytes(const QByteArray& data) const;
	bool parseValueText(const QString& text, QByteArray* out) const;
	void refreshDisplayedValue(int handle);
	void refreshAllDisplayedValues();
	void updateValueEditPlaceholder();
	PlcConnectionConfig currentConfig() const;
	int selectedHandle() const;
	void appendLog(const QString& line);
	void syncWorkerLanguage();
	void updateProtocolFields();

	bool useChinese_ = true;
	PlcCommController* controller_ = nullptr;
	QGroupBox* connGroup_ = nullptr;
	QGroupBox* tagGroup_ = nullptr;
	QLabel* protocolLabel_ = nullptr;
	QLabel* ipLabel_ = nullptr;
	QLabel* portLabel_ = nullptr;
	QLabel* pathLabel_ = nullptr;
	QLabel* cpuLabel_ = nullptr;
	QLabel* timeoutLabel_ = nullptr;
	QLabel* logCaption_ = nullptr;
	QComboBox* protocolCombo_ = nullptr;
	QLineEdit* gatewayEdit_ = nullptr;
	QSpinBox* portSpin_ = nullptr;
	QLineEdit* pathEdit_ = nullptr;
	QLineEdit* cpuEdit_ = nullptr;
	QPushButton* connectBtn_ = nullptr;
	QPushButton* disconnectBtn_ = nullptr;
	QTableWidget* tagTable_ = nullptr;
	QLineEdit* tagNameEdit_ = nullptr;
	QLabel* formatLabel_ = nullptr;
	QComboBox* formatCombo_ = nullptr;
	QLineEdit* valueEdit_ = nullptr;
	QPushButton* addTagBtn_ = nullptr;
	QPushButton* removeTagBtn_ = nullptr;
	QPushButton* readBtn_ = nullptr;
	QPushButton* writeBtn_ = nullptr;
	QCheckBox* pollCheck_ = nullptr;
	QSpinBox* pollIntervalSpin_ = nullptr;
	QSpinBox* timeoutSpin_ = nullptr;
	QPlainTextEdit* logEdit_ = nullptr;
	QTimer* pollTimer_ = nullptr;
	QHash<int, QByteArray> rawByHandle_;
};

PLCCOMM_UI_EXPORT QWidget* createPlcCommWidget(QWidget* parent = nullptr);

#endif // PLCCOMMUI_PLCCOMMWIDGET_H
