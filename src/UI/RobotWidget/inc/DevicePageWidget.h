#ifndef ROBOTWIDGET_DEVICEPAGEWIDGET_H
#define ROBOTWIDGET_DEVICEPAGEWIDGET_H

/// @file DevicePageWidget.h
/// @brief 设备页：顶栏类型/品牌 Combo + 自适应缩略图网格，数据来自 resource/models

#include "robotwidget_global.h"

#include <QMap>
#include <QString>
#include <QVector>
#include <QWidget>

class QComboBox;
class QGridLayout;
class QLabel;
class QPushButton;
class QScrollArea;
class QToolButton;
class QVBoxLayout;

/// 设备页：顶栏类型/品牌 Combo + 自适应缩略图网格，数据来自 resource/models
class ROBOTWIDGET_EXPORT DevicePageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit DevicePageWidget(QWidget* parent = nullptr);
	void setModelsRootPath(const QString& absoluteDirPath);
	QString modelsRootPath() const { return m_modelsRoot; }
	void refreshButtons();
	void setUseChinese(bool chinese);

signals:
	void urdfImportRequested(const QString& urdfAbsolutePath);
	void customDeviceCreateRequested();
	void customDeviceEditRequested();
	void customDeviceExportUrdfRequested();

protected:
	void resizeEvent(QResizeEvent* event) override;
	bool eventFilter(QObject* watched, QEvent* event) override;

private:
	void setupUi(QVBoxLayout* rootLayout);
	void rescanPackagesAndRefreshUi();
	void fillTypeCombo();
	void fillBrandComboForSelectedType();
	void updateBrandComboVisibility();
	void rebuildModelTiles();
	void relayoutModelGrid();
	void updateUiLabels();
	QString selectedType() const;
	QString selectedBrand() const;
	void onTypeSelectionChanged();
	void onBrandSelectionChanged();
	void onRefreshClicked();

	QString m_modelsRoot;
	/// type → brand → 包根路径列表
	QMap<QString, QMap<QString, QStringList>> m_packagesByTypeBrand;

	bool m_useChinese = true;

	QLabel* m_typeLabel = nullptr;
	QLabel* m_brandLabel = nullptr;
	QComboBox* m_typeCombo = nullptr;
	QComboBox* m_brandCombo = nullptr;
	QPushButton* m_refreshBtn = nullptr;
	QPushButton* m_customDeviceBtn = nullptr;
	QPushButton* m_editCustomDeviceBtn = nullptr;
	QPushButton* m_exportCustomDeviceUrdfBtn = nullptr;
	QScrollArea* m_modelsScroll = nullptr;
	QWidget* m_modelsContainer = nullptr;
	QGridLayout* m_modelsGrid = nullptr;
	QLabel* m_statusLabel = nullptr;
	QVector<QToolButton*> m_modelButtons;
};

#endif // ROBOTWIDGET_DEVICEPAGEWIDGET_H
