#pragma once

#include <QMap>
#include <QString>
#include <QWidget>

class QListWidget;
class QVBoxLayout;
class QScrollArea;

/// 设备页：三列——设备类型 | 设备品牌 | 具体型号（缩略图按钮），数据来自 resource/models 目录树。
class DevicePageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit DevicePageWidget(QWidget* parent = nullptr);
	void setModelsRootPath(const QString& absoluteDirPath);
	QString modelsRootPath() const { return m_modelsRoot; }
	void refreshButtons();

signals:
	void urdfImportRequested(const QString& urdfAbsolutePath);

private:
	void setupDeviceColumns(QVBoxLayout* rootLayout);
	void rescanPackagesAndRefreshUi();
	void fillTypeList();
	void fillBrandListForSelectedType();
	void fillModelGridForSelection();
	void onTypeSelectionChanged();
	void onBrandSelectionChanged();

	QString m_modelsRoot;
	/// type -> brand -> list of package root paths (canonical)
	QMap<QString, QMap<QString, QStringList>> m_packagesByTypeBrand;

	QListWidget* m_listType = nullptr;
	QListWidget* m_listBrand = nullptr;
	QScrollArea* m_modelsScroll = nullptr;
	QWidget* m_modelsContainer = nullptr;
	QVBoxLayout* m_modelsLayout = nullptr;
};
