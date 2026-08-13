#ifndef ROBOTWIDGET_CUSTOMDEVICEAXISEDITORWIDGET_H
#define ROBOTWIDGET_CUSTOMDEVICEAXISEDITORWIDGET_H

/// @file CustomDeviceAxisEditorWidget.h
/// @brief 自定义设备多轴编辑：列表 CRUD + 旋转中心拾取入口

#include "robotwidget_global.h"

#include "CustomDeviceBackendData.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTimer;

class ROBOTWIDGET_EXPORT CustomDeviceAxisEditorWidget : public QWidget
{
	Q_OBJECT

public:
	static constexpr int kMaxAxes = 6;

	explicit CustomDeviceAxisEditorWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setAxes(const CustomDeviceAxisConfigSet& axes);
	CustomDeviceAxisConfigSet axes();

	int currentAxisIndex() const;
	bool currentAxisIsRotate() const;

	/// 写入当前选中旋转轴的局部原点（mm）
	void applyPickedOriginLocalMm(double x, double y, double z);
	/// 写入当前选中轴的方向（已单位化局部向量）
	void applyPickedAxisDirection(double x, double y, double z);
	bool useNormalAsAxis() const;

signals:
	void axesChanged();
	void pickOriginRequested();

private:
	void rebuildList();
	void loadFieldsFromSelection();
	void saveFieldsToSelection();
	void scheduleChanged();
	void onFieldChanged();
	void onListSelectionChanged();
	void onAddAxis();
	void onRemoveAxis();
	void onChangedDebounce();
	void refreshEmptyHint();
	void refreshMotionDependentUi();
	void refreshLimitSuffixes();

	bool m_useChinese = true;
	bool m_blockSignals = false;
	CustomDeviceAxisConfigSet m_axes;

	QLabel* m_emptyHint = nullptr;
	QGroupBox* m_editorGroup = nullptr;
	QFormLayout* m_form = nullptr;
	QListWidget* m_list = nullptr;
	QCheckBox* m_enabledCheck = nullptr;
	QLineEdit* m_nameEdit = nullptr;
	QComboBox* m_motionCombo = nullptr;
	QDoubleSpinBox* m_lowerSpin = nullptr;
	QDoubleSpinBox* m_upperSpin = nullptr;
	QDoubleSpinBox* m_homeSpin = nullptr;
	QDoubleSpinBox* m_axisSpin[3]{};
	QDoubleSpinBox* m_originSpin[3]{};
	QPushButton* m_addBtn = nullptr;
	QPushButton* m_removeBtn = nullptr;
	QPushButton* m_pickOriginBtn = nullptr;
	QCheckBox* m_useNormalAsAxisCheck = nullptr;
	QTimer* m_debounceTimer = nullptr;
};

#endif // ROBOTWIDGET_CUSTOMDEVICEAXISEDITORWIDGET_H
