#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWPROPERTYPANEL_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWPROPERTYPANEL_H

/// @file ProcessFlowPropertyPanel.h
/// @brief 选中节点属性编辑（按 kind 显隐）

#include "ProcessFlowNodeProps.h"

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;

class ProcessFlowPropertyPanel final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowPropertyPanel(QWidget* parent = nullptr);

	void applyLanguage(bool useChinese);
	void clearSelection();
	void setNodeProps(int nodeId, const ProcessFlowNodeProps& props);
	int currentNodeId() const { return m_nodeId; }

signals:
	void propsEdited(int nodeId, const ProcessFlowNodeProps& props);

private:
	void emitEdited();
	void onKindChanged(int index);
	void updateFieldVisibility(const QString& kind);
	void setSpin(QDoubleSpinBox* spin, double v);

	int m_nodeId = -1;
	bool m_block = false;
	bool m_useChinese = true;
	QFormLayout* m_form = nullptr;
	QLabel* m_titleLabel = nullptr;
	QLabel* m_kindLabel = nullptr;
	QLabel* m_timeLabel = nullptr;
	QLabel* m_invLabel = nullptr;
	QLabel* m_capLabel = nullptr;
	QLabel* m_setupLabel = nullptr;
	QLabel* m_priorityLabel = nullptr;
	QLabel* m_batchLabel = nullptr;
	QLabel* m_scrapLabel = nullptr;
	QLabel* m_mtbfLabel = nullptr;
	QLabel* m_mttrLabel = nullptr;
	QLabel* m_inputsLabel = nullptr;
	QLabel* m_bindBackendLabel = nullptr;
	QLabel* m_bindProgramLabel = nullptr;
	QComboBox* m_kindCombo = nullptr;
	QDoubleSpinBox* m_cycleSpin = nullptr;
	QDoubleSpinBox* m_inventorySpin = nullptr;
	QDoubleSpinBox* m_capacitySpin = nullptr;
	QDoubleSpinBox* m_setupSpin = nullptr;
	QDoubleSpinBox* m_prioritySpin = nullptr;
	QDoubleSpinBox* m_batchSpin = nullptr;
	QDoubleSpinBox* m_scrapSpin = nullptr;
	QDoubleSpinBox* m_mtbfSpin = nullptr;
	QDoubleSpinBox* m_mttrSpin = nullptr;
	QDoubleSpinBox* m_inputsSpin = nullptr;
	class QLineEdit* m_bindBackendEdit = nullptr;
	class QLineEdit* m_bindProgramEdit = nullptr;
};

#endif
