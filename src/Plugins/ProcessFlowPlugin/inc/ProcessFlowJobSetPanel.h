#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWJOBSETPANEL_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWJOBSETPANEL_H

/// @file ProcessFlowJobSetPanel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 多工艺 JobSet 编辑

#include <QJsonObject>
#include <QWidget>

class ProcessFlowCanvasWidget;
class QListWidget;
class QPushButton;
class QTableWidget;
class QLineEdit;

class ProcessFlowJobSetPanel final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowJobSetPanel(QWidget* parent = nullptr);

	void applyLanguage(bool useChinese);
	void setCanvas(ProcessFlowCanvasWidget* canvas);
	void loadFromJson(const QJsonObject& jobSet);
	QJsonObject toJson() const;

signals:
	void jobSetChanged();

public slots:
	void generateFromPath();

private:
	void rebuildOpTable();
	void onTemplateSelectionChanged();
	void addTemplate();
	void removeTemplate();
	void addOpRow();
	void removeOpRow();
	void syncCurrentTemplateFromTable();

	ProcessFlowCanvasWidget* m_canvas = nullptr;
	QListWidget* m_templates = nullptr;
	QLineEdit* m_nameEdit = nullptr;
	QTableWidget* m_ops = nullptr;
	QPushButton* m_addTpl = nullptr;
	QPushButton* m_delTpl = nullptr;
	QPushButton* m_genPath = nullptr;
	QPushButton* m_addOp = nullptr;
	QPushButton* m_delOp = nullptr;
	QJsonObject m_jobSet;
	bool m_zh = true;
	bool m_block = false;
};

#endif
