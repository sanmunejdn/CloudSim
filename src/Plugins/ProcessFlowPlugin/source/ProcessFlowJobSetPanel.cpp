/// @file ProcessFlowJobSetPanel.cpp
/// @brief JobSet 编辑面板

#include "ProcessFlowJobSetPanel.h"

#include "ProcessFlowCanvasWidget.h"
#include "ProcessFlowNodeProps.h"
#include "sim/SimModelBuilder.h"
#include "sim/SimRunConfig.h"

#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

ProcessFlowJobSetPanel::ProcessFlowJobSetPanel(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 8, 0, 0);
	auto* title = new QLabel(QStringLiteral("JobSet 工艺"), this);
	title->setObjectName(QStringLiteral("ProcessFlowSectionTitle"));

	m_templates = new QListWidget(this);
	m_templates->setMaximumHeight(80);
	m_nameEdit = new QLineEdit(this);
	m_ops = new QTableWidget(0, 4, this);
	m_ops->setHorizontalHeaderLabels(
		{QStringLiteral("机器ID"), QStringLiteral("加工s"), QStringLiteral("换型s"), QStringLiteral("优先级")});
	m_ops->horizontalHeader()->setStretchLastSection(true);
	m_ops->setMaximumHeight(140);

	auto* row1 = new QHBoxLayout();
	m_addTpl = new QPushButton(QStringLiteral("加模板"), this);
	m_delTpl = new QPushButton(QStringLiteral("删模板"), this);
	m_genPath = new QPushButton(QStringLiteral("从路径生成"), this);
	row1->addWidget(m_addTpl);
	row1->addWidget(m_delTpl);
	row1->addWidget(m_genPath);

	auto* row2 = new QHBoxLayout();
	m_addOp = new QPushButton(QStringLiteral("加Op"), this);
	m_delOp = new QPushButton(QStringLiteral("删Op"), this);
	row2->addWidget(m_addOp);
	row2->addWidget(m_delOp);

	root->addWidget(title);
	root->addWidget(m_templates);
	root->addWidget(new QLabel(QStringLiteral("名称"), this));
	root->addWidget(m_nameEdit);
	root->addLayout(row1);
	root->addWidget(m_ops);
	root->addLayout(row2);

	connect(m_templates, &QListWidget::currentRowChanged, this, [this](int) { onTemplateSelectionChanged(); });
	connect(m_addTpl, &QPushButton::clicked, this, &ProcessFlowJobSetPanel::addTemplate);
	connect(m_delTpl, &QPushButton::clicked, this, &ProcessFlowJobSetPanel::removeTemplate);
	connect(m_genPath, &QPushButton::clicked, this, &ProcessFlowJobSetPanel::generateFromPath);
	connect(m_addOp, &QPushButton::clicked, this, &ProcessFlowJobSetPanel::addOpRow);
	connect(m_delOp, &QPushButton::clicked, this, &ProcessFlowJobSetPanel::removeOpRow);
	connect(m_nameEdit, &QLineEdit::editingFinished, this,
			[this]()
			{
				syncCurrentTemplateFromTable();
				emit jobSetChanged();
			});
	connect(m_ops, &QTableWidget::cellChanged, this,
			[this](int, int)
			{
				if (m_block)
					return;
				syncCurrentTemplateFromTable();
				emit jobSetChanged();
			});
}

void ProcessFlowJobSetPanel::applyLanguage(bool useChinese)
{
	m_zh = useChinese;
	m_addTpl->setText(useChinese ? QStringLiteral("加模板") : QStringLiteral("Add Template"));
	m_delTpl->setText(useChinese ? QStringLiteral("删模板") : QStringLiteral("Del Template"));
	m_genPath->setText(useChinese ? QStringLiteral("从路径生成") : QStringLiteral("From Path"));
	m_addOp->setText(useChinese ? QStringLiteral("加Op") : QStringLiteral("Add Op"));
	m_delOp->setText(useChinese ? QStringLiteral("删Op") : QStringLiteral("Del Op"));
}

void ProcessFlowJobSetPanel::setCanvas(ProcessFlowCanvasWidget* canvas)
{
	m_canvas = canvas;
}

void ProcessFlowJobSetPanel::loadFromJson(const QJsonObject& jobSet)
{
	m_jobSet = jobSet;
	m_block = true;
	m_templates->clear();
	const QJsonArray tpls = m_jobSet.value(QStringLiteral("templates")).toArray();
	for (int i = 0; i < tpls.size(); ++i)
	{
		const QString name = tpls[i].toObject().value(QStringLiteral("name")).toString(QStringLiteral("job%1").arg(i + 1));
		m_templates->addItem(name);
	}
	m_block = false;
	if (m_templates->count() > 0)
		m_templates->setCurrentRow(0);
	else
		rebuildOpTable();
}

QJsonObject ProcessFlowJobSetPanel::toJson() const
{
	return m_jobSet;
}

void ProcessFlowJobSetPanel::onTemplateSelectionChanged()
{
	rebuildOpTable();
}

void ProcessFlowJobSetPanel::rebuildOpTable()
{
	m_block = true;
	m_ops->setRowCount(0);
	const int row = m_templates->currentRow();
	QJsonArray tpls = m_jobSet.value(QStringLiteral("templates")).toArray();
	if (row < 0 || row >= tpls.size())
	{
		m_nameEdit->clear();
		m_block = false;
		return;
	}
	const QJsonObject tpl = tpls[row].toObject();
	m_nameEdit->setText(tpl.value(QStringLiteral("name")).toString());
	const QJsonArray ops = tpl.value(QStringLiteral("ops")).toArray();
	m_ops->setRowCount(ops.size());
	for (int r = 0; r < ops.size(); ++r)
	{
		const QJsonObject o = ops[r].toObject();
		m_ops->setItem(r, 0, new QTableWidgetItem(QString::number(o.value(QStringLiteral("machineNodeId")).toInt())));
		m_ops->setItem(r, 1, new QTableWidgetItem(QString::number(o.value(QStringLiteral("processTimeSec")).toDouble())));
		m_ops->setItem(r, 2, new QTableWidgetItem(QString::number(o.value(QStringLiteral("setupTimeSec")).toDouble())));
		m_ops->setItem(r, 3, new QTableWidgetItem(QString::number(o.value(QStringLiteral("priority")).toDouble())));
	}
	m_block = false;
}

void ProcessFlowJobSetPanel::syncCurrentTemplateFromTable()
{
	const int row = m_templates->currentRow();
	QJsonArray tpls = m_jobSet.value(QStringLiteral("templates")).toArray();
	if (row < 0 || row >= tpls.size())
		return;
	QJsonObject tpl = tpls[row].toObject();
	tpl.insert(QStringLiteral("name"), m_nameEdit->text().trimmed().isEmpty() ? QStringLiteral("job") : m_nameEdit->text());
	QJsonArray ops;
	for (int r = 0; r < m_ops->rowCount(); ++r)
	{
		QJsonObject o;
		o.insert(QStringLiteral("machineNodeId"), m_ops->item(r, 0) ? m_ops->item(r, 0)->text().toInt() : -1);
		o.insert(QStringLiteral("processTimeSec"), m_ops->item(r, 1) ? m_ops->item(r, 1)->text().toDouble() : 0.0);
		o.insert(QStringLiteral("setupTimeSec"), m_ops->item(r, 2) ? m_ops->item(r, 2)->text().toDouble() : 0.0);
		o.insert(QStringLiteral("priority"), m_ops->item(r, 3) ? m_ops->item(r, 3)->text().toDouble() : 0.0);
		ops.append(o);
	}
	tpl.insert(QStringLiteral("ops"), ops);
	tpls[row] = tpl;
	m_jobSet.insert(QStringLiteral("templates"), tpls);
	if (m_templates->item(row))
		m_templates->item(row)->setText(tpl.value(QStringLiteral("name")).toString());
}

void ProcessFlowJobSetPanel::addTemplate()
{
	QJsonArray tpls = m_jobSet.value(QStringLiteral("templates")).toArray();
	QJsonObject tpl;
	tpl.insert(QStringLiteral("name"), QStringLiteral("job%1").arg(tpls.size() + 1));
	tpl.insert(QStringLiteral("ops"), QJsonArray());
	tpls.append(tpl);
	m_jobSet.insert(QStringLiteral("templates"), tpls);
	m_templates->addItem(tpl.value(QStringLiteral("name")).toString());
	m_templates->setCurrentRow(m_templates->count() - 1);
	emit jobSetChanged();
}

void ProcessFlowJobSetPanel::removeTemplate()
{
	const int row = m_templates->currentRow();
	QJsonArray tpls = m_jobSet.value(QStringLiteral("templates")).toArray();
	if (row < 0 || row >= tpls.size())
		return;
	tpls.removeAt(row);
	m_jobSet.insert(QStringLiteral("templates"), tpls);
	delete m_templates->takeItem(row);
	emit jobSetChanged();
}

void ProcessFlowJobSetPanel::addOpRow()
{
	const int r = m_ops->rowCount();
	m_ops->insertRow(r);
	int mid = -1;
	if (m_canvas)
	{
		// 取第一个 machine 节点
		const QJsonArray nodes = m_canvas->toJson().value(QStringLiteral("nodes")).toArray();
		for (const QJsonValue& v : nodes)
		{
			const QJsonObject props = v.toObject().value(QStringLiteral("props")).toObject();
			if (ProcessFlowNodeProps::isMachineKind(props.value(QStringLiteral("kind")).toString()))
			{
				mid = v.toObject().value(QStringLiteral("id")).toInt();
				break;
			}
		}
	}
	m_ops->setItem(r, 0, new QTableWidgetItem(QString::number(mid)));
	m_ops->setItem(r, 1, new QTableWidgetItem(QStringLiteral("30")));
	m_ops->setItem(r, 2, new QTableWidgetItem(QStringLiteral("0")));
	m_ops->setItem(r, 3, new QTableWidgetItem(QStringLiteral("0")));
	syncCurrentTemplateFromTable();
	emit jobSetChanged();
}

void ProcessFlowJobSetPanel::removeOpRow()
{
	const int r = m_ops->currentRow();
	if (r < 0)
		return;
	m_ops->removeRow(r);
	syncCurrentTemplateFromTable();
	emit jobSetChanged();
}

void ProcessFlowJobSetPanel::generateFromPath()
{
	if (!m_canvas)
		return;
	SimRunConfig cfg;
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(m_canvas->toJson(), cfg);
	if (!built.ok || built.jobSet.templates.isEmpty())
		return;
	QJsonArray tpls;
	for (const auto& tmpl : built.jobSet.templates)
	{
		QJsonObject o;
		o.insert(QStringLiteral("name"), tmpl.name);
		QJsonArray ops;
		for (const auto& op : tmpl.ops)
		{
			QJsonObject oo;
			oo.insert(QStringLiteral("machineNodeId"), op.machineNodeId);
			oo.insert(QStringLiteral("processTimeSec"), op.processTimeSec);
			oo.insert(QStringLiteral("setupTimeSec"), op.setupTimeSec);
			oo.insert(QStringLiteral("priority"), op.priority);
			ops.append(oo);
		}
		o.insert(QStringLiteral("ops"), ops);
		tpls.append(o);
	}
	m_jobSet.insert(QStringLiteral("templates"), tpls);
	loadFromJson(m_jobSet);
	emit jobSetChanged();
}
