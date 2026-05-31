#include "TrajectoryOpParamPanel.h"

#include "TrajectoryOpBridge.h"

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{

std::string scopeKindToken(const RobotInstruction::OpScope::Kind kind)
{
	switch (kind)
	{
	case RobotInstruction::OpScope::Kind::EntireProgram:
		return "EntireProgram";
	case RobotInstruction::OpScope::Kind::PointIndexRange:
		return "PointIndexRange";
	case RobotInstruction::OpScope::Kind::InstructionIds:
		return "InstructionIds";
	case RobotInstruction::OpScope::Kind::Group:
	default:
		return "Group";
	}
}

} // namespace

TrajectoryOpParamPanel::TrajectoryOpParamPanel(QWidget* parent)
	: QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	m_form = new QFormLayout();
	layout->addLayout(m_form);
}

void TrajectoryOpParamPanel::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
}

void TrajectoryOpParamPanel::setLoading(const bool loading)
{
	m_loading = loading;
}

void TrajectoryOpParamPanel::setScopeGroupCombo(QComboBox* combo)
{
	m_scopeGroupCombo = combo;
	m_scopeGroupComboParent = combo ? combo->parentWidget() : nullptr;
}

void TrajectoryOpParamPanel::clear()
{
	clearRows();
}

void TrajectoryOpParamPanel::clearRows()
{
	if (m_clearingRows)
	{
		return;
	}
	m_clearingRows = true;

	if (!m_form)
	{
		m_rows.clear();
		m_clearingRows = false;
		return;
	}

	QLabel* scopeLabel = nullptr;
	for (trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key == "scope.groupId")
		{
			scopeLabel = row.label;
			continue;
		}
		QLabel* lbl = row.label;
		QWidget* w = row.widget;
		if (lbl && m_form->indexOf(lbl) >= 0)
		{
			m_form->removeWidget(lbl);
		}
		if (w && m_form->indexOf(w) >= 0)
		{
			m_form->removeWidget(w);
		}
		delete lbl;
		delete w;
		row.label = nullptr;
		row.widget = nullptr;
	}

	// 先从布局摘掉 scope 标签再 delete；removeRow(combo) 可能已销毁同行 label
	if (scopeLabel)
	{
		if (m_form->indexOf(scopeLabel) >= 0)
		{
			m_form->removeWidget(scopeLabel);
		}
		delete scopeLabel;
		scopeLabel = nullptr;
	}
	// removeRow 会 delete 行内控件；scope 下拉需复用，只能 removeWidget
	if (m_scopeGroupCombo && m_form->indexOf(m_scopeGroupCombo) >= 0)
	{
		m_form->removeWidget(m_scopeGroupCombo);
	}
	if (m_scopeGroupCombo && m_scopeGroupComboParent)
	{
		m_scopeGroupCombo->setParent(m_scopeGroupComboParent);
	}

	m_rows.clear();
	m_clearingRows = false;
}

std::string TrajectoryOpParamPanel::currentScopeKindToken() const
{
	for (const trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key != "scope.kind" || !row.read)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamValue value{};
		if (row.read(value))
		{
			return scopeKindToken(static_cast<RobotInstruction::OpScope::Kind>(value.asInt));
		}
	}
	return "Group";
}

void TrajectoryOpParamPanel::updateFieldVisibility()
{
	const std::string scopeToken = currentScopeKindToken();
	for (trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key == "scope.groupId")
		{
			const bool visible = scopeToken == "Group";
			if (row.label)
			{
				row.label->setVisible(visible);
			}
			if (m_scopeGroupCombo)
			{
				m_scopeGroupCombo->setVisible(visible);
			}
			continue;
		}
		const bool visible = row.field.visibleWhenScopeKind.empty()
			|| row.field.visibleWhenScopeKind == scopeToken;
		if (row.label)
		{
			row.label->setVisible(visible);
		}
		if (row.widget)
		{
			row.widget->setVisible(visible);
		}
	}
}

void TrajectoryOpParamPanel::rebuildForOp(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const trajectory_algo::ITrajectoryOp* algo)
{
	if (m_rebuilding || m_clearingRows)
	{
		return;
	}
	m_rebuilding = true;
	const bool wasLoading = m_loading;
	m_loading = true;
	clearRows();
	if (!algo)
	{
		m_loading = wasLoading;
		m_rebuilding = false;
		return;
	}
	const std::vector<trajectory_algo::TrajectoryOpParamField> fields =
		RobotInstruction::trajectoryOpAllParamFields(*algo);
	for (const trajectory_algo::TrajectoryOpParamField& field : fields)
	{
		if (field.key == "scope.groupId")
		{
			if (m_scopeGroupCombo)
			{
				auto* label = new QLabel(m_useChinese ? QStringLiteral("分组") : QStringLiteral("Group"), this);
				m_form->addRow(label, m_scopeGroupCombo);
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_scopeGroupCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		trajectory_algo::TrajectoryParamBinding binding =
			trajectory_algo::TrajectoryParamWidgetFactory::create(field, m_useChinese);
		if (!binding.widget)
		{
			continue;
		}
		m_form->addRow(binding.label, binding.widget);
		trajectory_algo::TrajectoryParamValue value{};
		if (RobotInstruction::trajectoryOpParamRead(op, field, value) && binding.write)
		{
			binding.write(value);
		}
		if (field.type != trajectory_algo::TrajectoryParamType::Message)
		{
			QObject::connect(binding.widget, &QWidget::destroyed, this, []() {});
			if (auto* spin = binding.widget->findChild<QDoubleSpinBox*>())
			{
				spin->setKeyboardTracking(false);
				connect(spin, &QAbstractSpinBox::editingFinished, this, [this]() {
					if (!m_loading)
					{
						updateFieldVisibility();
						emit paramsChanged();
					}
				});
			}
			if (auto* spin = binding.widget->findChild<QSpinBox*>())
			{
				spin->setKeyboardTracking(false);
				connect(spin, &QAbstractSpinBox::editingFinished, this, [this]() {
					if (!m_loading)
					{
						updateFieldVisibility();
						emit paramsChanged();
					}
				});
			}
			if (auto* combo = binding.widget->findChild<QComboBox*>())
			{
				connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
					if (!m_loading)
					{
						updateFieldVisibility();
						emit paramsChanged();
					}
				});
			}
		}
		m_rows.push_back(std::move(binding));
	}
	updateFieldVisibility();
	m_loading = wasLoading;
	m_rebuilding = false;
}

bool TrajectoryOpParamPanel::applyTo(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const trajectory_algo::ITrajectoryOp* algo,
	std::string* errMsg)
{
	if (!algo || m_rebuilding)
	{
		return false;
	}
	for (const trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key == "scope.groupId")
		{
			if (m_scopeGroupCombo && m_scopeGroupCombo->currentIndex() >= 0)
			{
				op.scope.groupId = m_scopeGroupCombo->currentData().toString().toStdString();
			}
			continue;
		}
		if (!row.read)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamValue value{};
		if (!row.read(value))
		{
			continue;
		}
		RobotInstruction::trajectoryOpParamWrite(op, row.field, value);
	}
	return algo->validate(op, errMsg);
}
