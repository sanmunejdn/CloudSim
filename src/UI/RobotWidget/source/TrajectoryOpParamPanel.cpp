#include "TrajectoryOpParamPanel.h"

#include "TrajectoryOpBridge.h"

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
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

void applyFieldWidthPolicy(QWidget* widget)
{
	if (!widget)
	{
		return;
	}
	widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	for (QDoubleSpinBox* spin : widget->findChildren<QDoubleSpinBox*>())
	{
		spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	}
	for (QSpinBox* spin : widget->findChildren<QSpinBox*>())
	{
		spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	}
	for (QComboBox* combo : widget->findChildren<QComboBox*>())
	{
		combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	}
}

} // namespace

TrajectoryOpParamPanel::TrajectoryOpParamPanel(QWidget* parent)
	: QWidget(parent)
{
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);
	m_form = new QFormLayout();
	m_form->setContentsMargins(0, 0, 0, 0);
	m_form->setSpacing(1);
	m_form->setVerticalSpacing(1);
	m_form->setHorizontalSpacing(8);
	m_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
	m_form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	layout->addLayout(m_form, 1);
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

void TrajectoryOpParamPanel::setGeometryBackendCombo(QComboBox* combo)
{
	m_geometryBackendCombo = combo;
	m_geometryBackendComboParent = combo ? combo->parentWidget() : nullptr;
}

void TrajectoryOpParamPanel::setGeometryBackendPickButton(QPushButton* button)
{
	m_geometryBackendPickBtn = button;
	m_geometryBackendPickBtnParent = button ? button->parentWidget() : nullptr;
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
	QLabel* geometryBackendLabel = nullptr;
	for (trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key == "scope.groupId")
		{
			scopeLabel = row.label;
			continue;
		}
		if (row.field.key == "project.targetBackendId")
		{
			geometryBackendLabel = row.label;
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
	if (geometryBackendLabel)
	{
		if (m_form->indexOf(geometryBackendLabel) >= 0)
		{
			m_form->removeWidget(geometryBackendLabel);
		}
		delete geometryBackendLabel;
		geometryBackendLabel = nullptr;
	}
	if (m_geometryBackendCombo && m_form->indexOf(m_geometryBackendCombo) >= 0)
	{
		m_form->removeWidget(m_geometryBackendCombo);
	}
	if (m_geometryBackendCombo && m_geometryBackendComboParent)
	{
		m_geometryBackendCombo->setParent(m_geometryBackendComboParent);
	}
	if (m_geometryBackendPickBtn && m_form->indexOf(m_geometryBackendPickBtn) >= 0)
	{
		m_form->removeWidget(m_geometryBackendPickBtn);
	}
	if (m_geometryBackendPickBtn && m_geometryBackendPickBtnParent)
	{
		m_geometryBackendPickBtn->setParent(m_geometryBackendPickBtnParent);
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

int TrajectoryOpParamPanel::currentIntFieldValue(const std::string& key) const
{
	for (const trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key != key || !row.read)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamValue value{};
		if (row.read(value) && value.kind == trajectory_algo::TrajectoryParamValue::Kind::Int)
		{
			return value.asInt;
		}
	}
	return -1;
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
		bool visible = row.field.visibleWhenScopeKind.empty()
			|| row.field.visibleWhenScopeKind == scopeToken;
		if (visible && !row.field.visibleWhenFieldKey.empty() && row.field.visibleWhenIntValue >= 0)
		{
			visible = currentIntFieldValue(row.field.visibleWhenFieldKey) == row.field.visibleWhenIntValue;
		}
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
				applyFieldWidthPolicy(m_scopeGroupCombo);
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_scopeGroupCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		if (field.key == "project.targetBackendId")
		{
			if (m_geometryBackendCombo)
			{
				auto* label = new QLabel(
					m_useChinese ? QStringLiteral("几何对象") : QStringLiteral("Geometry Backend"),
					this);
				auto* rowWidget = new QWidget(this);
				applyFieldWidthPolicy(rowWidget);
				auto* rowLayout = new QHBoxLayout(rowWidget);
				rowLayout->setContentsMargins(0, 0, 0, 0);
				rowLayout->setSpacing(2);
				m_geometryBackendCombo->setFixedHeight(26);
				applyFieldWidthPolicy(m_geometryBackendCombo);
				rowLayout->addWidget(m_geometryBackendCombo, 1);
				if (m_geometryBackendPickBtn)
				{
					rowLayout->addWidget(m_geometryBackendPickBtn);
				}
				m_form->addRow(label, rowWidget);
				const int idx = m_geometryBackendCombo->findData(
					QString::fromStdString(op.project.targetBackendId));
				if (idx >= 0)
				{
					m_geometryBackendCombo->setCurrentIndex(idx);
				}
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_geometryBackendCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		if (field.type == trajectory_algo::TrajectoryParamType::Message)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamBinding binding =
			trajectory_algo::TrajectoryParamWidgetFactory::create(field, m_useChinese);
		if (!binding.widget)
		{
			continue;
		}
		m_form->addRow(binding.label, binding.widget);
		applyFieldWidthPolicy(binding.widget);
		if (field.type == trajectory_algo::TrajectoryParamType::Vec3 && binding.writeVec3)
		{
			trajectory_algo::TrajectoryOpParamField fx = field;
			fx.key = field.key + field.vec3SuffixX;
			trajectory_algo::TrajectoryParamValue vx{};
			double x = field.defaultDouble;
			double y = 0.0;
			double z = -1.0;
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) && vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
			{
				x = vx.asDouble;
			}
			fx.key = field.key + field.vec3SuffixY;
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) && vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
			{
				y = vx.asDouble;
			}
			fx.key = field.key + field.vec3SuffixZ;
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) && vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
			{
				z = vx.asDouble;
			}
			binding.writeVec3(x, y, z);
		}
		else
		{
			trajectory_algo::TrajectoryParamValue value{};
			if (RobotInstruction::trajectoryOpParamRead(op, field, value) && binding.write)
			{
				binding.write(value);
			}
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
		if (row.field.key == "project.targetBackendId")
		{
			const int comboIdx = m_geometryBackendCombo ? m_geometryBackendCombo->currentIndex() : -1;
			if (m_geometryBackendCombo && comboIdx >= 0)
			{
				op.project.targetBackendId =
					m_geometryBackendCombo->currentData().toString().toStdString();
			}
			continue;
		}
		if (row.field.type == trajectory_algo::TrajectoryParamType::Vec3 && row.readVec3)
		{
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			if (!row.readVec3(x, y, z))
			{
				continue;
			}
			trajectory_algo::TrajectoryOpParamField fx = row.field;
			fx.type = trajectory_algo::TrajectoryParamType::Double;
			trajectory_algo::TrajectoryParamValue vx{};
			vx.kind = trajectory_algo::TrajectoryParamValue::Kind::Double;
			fx.key = row.field.key + row.field.vec3SuffixX;
			vx.asDouble = x;
			RobotInstruction::trajectoryOpParamWrite(op, fx, vx);
			fx.key = row.field.key + row.field.vec3SuffixY;
			vx.asDouble = y;
			RobotInstruction::trajectoryOpParamWrite(op, fx, vx);
			fx.key = row.field.key + row.field.vec3SuffixZ;
			vx.asDouble = z;
			RobotInstruction::trajectoryOpParamWrite(op, fx, vx);
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
