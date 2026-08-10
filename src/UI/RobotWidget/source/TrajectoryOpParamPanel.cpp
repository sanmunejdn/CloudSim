/// @file TrajectoryOpParamPanel.cpp
/// @brief TrajectoryOpParamPanel 实现

#include "TrajectoryOpParamPanel.h"

#include "TrajectoryOpBridge.h"

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>

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

TrajectoryOpParamPanel::TrajectoryOpParamPanel(QWidget* parent) : QWidget(parent)
{
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(2);
	m_form = new QFormLayout();
	m_form->setContentsMargins(0, 0, 0, 0);
	m_form->setSpacing(1);
	m_form->setVerticalSpacing(1);
	m_form->setHorizontalSpacing(8);
	m_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
	m_form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	layout->addLayout(m_form, 1);
	m_scopeHintLabel = new QLabel(this);
	m_scopeHintLabel->setWordWrap(true);
	m_scopeHintLabel->setVisible(false);
	m_scopeHintLabel->setStyleSheet(QStringLiteral("color: #288cf0;"));
	layout->addWidget(m_scopeHintLabel);
}

void TrajectoryOpParamPanel::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	updateScopeHint();
}

void TrajectoryOpParamPanel::setLoading(const bool loading)
{
	m_loading = loading;
}

void TrajectoryOpParamPanel::setPointIndexLimit(const int pointCount)
{
	m_pointIndexLimit = std::max(0, pointCount);
	applyPointIndexLimitToSpins();
	fillPointRangeIfNeeded();
	updateScopeHint();
}

void TrajectoryOpParamPanel::setEditingRawCloud(const bool editingRaw)
{
	m_editingRawCloud = editingRaw;
	updateScopeHint();
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

void TrajectoryOpParamPanel::setNonRigidSourceBackendCombo(QComboBox* combo)
{
	m_nonRigidSourceCombo = combo;
	m_nonRigidSourceComboParent = combo ? combo->parentWidget() : nullptr;
}

void TrajectoryOpParamPanel::setNonRigidTargetBackendCombo(QComboBox* combo)
{
	m_nonRigidTargetCombo = combo;
	m_nonRigidTargetComboParent = combo ? combo->parentWidget() : nullptr;
}

void TrajectoryOpParamPanel::setExternalTcpBackendCombo(QComboBox* combo)
{
	m_externalTcpBackendCombo = combo;
	m_externalTcpBackendComboParent = combo ? combo->parentWidget() : nullptr;
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
	QLabel* nonRigidSourceLabel = nullptr;
	QLabel* nonRigidTargetLabel = nullptr;
	QLabel* externalTcpLabel = nullptr;
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
		if (row.field.key == "nrr.sourceBackendId")
		{
			nonRigidSourceLabel = row.label;
			continue;
		}
		if (row.field.key == "nrr.targetBackendId")
		{
			nonRigidTargetLabel = row.label;
			continue;
		}
		if (row.field.key == "toWorkpiece.externalTcpBackendId")
		{
			externalTcpLabel = row.label;
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
	if (nonRigidSourceLabel)
	{
		if (m_form->indexOf(nonRigidSourceLabel) >= 0)
		{
			m_form->removeWidget(nonRigidSourceLabel);
		}
		delete nonRigidSourceLabel;
		nonRigidSourceLabel = nullptr;
	}
	if (m_nonRigidSourceCombo && m_form->indexOf(m_nonRigidSourceCombo) >= 0)
	{
		m_form->removeWidget(m_nonRigidSourceCombo);
	}
	if (m_nonRigidSourceCombo && m_nonRigidSourceComboParent)
	{
		m_nonRigidSourceCombo->setParent(m_nonRigidSourceComboParent);
	}
	if (nonRigidTargetLabel)
	{
		if (m_form->indexOf(nonRigidTargetLabel) >= 0)
		{
			m_form->removeWidget(nonRigidTargetLabel);
		}
		delete nonRigidTargetLabel;
		nonRigidTargetLabel = nullptr;
	}
	if (m_nonRigidTargetCombo && m_form->indexOf(m_nonRigidTargetCombo) >= 0)
	{
		m_form->removeWidget(m_nonRigidTargetCombo);
	}
	if (m_nonRigidTargetCombo && m_nonRigidTargetComboParent)
	{
		m_nonRigidTargetCombo->setParent(m_nonRigidTargetComboParent);
	}
	if (externalTcpLabel)
	{
		if (m_form->indexOf(externalTcpLabel) >= 0)
		{
			m_form->removeWidget(externalTcpLabel);
		}
		delete externalTcpLabel;
		externalTcpLabel = nullptr;
	}
	if (m_externalTcpBackendCombo && m_form->indexOf(m_externalTcpBackendCombo) >= 0)
	{
		m_form->removeWidget(m_externalTcpBackendCombo);
	}
	if (m_externalTcpBackendCombo && m_externalTcpBackendComboParent)
	{
		m_externalTcpBackendCombo->setParent(m_externalTcpBackendComboParent);
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
		bool visible = row.field.visibleWhenScopeKind.empty() || row.field.visibleWhenScopeKind == scopeToken;
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
	if (scopeToken == "PointIndexRange")
	{
		fillPointRangeIfNeeded();
	}
	updateScopeHint();
}

void TrajectoryOpParamPanel::applyPointIndexLimitToSpins()
{
	const int maxN = std::max(1, m_pointIndexLimit);
	for (trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (row.field.key != "scope.pointFrom" && row.field.key != "scope.pointTo")
		{
			continue;
		}
		if (!row.widget)
		{
			continue;
		}
		if (auto* spin = row.widget->findChild<QSpinBox*>())
		{
			spin->setMinimum(1);
			spin->setMaximum(maxN);
			if (spin->value() > maxN)
			{
				spin->setValue(maxN);
			}
			if (spin->value() < 1)
			{
				spin->setValue(1);
			}
		}
	}
}

void TrajectoryOpParamPanel::fillPointRangeIfNeeded()
{
	if (m_pointIndexLimit <= 0 || currentScopeKindToken() != "PointIndexRange")
	{
		return;
	}
	const int maxN = m_pointIndexLimit;
	const int from = currentIntFieldValue("scope.pointFrom");
	const int to = currentIntFieldValue("scope.pointTo");
	const bool needFill = (to <= 1 && maxN > 1) || from < 1 || to < from || to > maxN || from > maxN;
	if (!needFill)
	{
		applyPointIndexLimitToSpins();
		return;
	}
	for (trajectory_algo::TrajectoryParamBinding& row : m_rows)
	{
		if (!row.widget || !row.write)
		{
			continue;
		}
		trajectory_algo::TrajectoryParamValue value{};
		value.kind = trajectory_algo::TrajectoryParamValue::Kind::Int;
		if (row.field.key == "scope.pointFrom")
		{
			value.asInt = 1;
			row.write(value);
		}
		else if (row.field.key == "scope.pointTo")
		{
			value.asInt = maxN;
			row.write(value);
		}
	}
	applyPointIndexLimitToSpins();
}

void TrajectoryOpParamPanel::updateScopeHint()
{
	if (!m_scopeHintLabel)
	{
		return;
	}
	const std::string scopeToken = currentScopeKindToken();
	if (m_editingRawCloud && scopeToken == "Group")
	{
		m_scopeHintLabel->setText(
			m_useChinese ? QStringLiteral("离散点阶段请用「P 范围」；分组作用于生成程序后的指令组")
						 : QStringLiteral("On discrete points use Point range; Group applies after emit to program"));
		m_scopeHintLabel->setVisible(true);
		return;
	}
	if (scopeToken == "PointIndexRange" && m_pointIndexLimit > 0)
	{
		m_scopeHintLabel->setText(m_useChinese
									  ? QStringLiteral("P 范围为离散点序号 1…%1，算子仅作用于该区间")
											.arg(m_pointIndexLimit)
									  : QStringLiteral("Point range is 1…%1 on the discrete cloud").arg(m_pointIndexLimit));
		m_scopeHintLabel->setVisible(true);
		return;
	}
	m_scopeHintLabel->clear();
	m_scopeHintLabel->setVisible(false);
}

void TrajectoryOpParamPanel::rebuildForOp(const RobotInstruction::TrajectoryOpDescriptor& op,
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
				auto* label =
					new QLabel(m_useChinese ? QStringLiteral("几何对象") : QStringLiteral("Geometry Backend"), this);
				m_geometryBackendCombo->setFixedHeight(26);
				applyFieldWidthPolicy(m_geometryBackendCombo);
				m_form->addRow(label, m_geometryBackendCombo);
				const std::string targetBackendId = RobotInstruction::trajectoryOpProjectTargetBackendId(op);
				const int idx = m_geometryBackendCombo->findData(QString::fromStdString(targetBackendId));
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
		if (field.key == "nrr.sourceBackendId")
		{
			if (m_nonRigidSourceCombo)
			{
				auto* label =
					new QLabel(m_useChinese ? QStringLiteral("源几何") : QStringLiteral("Source Geometry"), this);
				m_nonRigidSourceCombo->setFixedHeight(26);
				applyFieldWidthPolicy(m_nonRigidSourceCombo);
				m_form->addRow(label, m_nonRigidSourceCombo);
				const std::string backendId = RobotInstruction::trajectoryOpNonRigidSourceBackendId(op);
				const int idx = m_nonRigidSourceCombo->findData(QString::fromStdString(backendId));
				if (idx >= 0)
				{
					m_nonRigidSourceCombo->setCurrentIndex(idx);
				}
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_nonRigidSourceCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		if (field.key == "nrr.targetBackendId")
		{
			if (m_nonRigidTargetCombo)
			{
				auto* label =
					new QLabel(m_useChinese ? QStringLiteral("目标几何") : QStringLiteral("Target Geometry"), this);
				m_nonRigidTargetCombo->setFixedHeight(26);
				applyFieldWidthPolicy(m_nonRigidTargetCombo);
				m_form->addRow(label, m_nonRigidTargetCombo);
				const std::string backendId = RobotInstruction::trajectoryOpNonRigidTargetBackendId(op);
				const int idx = m_nonRigidTargetCombo->findData(QString::fromStdString(backendId));
				if (idx >= 0)
				{
					m_nonRigidTargetCombo->setCurrentIndex(idx);
				}
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_nonRigidTargetCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		if (field.key == "toWorkpiece.externalTcpBackendId")
		{
			if (m_externalTcpBackendCombo)
			{
				auto* label = new QLabel(m_useChinese ? QStringLiteral("外部 TCP 坐标系")
													  : QStringLiteral("External TCP Frame"),
										 this);
				m_externalTcpBackendCombo->setFixedHeight(26);
				applyFieldWidthPolicy(m_externalTcpBackendCombo);
				m_form->addRow(label, m_externalTcpBackendCombo);
				const std::string backendId = RobotInstruction::trajectoryOpToWorkpieceExternalTcpBackendId(op);
				const int idx = m_externalTcpBackendCombo->findData(QString::fromStdString(backendId));
				if (idx >= 0)
				{
					m_externalTcpBackendCombo->setCurrentIndex(idx);
				}
				else if (m_externalTcpBackendCombo->count() > 0)
				{
					m_externalTcpBackendCombo->setCurrentIndex(0);
				}
				trajectory_algo::TrajectoryParamBinding binding{};
				binding.label = label;
				binding.widget = m_externalTcpBackendCombo;
				binding.field = field;
				m_rows.push_back(binding);
			}
			continue;
		}
		// 选中坐标系后隐藏手动六参数
		if (field.key == "toWorkpiece.externalTcpXMm" || field.key == "toWorkpiece.externalTcpYMm"
			|| field.key == "toWorkpiece.externalTcpZMm" || field.key == "toWorkpiece.externalTcpRxDeg"
			|| field.key == "toWorkpiece.externalTcpRyDeg" || field.key == "toWorkpiece.externalTcpRzDeg")
		{
			const std::string tcpBackendId = RobotInstruction::trajectoryOpToWorkpieceExternalTcpBackendId(op);
			if (!tcpBackendId.empty())
			{
				continue;
			}
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
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) &&
				vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
			{
				x = vx.asDouble;
			}
			fx.key = field.key + field.vec3SuffixY;
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) &&
				vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
			{
				y = vx.asDouble;
			}
			fx.key = field.key + field.vec3SuffixZ;
			if (RobotInstruction::trajectoryOpParamRead(op, fx, vx) &&
				vx.kind == trajectory_algo::TrajectoryParamValue::Kind::Double)
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
				connect(spin, &QAbstractSpinBox::editingFinished, this,
						[this]()
						{
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
				connect(spin, &QAbstractSpinBox::editingFinished, this,
						[this]()
						{
							if (!m_loading)
							{
								updateFieldVisibility();
								emit paramsChanged();
							}
						});
			}
			if (auto* combo = binding.widget->findChild<QComboBox*>())
			{
				connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
						[this]()
						{
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
	applyPointIndexLimitToSpins();
	fillPointRangeIfNeeded();
	updateScopeHint();
	m_loading = wasLoading;
	m_rebuilding = false;
}

bool TrajectoryOpParamPanel::applyTo(RobotInstruction::TrajectoryOpDescriptor& op,
									 const trajectory_algo::ITrajectoryOp* algo, std::string* errMsg)
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
				RobotInstruction::trajectoryOpSetProjectTargetBackendId(
					op, m_geometryBackendCombo->currentData().toString().toStdString());
			}
			continue;
		}
		if (row.field.key == "nrr.sourceBackendId")
		{
			if (m_nonRigidSourceCombo && m_nonRigidSourceCombo->currentIndex() >= 0)
			{
				RobotInstruction::trajectoryOpSetNonRigidSourceBackendId(
					op, m_nonRigidSourceCombo->currentData().toString().toStdString());
			}
			continue;
		}
		if (row.field.key == "nrr.targetBackendId")
		{
			if (m_nonRigidTargetCombo && m_nonRigidTargetCombo->currentIndex() >= 0)
			{
				RobotInstruction::trajectoryOpSetNonRigidTargetBackendId(
					op, m_nonRigidTargetCombo->currentData().toString().toStdString());
			}
			continue;
		}
		if (row.field.key == "toWorkpiece.externalTcpBackendId")
		{
			if (m_externalTcpBackendCombo && m_externalTcpBackendCombo->currentIndex() >= 0)
			{
				RobotInstruction::trajectoryOpSetToWorkpieceExternalTcpBackendId(
					op, m_externalTcpBackendCombo->currentData().toString().toStdString());
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
