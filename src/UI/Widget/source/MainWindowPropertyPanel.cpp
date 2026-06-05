#include "MainWindow.h"

#include <algorithm>
#include <memory>

#include <QList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendVisualSync.h"
#include "DocumentHostEvents.h"
#include "CoreTypes.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "BackendPropertySchema.h"
#include "BackendPropertyRow.h"
#include "MainWindow_p.h"
#include "WidgetRenderAccess.h"
#include "RobotInstructionPropertySchema.h"
#include "RunLogger.h"
#include "RunInfoPage.h"
#include "RobotSimulationController.h"
#include "../RobotWidget/inc/InstructionPropertyPanel.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;

namespace
{
const property_core::PropertyDescriptor* instructionPropertyDescriptorForKey(const QString& key)
{
	const std::string keyStd = key.toStdString();
	return RobotInstruction::findInstructionPropertyDescriptor(keyStd);
}

const property_core::PropertyDescriptor* backendPropertyDescriptorForKey(const QString& key)
{
	const std::string keyStd = key.toStdString();
	return backend_property_schema::findAnyBackendPropertyDescriptor(keyStd);
}

const property_core::PropertyDescriptor* panelPropertyDescriptorForKey(const QString& key)
{
	if (const property_core::PropertyDescriptor* d = instructionPropertyDescriptorForKey(key))
	{
		return d;
	}
	return backendPropertyDescriptorForKey(key);
}

QString snapshotPropertyValueForKey(const nlohmann::json& rows, const QString& key)
{
	if (!rows.is_array() || key.isEmpty())
	{
		return {};
	}
	const std::string want = key.toStdString();
	for (const auto& r : rows)
	{
		if (!r.is_object())
		{
			continue;
		}
		if (r.value(backend_property_json::kKey, std::string()) == want)
		{
			return QString::fromStdString(r.value(backend_property_json::kValue, std::string()));
		}
	}
	return {};
}

int propertyEditorTypeForKey(const QString& key, bool editable)
{
	if (!editable)
	{
		return QVariant::String;
	}
	if (const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool: return QVariant::Bool;
		case property_core::PropertyType::Int: return QVariant::Int;
		case property_core::PropertyType::Double: return QVariant::Double;
		case property_core::PropertyType::Enum: return QtVariantPropertyManager::enumTypeId();
		default: return QVariant::String;
		}
	}
	if (const property_core::PropertyDescriptor* descriptor = backendPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool: return QVariant::Bool;
		case property_core::PropertyType::Int: return QVariant::Int;
		case property_core::PropertyType::Double: return QVariant::Double;
		default: return QVariant::String;
		}
	}
	if (key == QStringLiteral("motion.tool.frameId") || key == QStringLiteral("motion.user.frameId")
		|| key == QStringLiteral("motion.target.frame"))
	{
		return QtVariantPropertyManager::enumTypeId();
	}
	return QVariant::String;
}

QString variantValueToString(const QVariant& value)
{
	switch (value.type())
	{
	case QVariant::Double:
		return QString::number(value.toDouble(), 'g', 12);
	case QVariant::Int:
		return QString::number(value.toInt());
	case QVariant::Bool:
		return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
	default:
		return value.toString();
	}
}

QString instructionEnumTokenFromValue(const QString& propertyKey, const QVariant& value)
{
	const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(propertyKey);
	if (!descriptor || descriptor->type != property_core::PropertyType::Enum)
	{
		return variantValueToString(value);
	}
	const int idx = value.toInt();
	if (idx >= 0 && idx < static_cast<int>(descriptor->constraints.enumConstraint.options.size()))
	{
		return QString::fromStdString(descriptor->constraints.enumConstraint.options[static_cast<size_t>(idx)]);
	}
	return variantValueToString(value);
}

QString axisConfigEnumDisplayName(const QString& propertyKey, const QString& token, bool chinese)
{
	if (propertyKey == QStringLiteral("motion.axisConfig.preset"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("ELBOW_UP")) return chinese ? QStringLiteral("肘上") : QStringLiteral("Elbow up");
		if (token == QStringLiteral("ELBOW_DOWN")) return chinese ? QStringLiteral("肘下") : QStringLiteral("Elbow down");
		if (token == QStringLiteral("WRIST_FLIP")) return chinese ? QStringLiteral("腕翻") : QStringLiteral("Wrist flip");
		if (token == QStringLiteral("WRIST_NO_FLIP")) return chinese ? QStringLiteral("腕不翻") : QStringLiteral("Wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_NO_FLIP")) return chinese ? QStringLiteral("肘上/腕不翻") : QStringLiteral("Elbow up, wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_FLIP")) return chinese ? QStringLiteral("肘上/腕翻") : QStringLiteral("Elbow up, wrist flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_NO_FLIP")) return chinese ? QStringLiteral("肘下/腕不翻") : QStringLiteral("Elbow down, wrist no-flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_FLIP")) return chinese ? QStringLiteral("肘下/腕翻") : QStringLiteral("Elbow down, wrist flip");
		if (token == QStringLiteral("CUSTOM")) return chinese ? QStringLiteral("自定义") : QStringLiteral("Custom");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.elbow"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("UP")) return chinese ? QStringLiteral("肘上") : QStringLiteral("Up");
		if (token == QStringLiteral("DOWN")) return chinese ? QStringLiteral("肘下") : QStringLiteral("Down");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.wrist"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("NO_FLIP")) return chinese ? QStringLiteral("腕不翻") : QStringLiteral("No flip");
		if (token == QStringLiteral("FLIP")) return chinese ? QStringLiteral("腕翻") : QStringLiteral("Flip");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.arm"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("FRONT")) return chinese ? QStringLiteral("臂前") : QStringLiteral("Front");
		if (token == QStringLiteral("BACK")) return chinese ? QStringLiteral("臂后") : QStringLiteral("Back");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.turn.j1")
		|| propertyKey == QStringLiteral("motion.axisConfig.turn.j4")
		|| propertyKey == QStringLiteral("motion.axisConfig.turn.j6"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		return chinese ? QStringLiteral("转 %1").arg(token) : QStringLiteral("Turn %1").arg(token);
	}
	return token;
}

bool isPoseComponentKey(const QString& key)
{
	return key == QStringLiteral("pose.x") || key == QStringLiteral("pose.y") || key == QStringLiteral("pose.z");
}

bool isRotationComponentKey(const QString& key)
{
	return key == QStringLiteral("rotation.x") || key == QStringLiteral("rotation.y") || key == QStringLiteral("rotation.z");
}

bool updateVec3ComponentFromKey(const QString& key, const QString& valueText, BackendVec3& ioVec)
{
	bool ok = false;
	const double v = valueText.toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	if (key.endsWith(QStringLiteral(".x")))
	{
		ioVec.x = v;
		return true;
	}
	if (key.endsWith(QStringLiteral(".y")))
	{
		ioVec.y = v;
		return true;
	}
	if (key.endsWith(QStringLiteral(".z")))
	{
		ioVec.z = v;
		return true;
	}
	return false;
}
} // namespace

void MainWindow::appendPropertyBrowserRow(
	const QString& propertyKey,
	const QString& displayLabel,
	const QString& value,
	bool editable,
	const std::vector<std::string>* enumOptionTokens,
	const QStringList* enumDisplayNames,
	const QString& toolTip)
{
	if (!m_variantManager || !m_propertyBrowser)
	{
		return;
	}
	QtVariantProperty* prop = nullptr;
	const int editorType = propertyEditorTypeForKey(propertyKey, editable);
	if (editorType == QVariant::Double)
	{
		bool ok = false;
		const double dv = value.toDouble(&ok);
		if (ok)
		{
			prop = m_variantManager->addProperty(QVariant::Double, displayLabel);
			m_variantManager->setAttribute(prop, QStringLiteral("decimals"), 6);
			m_variantManager->setValue(prop, dv);
			if (const property_core::PropertyDescriptor* d = panelPropertyDescriptorForKey(propertyKey))
			{
				if (d->constraints.rangeDouble.enabled)
				{
					m_variantManager->setAttribute(prop, QStringLiteral("minimum"), d->constraints.rangeDouble.minValue);
					m_variantManager->setAttribute(prop, QStringLiteral("maximum"), d->constraints.rangeDouble.maxValue);
				}
			}
		}
	}
	else if (editorType == QVariant::Int)
	{
		bool ok = false;
		const int iv = value.toInt(&ok);
		if (ok)
		{
			prop = m_variantManager->addProperty(QVariant::Int, displayLabel);
			m_variantManager->setValue(prop, iv);
		}
	}
	else if (editorType == QVariant::Bool)
	{
		const QString lower = value.trimmed().toLower();
		if (lower == QStringLiteral("true")
			|| lower == QStringLiteral("false")
			|| lower == QStringLiteral("1")
			|| lower == QStringLiteral("0"))
		{
			prop = m_variantManager->addProperty(QVariant::Bool, displayLabel);
			m_variantManager->setValue(prop, lower == QStringLiteral("true") || lower == QStringLiteral("1"));
		}
	}
	else if (editorType == QtVariantPropertyManager::enumTypeId())
	{
		const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(propertyKey);
		const std::vector<std::string>* optionsPtr = nullptr;
		std::vector<std::string> descriptorOptions;
		if (enumOptionTokens && !enumOptionTokens->empty())
		{
			optionsPtr = enumOptionTokens;
		}
		else if (descriptor && descriptor->type == property_core::PropertyType::Enum)
		{
			descriptorOptions = descriptor->constraints.enumConstraint.options;
			optionsPtr = &descriptorOptions;
		}
		if (optionsPtr && !optionsPtr->empty())
		{
			const std::vector<std::string>& options = *optionsPtr;
			QStringList enumNames;
			int selectedIndex = 0;
			const QString valueTrim = value.trimmed();
			for (size_t i = 0; i < options.size(); ++i)
			{
				const QString token = QString::fromStdString(options[i]);
				if (enumDisplayNames && static_cast<int>(i) < enumDisplayNames->size()
					&& !enumDisplayNames->at(static_cast<int>(i)).isEmpty())
				{
					enumNames << enumDisplayNames->at(static_cast<int>(i));
				}
				else
				{
					enumNames << axisConfigEnumDisplayName(propertyKey, token, m_useChinese);
				}
				if (token.compare(valueTrim, Qt::CaseInsensitive) == 0)
				{
					selectedIndex = static_cast<int>(i);
				}
			}
			if (!enumNames.isEmpty())
			{
				if (selectedIndex >= enumNames.size())
				{
					selectedIndex = 0;
				}
				prop = m_variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), displayLabel);
				m_variantManager->setAttribute(prop, QStringLiteral("enumNames"), enumNames);
				m_variantManager->setValue(prop, selectedIndex);
				QStringList tokenList;
				tokenList.reserve(static_cast<int>(options.size()));
				for (const std::string& opt : options)
				{
					tokenList << QString::fromStdString(opt);
				}
				m_propertyEnumTokens.insert(prop, tokenList);
			}
		}
	}
	if (!prop)
	{
		prop = m_variantManager->addProperty(QVariant::String, displayLabel);
		m_variantManager->setValue(prop, value);
		if (!editable)
		{
			prop->setEnabled(false);
		}
	}
	prop->setWhatsThis(propertyKey);
	if (!toolTip.isEmpty())
	{
		prop->setToolTip(toolTip);
	}
	m_propertyBrowser->addProperty(prop);
}

QString MainWindow::propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const
{
	const auto tr = [this](const QString& en, const QString& zh) { return i18n(en, zh); };

	if (key == QStringLiteral("core.id"))
	{
		return tr(QStringLiteral("ID"), QStringLiteral("标识"));
	}
	if (key == QStringLiteral("core.name"))
	{
		return tr(QStringLiteral("Name"), QStringLiteral("名称"));
	}
	if (key == QStringLiteral("core.class"))
	{
		return tr(QStringLiteral("Class"), QStringLiteral("类型"));
	}
	if (key == QStringLiteral("ui.active_axis"))
	{
		return tr(QStringLiteral("Active axis"), QStringLiteral("活动轴"));
	}
	if (key == QStringLiteral("pose.x"))
	{
		return tr(QStringLiteral("Position X"), QStringLiteral("位置 X"));
	}
	if (key == QStringLiteral("pose.y"))
	{
		return tr(QStringLiteral("Position Y"), QStringLiteral("位置 Y"));
	}
	if (key == QStringLiteral("pose.z"))
	{
		return tr(QStringLiteral("Position Z"), QStringLiteral("位置 Z"));
	}
	if (key == QStringLiteral("rotation.x"))
	{
		return tr(QStringLiteral("Rotation X (°)"), QStringLiteral("旋转 X (°)"));
	}
	if (key == QStringLiteral("rotation.y"))
	{
		return tr(QStringLiteral("Rotation Y (°)"), QStringLiteral("旋转 Y (°)"));
	}
	if (key == QStringLiteral("rotation.z"))
	{
		return tr(QStringLiteral("Rotation Z (°)"), QStringLiteral("旋转 Z (°)"));
	}
	if (key == QStringLiteral("pose.frame"))
	{
		return tr(QStringLiteral("Pose reference frame"), QStringLiteral("位姿参考系"));
	}
	if (key == QStringLiteral("color.r"))
	{
		return tr(QStringLiteral("Color R"), QStringLiteral("颜色 R"));
	}
	if (key == QStringLiteral("color.g"))
	{
		return tr(QStringLiteral("Color G"), QStringLiteral("颜色 G"));
	}
	if (key == QStringLiteral("color.b"))
	{
		return tr(QStringLiteral("Color B"), QStringLiteral("颜色 B"));
	}
	if (key == QStringLiteral("color.a"))
	{
		return tr(QStringLiteral("Color A"), QStringLiteral("颜色 A"));
	}
	if (key == QStringLiteral("follow.targetName"))
	{
		return tr(QStringLiteral("Follow target (object name)"), QStringLiteral("跟随目标（对象名称）"));
	}
	if (key == QStringLiteral("mesh.triangle_count"))
	{
		return tr(QStringLiteral("Triangle count"), QStringLiteral("三角形数"));
	}
	if (key == QStringLiteral("motion.target.frame"))
	{
		return tr(QStringLiteral("Target frame"), QStringLiteral("目标坐标系"));
	}
	if (key == QStringLiteral("motion.tool.frameId"))
	{
		return tr(QStringLiteral("Tool frame"), QStringLiteral("工具坐标系"));
	}
	if (key == QStringLiteral("motion.user.frameId"))
	{
		return tr(QStringLiteral("User frame"), QStringLiteral("用户坐标系"));
	}
	if (key == QStringLiteral("motion.target.pose.x"))
	{
		return tr(QStringLiteral("Target X (mm)"), QStringLiteral("目标 X (mm)"));
	}
	if (key == QStringLiteral("motion.target.pose.y"))
	{
		return tr(QStringLiteral("Target Y (mm)"), QStringLiteral("目标 Y (mm)"));
	}
	if (key == QStringLiteral("motion.target.pose.z"))
	{
		return tr(QStringLiteral("Target Z (mm)"), QStringLiteral("目标 Z (mm)"));
	}
	if (key == QStringLiteral("motion.target.euler.rx"))
	{
		return tr(QStringLiteral("Euler RX (deg)"), QStringLiteral("欧拉角 RX (deg)"));
	}
	if (key == QStringLiteral("motion.target.euler.ry"))
	{
		return tr(QStringLiteral("Euler RY (deg)"), QStringLiteral("欧拉角 RY (deg)"));
	}
	if (key == QStringLiteral("motion.target.euler.rz"))
	{
		return tr(QStringLiteral("Euler RZ (deg)"), QStringLiteral("欧拉角 RZ (deg)"));
	}
	if (key == QStringLiteral("motion.speed"))
	{
		return tr(QStringLiteral("Speed"), QStringLiteral("速度"));
	}
	if (key == QStringLiteral("motion.acc"))
	{
		return tr(QStringLiteral("Acceleration"), QStringLiteral("加速度"));
	}
	if (key == QStringLiteral("motion.axisConfig") || key == QStringLiteral("motion.axisConfig.preset"))
	{
		return tr(QStringLiteral("Axis config preset"), QStringLiteral("轴配置预设"));
	}
	if (key == QStringLiteral("motion.axisConfig.elbow"))
	{
		return tr(QStringLiteral("Elbow posture"), QStringLiteral("肘部姿态"));
	}
	if (key == QStringLiteral("motion.axisConfig.wrist"))
	{
		return tr(QStringLiteral("Wrist posture"), QStringLiteral("腕部姿态"));
	}
	if (key == QStringLiteral("motion.axisConfig.arm"))
	{
		return tr(QStringLiteral("Arm posture"), QStringLiteral("臂形前后"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j1"))
	{
		return tr(QStringLiteral("J1 turn (rev)"), QStringLiteral("J1 转数"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j4"))
	{
		return tr(QStringLiteral("J4 turn (rev)"), QStringLiteral("J4 转数"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j6"))
	{
		return tr(QStringLiteral("J6 turn (rev)"), QStringLiteral("J6 转数"));
	}
	if (key == QStringLiteral("motion.blendRadius"))
	{
		return tr(QStringLiteral("Blend Radius (mm)"), QStringLiteral("平滑半径 (mm)"));
	}
	if (key == QStringLiteral("motion.pointIndex"))
	{
		return tr(QStringLiteral("Waypoint index"), QStringLiteral("点位编号"));
	}
	if (const property_core::PropertyDescriptor* descriptor = panelPropertyDescriptorForKey(key))
	{
		return QString::fromStdString(descriptor->label);
	}
	return labelEnFallback;
}

QString MainWindow::instructionEnumTokenFromProperty(QtProperty* property, const QVariant& value) const
{
	if (property)
	{
		const auto it = m_propertyEnumTokens.constFind(property);
		if (it != m_propertyEnumTokens.constEnd())
		{
			const int idx = value.toInt();
			if (idx >= 0 && idx < it->size())
			{
				return it->at(idx);
			}
		}
	}
	return instructionEnumTokenFromValue(property ? property->whatsThis() : QString(), value);
}

void MainWindow::invalidateFeasibleAxisConfigurationCache()
{
	if (m_robotSimulation)
	{
		m_robotSimulation->invalidateFeasibleAxisConfigurationCache();
	}
}

void MainWindow::applySuggestedAxisPresetFromSeedIfNeeded(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(
		m_instructionPropertyUiHost, instruction, seedJointRad, feasible);
}

void MainWindow::updateInstructionPropertyPanel(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const bool refreshFeasibleAxisOptions)
{
	InstructionPropertyPanel::update(m_instructionPropertyUiHost, instruction, refreshFeasibleAxisOptions);
}

void MainWindow::updatePropertyPanel(const QString& backendId)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	if (!backendId.isEmpty() && m_followTargetNameDebounceTimer.isActive()
		&& backendId == m_followTargetNameDebounceBackendId)
	{
		return;
	}
	if (backendId.isEmpty())
	{
		m_followTargetNameDebounceTimer.stop();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
	}
	else if (!m_followTargetNameDebounceBackendId.isEmpty() && backendId != m_followTargetNameDebounceBackendId)
	{
		m_followTargetNameDebounceTimer.stop();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
	}
	m_updatingPropertyBrowser = true;
	m_variantManager->clear();
	if (backendId.isEmpty())
	{
		m_updatingPropertyBrowser = false;
		return;
	}

	DocumentPage* docPage = currentPage();
	if (!docPage || !docPage->data().isValid(backendId))
	{
		m_updatingPropertyBrowser = false;
		return;
	}
	const QVector<cloudsim::core::PropertyRowDto> rows = docPage->data().propertyRows(backendId);
	for (const cloudsim::core::PropertyRowDto& r : rows)
	{
		QString key = r.key;
		bool editable = r.editable;
		if (key == QStringLiteral("follow.targetName"))
		{
			editable = true;
		}
		const QString label = propertyDisplayLabelForKey(key, r.labelEn);
		appendPropertyBrowserRow(key, label, r.value, editable);
	}

	const bool showAxis = docPage->data().geometryKind(backendId) == cloudsim::core::GeometryKind::Points;
	if (showAxis)
	{
		const QString axisKey = QStringLiteral("ui.active_axis");
		appendPropertyBrowserRow(
			axisKey,
			propertyDisplayLabelForKey(axisKey, QStringLiteral("Active axis")),
			m_activeAxisName,
			false);
	}

	m_updatingPropertyBrowser = false;
}

void MainWindow::onVariantPropertyValueChanged(QtProperty* property, const QVariant& value)
{
	if (m_updatingPropertyBrowser || !property)
	{
		return;
	}
	if (InstructionPropertyPanel::handleVariantPropertyValueChanged(m_instructionPropertyUiHost, property, value))
	{
		return;
	}
	if (!m_backendTree || !renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const QString backendId = m_selectionState.selectedBackendId();
	if (backendId.isEmpty())
	{
		return;
	}

	const QString propertyKey = property->whatsThis();
	if (propertyKey == QStringLiteral("follow.targetName"))
	{
		m_followTargetNameDebounceBackendId = backendId;
		m_followTargetNameDebounceText = variantValueToString(value);
		m_followTargetNameDebounceTimer.start(400);
		return;
	}
	if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("ui.")))
	{
		return;
	}
	const QString valueText = variantValueToString(value);

	const QByteArray keyBytes = propertyKey.toUtf8();
	const std::string keyUtf8(keyBytes.constData(), static_cast<std::size_t>(keyBytes.size()));
	const QByteArray valueBytes = valueText.toUtf8();
	const std::string valueUtf8(valueBytes.constData(), static_cast<std::size_t>(valueBytes.size()));

	DocumentPage* docPage = currentPage();
	if (!docPage)
	{
		RunLogger::debug(std::string("[PropertyCommitDBG] skip commit without document page id=")
			+ backendId.toStdString() + " key=" + keyUtf8);
		RunLogger::flush();
		return;
	}

	QString oldValue;
	for (const cloudsim::core::PropertyRowDto& row : docPage->data().propertyRows(backendId))
	{
		if (row.key == propertyKey)
		{
			oldValue = row.value;
			break;
		}
	}

	QString dsErr;
	const bool applyOk = docPage->data().applyPropertyChange(backendId, propertyKey, valueText, &dsErr);
	if (!applyOk)
	{
		RunLogger::debug(std::string("[PropertyCommitDBG] apply failed id=") + backendId.toStdString()
			+ " key=" + keyUtf8
			+ " old=" + oldValue.toStdString()
			+ " new=" + valueUtf8
			+ " err=" + dsErr.toStdString());
		RunLogger::flush();
		updatePropertyPanel(backendId);
		return;
	}
	RunLogger::debug(std::string("[PropertyCommitDBG] apply ok id=") + backendId.toStdString()
		+ " key=" + keyUtf8
		+ " old=" + oldValue.toStdString()
		+ " new=" + valueUtf8);
	RunLogger::flush();

	const bool followKey = propertyKey.startsWith(QStringLiteral("follow."));
	if (followKey)
	{
		afterBackendFollowPropertyEdited(propertyKey, valueText);
	}
	else
	{
		docPage->markFollowAttachmentDirtyFromBackendMove(backendId);
	}

	if (cloudsim::host::propertyKeyNeedsVisualSync(propertyKey))
	{
		const bool applyColor = propertyKey.contains(QStringLiteral("color"), Qt::CaseInsensitive);
		cloudsim::host::syncVisualAfterPropertyChangeById(*docPage, backendId, applyColor);
		if (cloudsim::host::propertyKeyCommitsPose(propertyKey))
		{
			cloudsim::host::publishPoseCommittedFromBackendId(*docPage, backendId);
		}
	}
	if (isPoseComponentKey(propertyKey) || isRotationComponentKey(propertyKey))
	{
		syncRobotKinematicsAfterPoseEdit(backendId);
	}

	schedulePropertyPanelCommitRefresh(backendId);
}

void MainWindow::flushFollowTargetNamePropertyEdit()
{
	if (!m_backendTree || !renderWidgetFromPage(currentPage()) || !m_selectionState.hasBackendSelection())
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}
	const QString selId = m_selectionState.selectedBackendId();
	if (selId != m_followTargetNameDebounceBackendId)
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}
	if (m_followTargetNameDebounceBackendId.isEmpty())
	{
		return;
	}

	DocumentPage* docPage = currentPage();
	const QString propertyKey = QStringLiteral("follow.targetName");
	const QString backendId = m_followTargetNameDebounceBackendId;

	if (!docPage)
	{
		RunLogger::debug(std::string("[PropertyCommitDBG] skip follow name commit without document page id=")
			+ backendId.toStdString());
		RunLogger::flush();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}

	QString dsErr;
	const bool applyOk = docPage->data().applyPropertyChange(backendId, propertyKey, m_followTargetNameDebounceText,
		&dsErr);
	if (!applyOk)
	{
		m_followTargetNameDebounceBackendId.clear();
		updatePropertyPanel(backendId);
		return;
	}

	afterBackendFollowPropertyEdited(propertyKey, m_followTargetNameDebounceText);

	m_followTargetNameDebounceBackendId.clear();
	m_followTargetNameDebounceText.clear();
	updatePropertyPanel(backendId);
}
