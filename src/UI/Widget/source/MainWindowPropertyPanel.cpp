/// @file MainWindowPropertyPanel.cpp
/// @brief 属性面板绑定

#include "../RobotWidget/inc/InstructionPropertyPanel.h"
#include "BackendPropertyRow.h"
#include "BackendPropertySchema.h"
#include "BackendTypeIds.h"
#include "BackendVisualSync.h"
#include "CoreTypes.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "io/CustomDeviceHostOps.h"
#include "MainWindow.h"
#include "MainWindow_p.h"
#include "RobotInstructionPropertySchema.h"
#include "RobotSimulationController.h"
#include "RunInfoPage.h"
#include "RunLogger.h"
#include "WidgetRenderAccess.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

#include <QAbstractItemView>
#include <QColor>
#include <QEvent>
#include <QList>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QWidget>
#include <algorithm>
#include <cmath>
#include <memory>

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
	// 动态枚举优先：schema 里仍是 String，但面板用本机信号表做下拉
	if (key == QStringLiteral("motion.tool.frameId") || key == QStringLiteral("motion.user.frameId") ||
		key == QStringLiteral("motion.target.frame") || key == QStringLiteral("logic.io.signalName") ||
		key == QStringLiteral("logic.condition.signalName") || key == QStringLiteral("logic.condition.kind") ||
		key == QStringLiteral("logic.condition.equals") || key == QStringLiteral("logic.device.backendId"))
	{
		return QtVariantPropertyManager::enumTypeId();
	}
	if (const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool:
			return QVariant::Bool;
		case property_core::PropertyType::Int:
			return QVariant::Int;
		case property_core::PropertyType::Double:
			return QVariant::Double;
		case property_core::PropertyType::Enum:
			return QtVariantPropertyManager::enumTypeId();
		default:
			return QVariant::String;
		}
	}
	if (const property_core::PropertyDescriptor* descriptor = backendPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool:
			return QVariant::Bool;
		case property_core::PropertyType::Int:
			return QVariant::Int;
		case property_core::PropertyType::Double:
			return QVariant::Double;
		default:
			return QVariant::String;
		}
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
		if (token == QStringLiteral("AUTO"))
			return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("ELBOW_UP"))
			return chinese ? QStringLiteral("肘上") : QStringLiteral("Elbow up");
		if (token == QStringLiteral("ELBOW_DOWN"))
			return chinese ? QStringLiteral("肘下") : QStringLiteral("Elbow down");
		if (token == QStringLiteral("WRIST_FLIP"))
			return chinese ? QStringLiteral("腕翻") : QStringLiteral("Wrist flip");
		if (token == QStringLiteral("WRIST_NO_FLIP"))
			return chinese ? QStringLiteral("腕不翻") : QStringLiteral("Wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_NO_FLIP"))
			return chinese ? QStringLiteral("肘上/腕不翻") : QStringLiteral("Elbow up, wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_FLIP"))
			return chinese ? QStringLiteral("肘上/腕翻") : QStringLiteral("Elbow up, wrist flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_NO_FLIP"))
			return chinese ? QStringLiteral("肘下/腕不翻") : QStringLiteral("Elbow down, wrist no-flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_FLIP"))
			return chinese ? QStringLiteral("肘下/腕翻") : QStringLiteral("Elbow down, wrist flip");
		if (token == QStringLiteral("CUSTOM"))
			return chinese ? QStringLiteral("自定义") : QStringLiteral("Custom");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.elbow"))
	{
		if (token == QStringLiteral("AUTO"))
			return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("UP"))
			return chinese ? QStringLiteral("肘上") : QStringLiteral("Up");
		if (token == QStringLiteral("DOWN"))
			return chinese ? QStringLiteral("肘下") : QStringLiteral("Down");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.wrist"))
	{
		if (token == QStringLiteral("AUTO"))
			return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("NO_FLIP"))
			return chinese ? QStringLiteral("腕不翻") : QStringLiteral("No flip");
		if (token == QStringLiteral("FLIP"))
			return chinese ? QStringLiteral("腕翻") : QStringLiteral("Flip");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.arm"))
	{
		if (token == QStringLiteral("AUTO"))
			return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("FRONT"))
			return chinese ? QStringLiteral("臂前") : QStringLiteral("Front");
		if (token == QStringLiteral("BACK"))
			return chinese ? QStringLiteral("臂后") : QStringLiteral("Back");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.turn.j1") ||
		propertyKey == QStringLiteral("motion.axisConfig.turn.j4") ||
		propertyKey == QStringLiteral("motion.axisConfig.turn.j6"))
	{
		if (token == QStringLiteral("AUTO"))
			return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		return chinese ? QStringLiteral("转 %1").arg(token) : QStringLiteral("Turn %1").arg(token);
	}
	if (propertyKey == QStringLiteral("logic.condition.kind"))
	{
		if (token == QStringLiteral("always"))
			return chinese ? QStringLiteral("延时") : QStringLiteral("Delay");
		if (token == QStringLiteral("io"))
			return chinese ? QStringLiteral("等待信号") : QStringLiteral("Wait signal");
		if (token == QStringLiteral("never"))
			return chinese ? QStringLiteral("永不") : QStringLiteral("Never");
		if (token == QStringLiteral("compare"))
			return chinese ? QStringLiteral("比较") : QStringLiteral("Compare");
	}
	if (propertyKey == QStringLiteral("logic.condition.equals"))
	{
		if (token == QStringLiteral("0"))
			return chinese ? QStringLiteral("为 0") : QStringLiteral("Equals 0");
		if (token == QStringLiteral("1"))
			return chinese ? QStringLiteral("为 1") : QStringLiteral("Equals 1");
	}
	return token;
}

bool isPoseComponentKey(const QString& key)
{
	return key == QStringLiteral("pose.x") || key == QStringLiteral("pose.y") || key == QStringLiteral("pose.z");
}

bool isRotationComponentKey(const QString& key)
{
	return key == QStringLiteral("rotation.x") || key == QStringLiteral("rotation.y") ||
		   key == QStringLiteral("rotation.z");
}

bool isColorComponentKey(const QString& key)
{
	return key == QStringLiteral("color.r") || key == QStringLiteral("color.g") || key == QStringLiteral("color.b") ||
		   key == QStringLiteral("color.a");
}

QString propertyRowValue(DocumentPage& doc, const QString& backendId, const QString& key)
{
	const QVector<cloudsim::core::PropertyRowDto> rows = doc.data().propertyRows(backendId);
	for (const cloudsim::core::PropertyRowDto& r : rows)
	{
		if (r.key == key)
		{
			return r.value;
		}
	}
	return QString();
}

double clampUnitInterval(const double v)
{
	return std::clamp(v, 0.0, 1.0);
}

bool poseDtoNearlyEqual(const cloudsim::core::PoseDto& a, const cloudsim::core::PoseDto& b)
{
	// 勿用 near：windows.h 会把它宏成空
	const auto within = [](double x, double y) { return std::abs(x - y) <= 1e-3; };
	return within(a.positionMm.x, b.positionMm.x) && within(a.positionMm.y, b.positionMm.y) &&
		   within(a.positionMm.z, b.positionMm.z) && within(a.eulerDeg.x, b.eulerDeg.x) &&
		   within(a.eulerDeg.y, b.eulerDeg.y) && within(a.eulerDeg.z, b.eulerDeg.z);
}

bool propertyNumericValuesEqual(const QVariant& editorValue, const QString& rowText)
{
	bool okRow = false;
	const double row = rowText.toDouble(&okRow);
	if (!okRow)
	{
		return false;
	}
	bool okEd = false;
	double ed = 0.0;
	if (editorValue.type() == QVariant::Double)
	{
		ed = editorValue.toDouble();
		okEd = true;
	}
	else
	{
		ed = editorValue.toString().toDouble(&okEd);
	}
	if (!okEd)
	{
		return false;
	}
	return std::abs(ed - row) <= 5e-4;
}

bool colorFromPropertyRows(const QVector<cloudsim::core::PropertyRowDto>& rows, QColor* outColor)
{
	if (!outColor)
	{
		return false;
	}
	double r = 1.0;
	double g = 1.0;
	double b = 1.0;
	double a = 1.0;
	bool hasR = false;
	bool hasG = false;
	bool hasB = false;
	bool hasA = false;
	for (const cloudsim::core::PropertyRowDto& row : rows)
	{
		bool ok = false;
		const double v = row.value.toDouble(&ok);
		if (!ok)
		{
			continue;
		}
		if (row.key == QStringLiteral("color.r"))
		{
			r = v;
			hasR = true;
		}
		else if (row.key == QStringLiteral("color.g"))
		{
			g = v;
			hasG = true;
		}
		else if (row.key == QStringLiteral("color.b"))
		{
			b = v;
			hasB = true;
		}
		else if (row.key == QStringLiteral("color.a"))
		{
			a = v;
			hasA = true;
		}
	}
	if (!hasR && !hasG && !hasB && !hasA)
	{
		return false;
	}
	*outColor =
		QColor::fromRgbF(clampUnitInterval(r), clampUnitInterval(g), clampUnitInterval(b), clampUnitInterval(a));
	return true;
}

QVariant propertyRowValueToVariant(const QString& key, const QString& value, bool editable)
{
	const int editorType = propertyEditorTypeForKey(key, editable);
	if (editorType == QVariant::Double)
	{
		bool ok = false;
		const double dv = value.toDouble(&ok);
		if (ok)
		{
			return dv;
		}
	}
	else if (editorType == QVariant::Int)
	{
		bool ok = false;
		const int iv = value.toInt(&ok);
		if (ok)
		{
			return iv;
		}
	}
	else if (editorType == QVariant::Bool)
	{
		const QString lower = value.trimmed().toLower();
		if (lower == QStringLiteral("true") || lower == QStringLiteral("false") || lower == QStringLiteral("1") ||
			lower == QStringLiteral("0"))
		{
			return lower == QStringLiteral("true") || lower == QStringLiteral("1");
		}
	}
	return value;
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

void MainWindow::appendPropertyBrowserRow(const QString& propertyKey, const QString& displayLabel, const QString& value,
										  bool editable, const std::vector<std::string>* enumOptionTokens,
										  const QStringList* enumDisplayNames, const QString& toolTip)
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
					m_variantManager->setAttribute(prop, QStringLiteral("minimum"),
												   d->constraints.rangeDouble.minValue);
					m_variantManager->setAttribute(prop, QStringLiteral("maximum"),
												   d->constraints.rangeDouble.maxValue);
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
		if (lower == QStringLiteral("true") || lower == QStringLiteral("false") || lower == QStringLiteral("1") ||
			lower == QStringLiteral("0"))
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
				if (enumDisplayNames && static_cast<int>(i) < enumDisplayNames->size() &&
					!enumDisplayNames->at(static_cast<int>(i)).isEmpty())
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
	m_propertyKeyToVariant.insert(propertyKey, prop);
	m_propertyBrowser->addProperty(prop);
}

void MainWindow::appendColorPropertyBrowserRow(const QColor& color)
{
	if (!m_variantManager || !m_propertyBrowser)
	{
		return;
	}
	const QString colorKey = QStringLiteral("color");
	QtVariantProperty* prop =
		m_variantManager->addProperty(QVariant::Color, propertyDisplayLabelForKey(colorKey, QStringLiteral("Color")));
	m_variantManager->setValue(prop, color);
	prop->setWhatsThis(colorKey);
	m_propertyKeyToVariant.insert(colorKey, prop);
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
	if (key == QStringLiteral("color"))
	{
		return tr(QStringLiteral("Color"), QStringLiteral("颜色"));
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
	if (key == QStringLiteral("logic.io.signalName") || key == QStringLiteral("logic.condition.signalName"))
	{
		return tr(QStringLiteral("Signal name"), QStringLiteral("信号名"));
	}
	if (key == QStringLiteral("logic.condition.kind"))
	{
		return tr(QStringLiteral("Wait mode"), QStringLiteral("等待方式"));
	}
	if (key == QStringLiteral("logic.condition.port"))
	{
		return tr(QStringLiteral("IO port"), QStringLiteral("IO 端口"));
	}
	if (key == QStringLiteral("logic.condition.equals"))
	{
		return tr(QStringLiteral("Equals (0/1)"), QStringLiteral("目标值 (0/1)"));
	}
	if (key == QStringLiteral("logic.device.backendId"))
	{
		return tr(QStringLiteral("Device id"), QStringLiteral("设备 ID"));
	}
	if (key == QStringLiteral("logic.device.axisIndex"))
	{
		return tr(QStringLiteral("Axis index"), QStringLiteral("轴下标"));
	}
	if (key == QStringLiteral("logic.device.targetQ"))
	{
		return tr(QStringLiteral("Target q"), QStringLiteral("目标 q"));
	}
	if (key == QStringLiteral("logic.device.durationSec") || key == QStringLiteral("logic.wait.durationSec"))
	{
		return tr(QStringLiteral("Duration/Timeout (s)"), QStringLiteral("时长/超时 (s)"));
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
	const std::shared_ptr<RobotInstruction::Base>& instruction, const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(m_instructionPropertyUiHost, instruction,
																	   seedJointRad, feasible);
}

void MainWindow::updateInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction,
												const bool refreshFeasibleAxisOptions)
{
	if (instruction)
	{
		const QString instructionId = QString::fromStdString(instruction->id());
		if (m_propertyPanelDeferFullRebuild && instructionId != m_propertyPanelActiveEditContextId)
		{
			m_propertyPanelActiveEditKey.clear();
			m_propertyPanelActiveEditContextId.clear();
			m_propertyPanelDeferFullRebuild = false;
			m_propertyVisualPreviewTimer.stop();
			m_propertyVisualPreviewBackendId.clear();
			if (DocumentPage* docPage = currentPage())
			{
				docPage->setDeferPropertyPanelVisualFullSync(false);
			}
		}
		if (shouldDeferPropertyPanelRebuild(instructionId))
		{
			return;
		}
	}
	else if (m_propertyPanelDeferFullRebuild)
	{
		m_propertyPanelActiveEditKey.clear();
		m_propertyPanelActiveEditContextId.clear();
		m_propertyPanelDeferFullRebuild = false;
		m_propertyVisualPreviewTimer.stop();
		m_propertyVisualPreviewBackendId.clear();
		if (DocumentPage* docPage = currentPage())
		{
			docPage->setDeferPropertyPanelVisualFullSync(false);
		}
	}
	InstructionPropertyPanel::update(m_instructionPropertyUiHost, instruction, refreshFeasibleAxisOptions);
}

void MainWindow::clearPropertyKeyVariantMap()
{
	m_propertyKeyToVariant.clear();
}

bool MainWindow::shouldDeferPropertyPanelRebuild(const QString& contextId) const
{
	return m_propertyPanelDeferFullRebuild && !m_propertyPanelActiveEditKey.isEmpty() &&
		   contextId == m_propertyPanelActiveEditContextId;
}

void MainWindow::beginPropertyPanelNumericEdit(const QString& contextId, const QString& propertyKey)
{
	m_propertyPanelActiveEditContextId = contextId;
	m_propertyPanelActiveEditKey = propertyKey;
	m_propertyPanelDeferFullRebuild = true;
	m_propertyPanelCommitTimer.stop();
	m_propertyPanelCommitPendingBackendId.clear();
	if (DocumentPage* docPage = currentPage())
	{
		docPage->setDeferPropertyPanelVisualFullSync(true);
	}
}

void MainWindow::endPropertyPanelNumericEdit()
{
	if (!m_propertyPanelDeferFullRebuild)
	{
		return;
	}
	const QString contextId = m_propertyPanelActiveEditContextId;
	m_propertyPanelActiveEditKey.clear();
	m_propertyPanelActiveEditContextId.clear();
	m_propertyPanelDeferFullRebuild = false;
	m_propertyVisualPreviewTimer.stop();
	m_propertyVisualPreviewBackendId.clear();
	if (DocumentPage* docPage = currentPage())
	{
		docPage->setDeferPropertyPanelVisualFullSync(false);
	}
	flushPropertyPanelVisualCommit(contextId);
	flushPropertyPanelRefresh(contextId);
}

void MainWindow::scheduleThrottledPropertyVisualPreview(const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	m_propertyVisualPreviewBackendId = backendId;
	m_propertyVisualPreviewTimer.start(33);
}

void MainWindow::onPropertyVisualPreviewTimer()
{
	const QString backendId = m_propertyVisualPreviewBackendId;
	m_propertyVisualPreviewBackendId.clear();
	if (backendId.isEmpty() || !shouldDeferPropertyPanelRebuild(backendId))
	{
		return;
	}
	DocumentPage* docPage = currentPage();
	if (!docPage || !docPage->data().isValid(backendId))
	{
		return;
	}
	const auto obj = docPage->findObject(backendId.toStdString());
	if (obj && obj->className() == backend_type::kClassCustomDevice)
	{
		cloudsim::host::syncCustomDeviceKinematicsAfterRootPoseChange(*docPage, backendId.toStdString());
		return;
	}
	(void)docPage->syncOuterPatFromBackendId(backendId.toStdString());
}

void MainWindow::flushPropertyPanelVisualCommit(const QString& contextId)
{
	if (contextId.isEmpty())
	{
		return;
	}
	DocumentPage* docPage = currentPage();
	if (!docPage)
	{
		return;
	}
	if (m_activeInstructionForProperty && QString::fromStdString(m_activeInstructionForProperty->id()) == contextId)
	{
		applyRobotPoseForInstructionPreview(m_activeInstructionForProperty);
		refreshInstructionPoseAxes();
		return;
	}
	if (!m_selectionState.hasBackendSelection() || m_selectionState.selectedBackendId() != contextId)
	{
		return;
	}
	const QString backendId = contextId;
	cloudsim::host::syncVisualAfterPropertyChangeById(*docPage, backendId, false);
	cloudsim::host::publishPoseCommittedFromBackendId(*docPage, backendId);
	syncRobotKinematicsAfterPoseEdit(backendId);
	cloudsim::core::IRenderView* rv = &docPage->render();
	if (!rv->isTransformGizmoDragging())
	{
		(void)docPage->data().runFollowSolveAndSync(makeFollowSolveContextDto(*docPage), nullptr);
	}
}

void MainWindow::flushPropertyPanelRefresh(const QString& contextId)
{
	if (contextId.isEmpty())
	{
		return;
	}
	if (m_activeInstructionForProperty && QString::fromStdString(m_activeInstructionForProperty->id()) == contextId)
	{
		updateInstructionPropertyPanel(m_activeInstructionForProperty, false);
		return;
	}
	if (m_selectionState.hasBackendSelection() && m_selectionState.selectedBackendId() == contextId)
	{
		updatePropertyPanel(contextId);
	}
}

void MainWindow::syncPropertyPanelRowValues(const QString& backendId)
{
	if (!m_variantManager || backendId.isEmpty())
	{
		return;
	}
	DocumentPage* docPage = currentPage();
	if (!docPage || !docPage->data().isValid(backendId))
	{
		return;
	}
	m_updatingPropertyBrowser = true;
	const QVector<cloudsim::core::PropertyRowDto> rows = docPage->data().propertyRows(backendId);
	for (const cloudsim::core::PropertyRowDto& r : rows)
	{
		if (r.key == m_propertyPanelActiveEditKey)
		{
			continue;
		}
		QtProperty* prop = m_propertyKeyToVariant.value(r.key);
		if (!prop)
		{
			continue;
		}
		const QVariant v = propertyRowValueToVariant(r.key, r.value, r.editable);
		m_variantManager->setValue(prop, v);
	}
	if (QtProperty* colorProp = m_propertyKeyToVariant.value(QStringLiteral("color")))
	{
		QColor objectColor;
		if (colorFromPropertyRows(rows, &objectColor))
		{
			m_variantManager->setValue(colorProp, objectColor);
		}
	}
	m_updatingPropertyBrowser = false;
	if (m_propertyBrowser)
	{
		m_propertyBrowser->update();
	}
}

void MainWindow::syncPropertyPanelGizmoLiveValues(const QString& backendId)
{
	if (!m_variantManager || backendId.isEmpty())
	{
		return;
	}
	DocumentPage* docPage = currentPage();
	if (!docPage || !docPage->data().isValid(backendId))
	{
		return;
	}
	cloudsim::core::IRenderView* rv = &docPage->render();
	if (!rv->isTransformGizmoDragging())
	{
		return;
	}
	float px = 0.0f;
	float py = 0.0f;
	float pz = 0.0f;
	float rx = 0.0f;
	float ry = 0.0f;
	float rz = 0.0f;
	(void)rv->selectedPosition(px, py, pz);
	(void)rv->selectedRotationEulerDeg(rx, ry, rz);

	m_updatingPropertyBrowser = true;
	const auto applyKey = [&](const QString& key, const double v)
	{
		if (key == m_propertyPanelActiveEditKey)
		{
			return;
		}
		QtProperty* prop = m_propertyKeyToVariant.value(key);
		if (!prop)
		{
			return;
		}
		m_variantManager->setValue(prop, v);
	};
	applyKey(QStringLiteral("pose.x"), static_cast<double>(px));
	applyKey(QStringLiteral("pose.y"), static_cast<double>(py));
	applyKey(QStringLiteral("pose.z"), static_cast<double>(pz));
	applyKey(QStringLiteral("rotation.x"), static_cast<double>(rx));
	applyKey(QStringLiteral("rotation.y"), static_cast<double>(ry));
	applyKey(QStringLiteral("rotation.z"), static_cast<double>(rz));
	m_updatingPropertyBrowser = false;
	if (m_propertyBrowser)
	{
		m_propertyBrowser->update();
		if (QTreeWidget* tree = m_propertyBrowser->findChild<QTreeWidget*>())
		{
			tree->viewport()->update();
		}
	}
}

void MainWindow::scheduleInstructionPropertyRefreshDebounced(const std::shared_ptr<RobotInstruction::Base>& instruction,
															 const bool refreshFeasibleAxisOptions)
{
	if (!instruction)
	{
		return;
	}
	m_instructionPropertyRefreshPendingInstructionId = QString::fromStdString(instruction->id());
	m_instructionPropertyRefreshFeasibleAxis = refreshFeasibleAxisOptions;
	m_instructionPropertyRefreshTimer.start(220);
}

void MainWindow::onInstructionPropertyRefreshTimer()
{
	const QString wantId = m_instructionPropertyRefreshPendingInstructionId;
	m_instructionPropertyRefreshPendingInstructionId.clear();
	if (wantId.isEmpty() || !m_activeInstructionForProperty)
	{
		return;
	}
	if (QString::fromStdString(m_activeInstructionForProperty->id()) != wantId)
	{
		return;
	}
	if (shouldDeferPropertyPanelRebuild(wantId))
	{
		m_instructionPropertyRefreshPendingInstructionId = wantId;
		m_instructionPropertyRefreshTimer.start(220);
		return;
	}
	updateInstructionPropertyPanel(m_activeInstructionForProperty, m_instructionPropertyRefreshFeasibleAxis);
}

void MainWindow::installPropertyPanelEventFilter()
{
	if (!m_propertyBrowser)
	{
		return;
	}
	m_propertyBrowser->installEventFilter(this);
	if (QTreeWidget* propTree = m_propertyBrowser->findChild<QTreeWidget*>())
	{
		propTree->installEventFilter(this);
	}
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
	if (event->type() == QEvent::FocusOut && m_propertyPanelDeferFullRebuild && m_propertyBrowser)
	{
		QTreeWidget* propTree = m_propertyBrowser->findChild<QTreeWidget*>();
		if (propTree && (watched == propTree || watched == m_propertyBrowser ||
						 (qobject_cast<QWidget*>(watched) && propTree->isAncestorOf(qobject_cast<QWidget*>(watched)))))
		{
			QTimer::singleShot(0, this,
							   [this]()
							   {
								   if (!m_propertyPanelDeferFullRebuild || !m_propertyBrowser)
								   {
									   return;
								   }
								   QTreeWidget* tree = m_propertyBrowser->findChild<QTreeWidget*>();
								   if (!tree)
								   {
									   endPropertyPanelNumericEdit();
									   return;
								   }
								   QWidget* focusWidget = tree->focusWidget();
								   if (!focusWidget || !tree->isAncestorOf(focusWidget))
								   {
									   endPropertyPanelNumericEdit();
								   }
							   });
		}
	}
	return QMainWindow::eventFilter(watched, event);
}

void MainWindow::beginPropertyBrowserProgrammaticUpdate()
{
	++m_propertyBrowserUpdateGuardGeneration;
	m_updatingPropertyBrowser = true;
}

void MainWindow::endPropertyBrowserProgrammaticUpdate()
{
	const int gen = m_propertyBrowserUpdateGuardGeneration;
	QTimer::singleShot(0, this,
					   [this, gen]()
					   {
						   if (gen != m_propertyBrowserUpdateGuardGeneration)
						   {
							   return;
						   }
						   m_updatingPropertyBrowser = false;
					   });
}

void MainWindow::updatePropertyPanel(const QString& backendId)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	if (m_propertyPanelDeferFullRebuild && backendId != m_propertyPanelActiveEditContextId)
	{
		m_propertyPanelActiveEditKey.clear();
		m_propertyPanelActiveEditContextId.clear();
		m_propertyPanelDeferFullRebuild = false;
		m_propertyVisualPreviewTimer.stop();
		m_propertyVisualPreviewBackendId.clear();
		if (DocumentPage* docPage = currentPage())
		{
			docPage->setDeferPropertyPanelVisualFullSync(false);
		}
	}
	if (!backendId.isEmpty() && shouldDeferPropertyPanelRebuild(backendId))
	{
		syncPropertyPanelRowValues(backendId);
		return;
	}
	if (!backendId.isEmpty() && m_followTargetNameDebounceTimer.isActive() &&
		backendId == m_followTargetNameDebounceBackendId)
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
	beginPropertyBrowserProgrammaticUpdate();
	clearPropertyKeyVariantMap();
	m_variantManager->clear();
	if (backendId.isEmpty())
	{
		endPropertyBrowserProgrammaticUpdate();
		return;
	}

	DocumentPage* docPage = currentPage();
	if (!docPage || !docPage->data().isValid(backendId))
	{
		endPropertyBrowserProgrammaticUpdate();
		return;
	}
	const QVector<cloudsim::core::PropertyRowDto> rows = docPage->data().propertyRows(backendId);
	QColor objectColor;
	const bool hasObjectColor = colorFromPropertyRows(rows, &objectColor);
	for (const cloudsim::core::PropertyRowDto& r : rows)
	{
		QString key = r.key;
		if (isColorComponentKey(key))
		{
			continue;
		}
		bool editable = r.editable;
		if (key == QStringLiteral("follow.targetName"))
		{
			editable = true;
		}
		const QString label = propertyDisplayLabelForKey(key, r.labelEn);
		appendPropertyBrowserRow(key, label, r.value, editable);
	}
	if (hasObjectColor)
	{
		appendColorPropertyBrowserRow(objectColor);
	}

	const bool showAxis = docPage->data().geometryKind(backendId) == cloudsim::core::GeometryKind::Points;
	if (showAxis)
	{
		const QString axisKey = QStringLiteral("ui.active_axis");
		appendPropertyBrowserRow(axisKey, propertyDisplayLabelForKey(axisKey, QStringLiteral("Active axis")),
								 m_activeAxisName, false);
	}

	endPropertyBrowserProgrammaticUpdate();
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

	DocumentPage* docPage = currentPage();
	if (!docPage)
	{
		return;
	}

	const QString propertyKey = property->whatsThis();
	if (m_propertyKeyToVariant.value(propertyKey) != property)
	{
		return; // 上一对象编辑器残留
	}
	if (propertyKey == QStringLiteral("color"))
	{
		QColor qc = qvariant_cast<QColor>(value);
		if (!qc.isValid() && m_variantManager)
		{
			qc = qvariant_cast<QColor>(m_variantManager->value(static_cast<QtVariantProperty*>(property)));
		}
		if (!qc.isValid())
		{
			return;
		}
		cloudsim::core::ColorDto colorDto;
		colorDto.r = static_cast<float>(qc.redF());
		colorDto.g = static_cast<float>(qc.greenF());
		colorDto.b = static_cast<float>(qc.blueF());
		colorDto.a = static_cast<float>(qc.alphaF());
		QString dsErr;
		if (!docPage->data().applyColor(backendId, colorDto, &dsErr))
		{
			updatePropertyPanel(backendId);
			return;
		}
		cloudsim::host::syncVisualAfterPropertyChangeById(*docPage, backendId, true);
		schedulePropertyPanelCommitRefresh(backendId);
		return;
	}
	if (propertyKey == QStringLiteral("follow.targetName"))
	{
		const QString text = variantValueToString(value);
		if (text == propertyRowValue(*docPage, backendId, propertyKey))
		{
			return;
		}
		if (text.trimmed().isEmpty() &&
			!docPage->data().hasComponent(backendId, QStringLiteral("FollowAttachment")))
		{
			return;
		}
		m_followTargetNameDebounceBackendId = backendId;
		m_followTargetNameDebounceText = text;
		m_followTargetNameDebounceTimer.start(400);
		return;
	}
	if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("ui.")))
	{
		return;
	}
	const QString valueText = variantValueToString(value);
	const QString currentText = propertyRowValue(*docPage, backendId, propertyKey);
	if (valueText == currentText)
	{
		return;
	}
	const bool isPoseRotEdit = isPoseComponentKey(propertyKey) || isRotationComponentKey(propertyKey);
	if (isPoseRotEdit && propertyNumericValuesEqual(value, currentText))
	{
		return;
	}

	cloudsim::core::PoseDto poseBefore{};
	if (isPoseRotEdit)
	{
		poseBefore = docPage->data().worldPoseMm(backendId);
	}

	QString dsErr;
	const bool applyOk = docPage->data().applyPropertyChange(backendId, propertyKey, valueText, &dsErr);
	if (!applyOk)
	{
		updatePropertyPanel(backendId);
		return;
	}

	const bool followKey = propertyKey.startsWith(QStringLiteral("follow."));
	if (followKey)
	{
		afterBackendFollowPropertyEdited(propertyKey, valueText);
	}
	else if (isPoseRotEdit)
	{
		if (poseDtoNearlyEqual(poseBefore, docPage->data().worldPoseMm(backendId)))
		{
			return;
		}
		beginPropertyPanelNumericEdit(backendId, propertyKey);
		docPage->markFollowAttachmentDirtyFromBackendMove(backendId);
	}

	if (isPoseRotEdit)
	{
		scheduleThrottledPropertyVisualPreview(backendId);
	}
	else
	{
		schedulePropertyPanelCommitRefresh(backendId);
	}
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
		RunLogger::debug(std::string("[PropertyCommitDBG] skip follow name commit without document page id=") +
						 backendId.toStdString());
		RunLogger::flush();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}
	if (m_followTargetNameDebounceText.trimmed().isEmpty() &&
		!docPage->data().hasComponent(backendId, QStringLiteral("FollowAttachment")))
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}

	QString dsErr;
	const bool applyOk =
		docPage->data().applyPropertyChange(backendId, propertyKey, m_followTargetNameDebounceText, &dsErr);
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
