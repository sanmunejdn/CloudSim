/// @file PropertyPanelVariantEditorFactory.cpp
/// @brief 属性面板字符串编辑器工厂

#include "PropertyPanelVariantEditorFactory.h"

#include "MainWindow.h"

#include <QLineEdit>

PropertyPanelVariantEditorFactory::PropertyPanelVariantEditorFactory(MainWindow* mainWindow)
	: m_mainWindow(mainWindow)
{
}

QWidget* PropertyPanelVariantEditorFactory::createEditor(QtVariantPropertyManager* manager, QtProperty* property,
														 QWidget* parent)
{
	QWidget* editor = QtVariantEditorFactory::createEditor(manager, property, parent);
	if (!editor || !m_mainWindow)
	{
		return editor;
	}
	if (auto* lineEdit = qobject_cast<QLineEdit*>(editor))
	{
		const QString propertyKey = property->whatsThis();
		lineEdit->installEventFilter(m_mainWindow);
		QObject::connect(lineEdit, &QLineEdit::editingFinished, m_mainWindow,
						 [mw = m_mainWindow, propertyKey]() { mw->commitInlineTextPropertyEdit(propertyKey); });
	}
	return editor;
}
