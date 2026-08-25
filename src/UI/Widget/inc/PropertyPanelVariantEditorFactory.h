#ifndef WIDGET_PROPERTYPANELVARIANTEDITORFACTORY_H
#define WIDGET_PROPERTYPANELVARIANTEDITORFACTORY_H

/// @file PropertyPanelVariantEditorFactory.h
/// @brief 属性面板字符串编辑器：editingFinished 提交，避免 IME 组字期面板重建

#include <qtvariantproperty.h>

class MainWindow;

class PropertyPanelVariantEditorFactory final : public QtVariantEditorFactory
{
public:
	explicit PropertyPanelVariantEditorFactory(MainWindow* mainWindow);

protected:
	QWidget* createEditor(QtVariantPropertyManager* manager, QtProperty* property, QWidget* parent) override;

private:
	MainWindow* m_mainWindow = nullptr;
};

#endif // WIDGET_PROPERTYPANELVARIANTEDITORFACTORY_H
