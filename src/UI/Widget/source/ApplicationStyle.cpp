#include "ApplicationStyle.h"

#include "UiIcons.h"

#include "UiIconDecorators.h"

#include <QApplication>
#include <QColor>
#include <QPalette>
#include <QSettings>
#include <QStyleFactory>

namespace {

QString themeToString(ApplicationStyle::Theme t)
{
	return t == ApplicationStyle::Theme::Dark ? QStringLiteral("dark") : QStringLiteral("light");
}

ApplicationStyle::Theme themeFromString(const QString& s)
{
	return s.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0
		? ApplicationStyle::Theme::Dark
		: ApplicationStyle::Theme::Light;
}

QPalette makeLightPalette()
{
	QPalette p;
	p.setColor(QPalette::Window, QColor(240, 240, 240));
	p.setColor(QPalette::WindowText, Qt::black);
	p.setColor(QPalette::Base, Qt::white);
	p.setColor(QPalette::AlternateBase, QColor(233, 231, 227));
	p.setColor(QPalette::ToolTipBase, Qt::white);
	p.setColor(QPalette::ToolTipText, Qt::black);
	p.setColor(QPalette::Text, Qt::black);
	p.setColor(QPalette::Button, QColor(240, 240, 240));
	p.setColor(QPalette::ButtonText, Qt::black);
	p.setColor(QPalette::BrightText, Qt::red);
	p.setColor(QPalette::Link, QColor(0, 100, 200));
	p.setColor(QPalette::Highlight, QColor(0, 120, 215));
	p.setColor(QPalette::HighlightedText, Qt::white);
	p.setColor(QPalette::Disabled, QPalette::Text, QColor(120, 120, 120));
	p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(120, 120, 120));
	return p;
}

QPalette makeDarkPalette()
{
	QPalette p;
	p.setColor(QPalette::Window, QColor(53, 53, 53));
	p.setColor(QPalette::WindowText, Qt::white);
	p.setColor(QPalette::Base, QColor(35, 35, 35));
	p.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
	p.setColor(QPalette::ToolTipBase, QColor(60, 60, 60));
	p.setColor(QPalette::ToolTipText, Qt::white);
	p.setColor(QPalette::Text, Qt::white);
	p.setColor(QPalette::Button, QColor(53, 53, 53));
	p.setColor(QPalette::ButtonText, Qt::white);
	p.setColor(QPalette::BrightText, Qt::red);
	p.setColor(QPalette::Link, QColor(66, 163, 230));
	p.setColor(QPalette::Highlight, QColor(42, 130, 218));
	p.setColor(QPalette::HighlightedText, Qt::black);
	p.setColor(QPalette::Disabled, QPalette::Text, QColor(130, 130, 130));
	p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(130, 130, 130));
	return p;
}

} // namespace

namespace ApplicationStyle {

void applyTheme(QApplication* app, Theme theme)
{
	if (!app)
	{
		return;
	}
	app->setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
	app->setStyleSheet(QString());
	app->setPalette(theme == Theme::Dark ? makeDarkPalette() : makeLightPalette());

	UiIcons::setTheme(theme == Theme::Dark ? UiIcons::Theme::Dark : UiIcons::Theme::Light);
	UiIcons::invalidateCache();
	UiIconDecorators::refreshAllDecorated();
}

Theme loadSavedTheme()
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	return themeFromString(settings.value(QStringLiteral("theme"), QStringLiteral("light")).toString());
}

void saveTheme(Theme theme)
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	settings.setValue(QStringLiteral("theme"), themeToString(theme));
}

bool usesDarkTheme()
{
	return loadSavedTheme() == Theme::Dark;
}

} // namespace ApplicationStyle
