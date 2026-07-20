/// @file UiIconDecorators.cpp
/// @brief UiIconDecorators 实现

#include "UiIconDecorators.h"

#include <QAbstractButton>
#include <QAction>
#include <QList>
#include <QPointer>
#include <QSize>
#include <QToolButton>
#include <QVariant>
#include <algorithm>

namespace
{
struct ButtonDecoration
{
	QPointer<QAbstractButton> button;
	UiIconId id = UiIconId::Run;
	UiIconDecorators::IconPlacement placement = UiIconDecorators::IconPlacement::Leading;
	UiIcons::Size size = UiIcons::Size::Medium;
};

struct ActionDecoration
{
	QPointer<QAction> action;
	UiIconId id = UiIconId::Run;
	UiIcons::Size size = UiIcons::Size::Medium;
};

QList<ButtonDecoration>& buttonDecorations()
{
	static QList<ButtonDecoration> list;
	return list;
}

QList<ActionDecoration>& actionDecorations()
{
	static QList<ActionDecoration> list;
	return list;
}

void pruneDecorations()
{
	auto& buttons = buttonDecorations();
	buttons.erase(
		std::remove_if(buttons.begin(), buttons.end(), [](const ButtonDecoration& d) { return d.button.isNull(); }),
		buttons.end());

	auto& actions = actionDecorations();
	actions.erase(
		std::remove_if(actions.begin(), actions.end(), [](const ActionDecoration& d) { return d.action.isNull(); }),
		actions.end());
}

void applyButtonDecoration(const ButtonDecoration& deco)
{
	if (deco.button.isNull())
	{
		return;
	}

	const int px = static_cast<int>(deco.size);
	deco.button->setIcon(UiIcons::icon(deco.id, deco.size));
	deco.button->setIconSize(QSize(px, px));

	if (auto* toolBtn = qobject_cast<QToolButton*>(deco.button.data()))
	{
		switch (deco.placement)
		{
		case UiIconDecorators::IconPlacement::Leading:
			toolBtn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
			break;
		case UiIconDecorators::IconPlacement::TextUnderIcon:
			toolBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
			break;
		case UiIconDecorators::IconPlacement::IconOnly:
			toolBtn->setToolButtonStyle(Qt::ToolButtonIconOnly);
			break;
		}
	}
}

void registerButtonDecoration(QAbstractButton* btn, UiIconId id, UiIconDecorators::IconPlacement placement,
							  UiIcons::Size size)
{
	pruneDecorations();
	for (ButtonDecoration& deco : buttonDecorations())
	{
		if (deco.button == btn)
		{
			deco.id = id;
			deco.placement = placement;
			deco.size = size;
			applyButtonDecoration(deco);
			return;
		}
	}
	buttonDecorations().append(ButtonDecoration{btn, id, placement, size});
	applyButtonDecoration(buttonDecorations().constLast());
}

void registerActionDecoration(QAction* action, UiIconId id, UiIcons::Size size)
{
	pruneDecorations();
	for (ActionDecoration& deco : actionDecorations())
	{
		if (deco.action == action)
		{
			deco.id = id;
			deco.size = size;
			action->setIcon(UiIcons::icon(id, size));
			return;
		}
	}
	actionDecorations().append(ActionDecoration{action, id, size});
	action->setIcon(UiIcons::icon(id, size));
}

} // namespace

namespace UiIconDecorators
{
void apply(QAbstractButton* btn, UiIconId id, IconPlacement placement, UiIcons::Size size)
{
	if (!btn)
	{
		return;
	}
	registerButtonDecoration(btn, id, placement, size);
}

void apply(QAction* action, UiIconId id, UiIcons::Size size)
{
	if (!action)
	{
		return;
	}
	registerActionDecoration(action, id, size);
}

void refreshAllDecorated()
{
	pruneDecorations();
	for (const ButtonDecoration& deco : buttonDecorations())
	{
		applyButtonDecoration(deco);
	}
	for (const ActionDecoration& deco : actionDecorations())
	{
		if (!deco.action.isNull())
		{
			deco.action->setIcon(UiIcons::icon(deco.id, deco.size));
		}
	}
}

} // namespace UiIconDecorators
