#pragma once

/// @file PluginSidePanelLayout.h
/// @brief 插件侧栏窄折行/宽均分的控件策略与滚动区配置

#include "FlowLayout.h"

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QToolButton>
#include <QWidget>

inline void applySidePanelControlPolicies(QWidget* root)
{
	if (!root)
	{
		return;
	}
	if (qobject_cast<QPushButton*>(root) || qobject_cast<QToolButton*>(root))
	{
		root->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
		root->setMinimumWidth(0);
	}
	else if (qobject_cast<QAbstractSpinBox*>(root))
	{
		root->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
		root->setMinimumWidth(56);
	}
	else if (auto* combo = qobject_cast<QComboBox*>(root))
	{
		combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
		combo->setMinimumWidth(56);
		combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
		combo->setMinimumContentsLength(4);
	}
	else if (qobject_cast<QLineEdit*>(root))
	{
		root->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
		root->setMinimumWidth(56);
	}
	else if (auto* lab = qobject_cast<QLabel*>(root))
	{
		lab->setMinimumWidth(0);
	}

	const auto children = root->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
	for (QWidget* child : children)
	{
		applySidePanelControlPolicies(child);
	}
}

inline void configureSidePanelScroll(QScrollArea* scroll, QWidget* content)
{
	if (!scroll)
	{
		return;
	}
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	if (content)
	{
		content->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
		scroll->setWidget(content);
	}
}
