/// @file IndustrialCameraDockWidget.cpp
/// @brief 单侧栏 + 内嵌 Tab

#include "IndustrialCameraDockWidget.h"

#include "CameraPanelWidget.h"
#include "HandEyePanelWidget.h"
#include "IPluginHostContext.h"

#include <QTabWidget>
#include <QVBoxLayout>

IndustrialCameraDockWidget::IndustrialCameraDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, host_(host)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	tabs_ = new QTabWidget(this);
	cameraPanel_ = new CameraPanelWidget(host, tabs_);
	handEyePanel_ = new HandEyePanelWidget(cameraPanel_, host, tabs_);
	tabs_->addTab(cameraPanel_, QStringLiteral("Camera"));
	tabs_->addTab(handEyePanel_, QStringLiteral("Hand-Eye"));
	root->addWidget(tabs_);
	applyLanguage();
}

void IndustrialCameraDockWidget::setUseChinese(bool zh)
{
	zh_ = zh;
	if (cameraPanel_)
		cameraPanel_->setUseChinese(zh);
	if (handEyePanel_)
		handEyePanel_->setUseChinese(zh);
}

void IndustrialCameraDockWidget::applyLanguage()
{
	if (cameraPanel_)
		cameraPanel_->applyLanguage();
	if (handEyePanel_)
		handEyePanel_->applyLanguage();
	if (!tabs_)
		return;
	tabs_->setTabText(0, zh_ ? QStringLiteral("相机") : QStringLiteral("Camera"));
	tabs_->setTabText(1, zh_ ? QStringLiteral("手眼标定") : QStringLiteral("Hand-Eye"));
}
