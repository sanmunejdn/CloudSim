/// @file IndustrialCameraDockWidget.cpp
/// @brief 单侧栏 + 内嵌 Tab

#include "IndustrialCameraDockWidget.h"

#include "CameraPanelWidget.h"
#include "HandEyePanelWidget.h"
#include "IPluginHostContext.h"
#include "VisionGraspPanelWidget.h"

#include <QTabWidget>
#include <QVBoxLayout>

IndustrialCameraDockWidget::IndustrialCameraDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent), host_(host)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	tabs_ = new QTabWidget(this);
	cameraPanel_ = new CameraPanelWidget(host, tabs_);
	handEyePanel_ = new HandEyePanelWidget(cameraPanel_, host, tabs_);
	visionGraspPanel_ = new VisionGraspPanelWidget(handEyePanel_, host, tabs_);
	tabs_->addTab(cameraPanel_, QStringLiteral("Camera"));
	tabs_->addTab(handEyePanel_, QStringLiteral("Hand-Eye"));
	tabs_->addTab(visionGraspPanel_, QStringLiteral("Vision Grasp"));
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
	if (visionGraspPanel_)
		visionGraspPanel_->setUseChinese(zh);
}

void IndustrialCameraDockWidget::applyLanguage()
{
	if (cameraPanel_)
		cameraPanel_->applyLanguage();
	if (handEyePanel_)
		handEyePanel_->applyLanguage();
	if (visionGraspPanel_)
		visionGraspPanel_->applyLanguage();
	if (!tabs_)
		return;
	tabs_->setTabText(0, zh_ ? QStringLiteral("相机") : QStringLiteral("Camera"));
	tabs_->setTabText(1, zh_ ? QStringLiteral("手眼标定") : QStringLiteral("Hand-Eye"));
	tabs_->setTabText(2, zh_ ? QStringLiteral("视觉抓取") : QStringLiteral("Vision Grasp"));
}
