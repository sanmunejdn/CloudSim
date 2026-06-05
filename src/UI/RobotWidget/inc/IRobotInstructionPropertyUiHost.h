#pragma once

#include "robotwidget_global.h"

#include <QHash>
#include <QVariant>
#include <QVector>
#include <QString>
#include <QStringList>

#include <functional>
#include <memory>
#include <vector>

class IRobotDocumentHost;
class QtProperty;
class QtTreePropertyBrowser;
class QtVariantPropertyManager;
class SimulationCommandWidget;

namespace RobotInstruction
{
class Base;
struct FeasibleMotionAxisConfigurationOptions;
}

namespace cloudsim::core
{
struct FeasibleMotionAxisOptionsDto;
}

/// 仿真指令属性面板所需 MainWindow 能力（Widget 不 include RobotInstruction 头）
class ROBOTWIDGET_EXPORT IRobotInstructionPropertyUiHost
{
public:
	virtual ~IRobotInstructionPropertyUiHost() = default;

	virtual QtTreePropertyBrowser* propertyBrowser() = 0;
	virtual QtVariantPropertyManager* variantManager() = 0;
	virtual bool& updatingPropertyBrowserFlag() = 0;
	virtual QHash<QtProperty*, QStringList>& propertyEnumTokens() = 0;

	virtual IRobotDocumentHost* currentRobotDocument() = 0;
	virtual SimulationCommandWidget* simulationCommandPage() = 0;

	virtual bool applyInstructionPropertyChange(const QString& instructionId, const QString& key,
		const QString& value, QString* outError = nullptr) = 0;
	virtual int currentSimulationRobotInstanceIndex() const = 0;
	virtual void appendRunInfoMessage(const QString& en, const QString& zh) = 0;

	virtual bool useChinese() const = 0;
	virtual QString i18n(const QString& en, const QString& zh) const = 0;

	virtual void appendPropertyBrowserRow(
		const QString& propertyKey,
		const QString& displayLabel,
		const QString& value,
		bool editable,
		const std::vector<std::string>* enumOptionTokens = nullptr,
		const QStringList* enumDisplayNames = nullptr,
		const QString& toolTip = QString()) = 0;

	virtual QString propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const = 0;
	virtual QString instructionEnumTokenFromProperty(QtProperty* property, const QVariant& value) const = 0;

	virtual RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleMotionAxisConfigurationOptionsForInstruction(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		QVector<double>* outSeedJointRad = nullptr) = 0;

	virtual cloudsim::core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptionsDto() const = 0;

	virtual void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible) = 0;

	virtual void setActiveInstructionForProperty(const std::shared_ptr<RobotInstruction::Base>& instruction) = 0;
	virtual std::shared_ptr<RobotInstruction::Base> activeInstructionForProperty() const = 0;
	virtual void invalidateFeasibleAxisConfigurationCache() = 0;

	virtual void refreshInstructionPoseAxes() = 0;
	virtual void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction) = 0;
	virtual void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction) = 0;
	virtual void refreshRobotCoordinateFrameOverlays(const std::shared_ptr<RobotInstruction::Base>& instruction) = 0;

	virtual void scheduleInstructionPropertyRefresh(const std::shared_ptr<RobotInstruction::Base>& instruction,
		bool refreshFeasibleAxisOptions) = 0;
	virtual void scheduleDeferredFeasibleAxisProbe(const std::shared_ptr<RobotInstruction::Base>& instruction) = 0;
};
