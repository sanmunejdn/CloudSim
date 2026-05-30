#pragma once

#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "IRobotPropertyPanelHost.h"
#include "robotwidget_global.h"

#include <functional>
#include <memory>

struct PickResult;
enum class PickKind;

class QStatusBar;
class RunInfoPage;
namespace RobotInstruction
{
struct FeasibleMotionAxisConfigurationOptions;
struct PlanResult;
class Base;
}
class SimulationCommandWidget;
class RobotAxisControlWidget;
class RobotFrameSettingsWidget;
class DevicePageWidget;
class QAction;
class QtProperty;
class QtTreePropertyBrowser;
class QtVariantPropertyManager;
class BackendDataBase;

/// 机器人编排所需主窗口服务（Widget 实现）
class ROBOTWIDGET_EXPORT IRobotMainWindowHost : public IRobotPropertyPanelHost
{
public:
	~IRobotMainWindowHost() override = default;

	virtual IRobotDocumentHost* document() = 0;
	virtual const IRobotDocumentHost* document() const = 0;
	virtual IRobotOsgViewHost* osgView() = 0;

	virtual bool useChinese() const = 0;
	virtual QString i18n(const QString& en, const QString& zh) const = 0;

	virtual RunInfoPage* runInfoPage() = 0;
	virtual void appendRunInfo(const QString& message) = 0;
	virtual void appendRunWarning(const QString& message) = 0;
	virtual QStatusBar* statusBar() = 0;

	virtual SimulationCommandWidget* simulationCommandPage() = 0;
	virtual RobotAxisControlWidget* robotAxisControlPage() = 0;
	virtual RobotFrameSettingsWidget* robotFrameSettingsPage() = 0;
	virtual DevicePageWidget* devicePage() = 0;

	virtual QAction* simulationStartAction() = 0;
	virtual int currentSimulationRobotInstanceIndex() const = 0;

	virtual void refreshBackendTree() = 0;
	virtual void runFollowSolveAndSyncForCurrentDocument() = 0;
	virtual void clearBackendObjectSelection(bool clearTreeSelection) = 0;

	virtual std::shared_ptr<RobotInstruction::Base> activeInstructionForProperty() const = 0;
	virtual void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible) = 0;

	virtual bool registerUrdfRobot(const QString& urdfPath, bool quietUi) = 0;

	virtual bool planRobotMotionInstruction(
		RobotInstruction::Base& instruction,
		const QVector<double>& seedJointRad,
		int instanceIndex,
		const QString& urdfPath,
		const QString& defaultTcpLinkName,
		const QString& sceneRootBackendId,
		RobotInstruction::PlanResult& out,
		std::string* outErr) = 0;

	virtual void setMeshPickCommittedHandler(std::function<void(const PickResult&, PickKind)> handler) = 0;
	virtual void clearMeshPickCommittedHandler() = 0;
};
