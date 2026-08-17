/// @file TrajectoryGenerationPageWidget.cpp
/// @brief TrajectoryGenerationPage 控件

#include "TrajectoryGenerationPageWidget.h"

#include "FeatureTrajectoryPageWidget.h"
#include "MeshTrajectoryPageWidget.h"
#include "ProgramEditCommand.h"
#include "ProgramEditService.h"
#include "RawTrajectory.h"
#include "RobotProgramStore.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditSession.h"
#include "UiIconDecorators.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QShowEvent>
#include <QTabWidget>
#include <QVBoxLayout>

namespace

{
QString pathPlanComboLabel(const RobotInstruction::PathPlanInstruction& pp, const bool chinese)

{
	QString title = QString::fromStdString(pp.name().empty() ? pp.id() : pp.name());

	if (!pp.sourceFeatureJson().empty())

	{
		RobotInstruction::RawTrajectory traj;

		traj.sourceFeatureJson = pp.sourceFeatureJson();

		const std::string featureId = RobotInstruction::rawTrajectoryFeatureId(traj);

		if (!featureId.empty())

		{
			title += QStringLiteral(" · ") + QString::fromStdString(featureId);
		}
	}

	const QString phase = pp.phase() == RobotInstruction::PathPlanPhase::Applied

							  ? (chinese ? QStringLiteral("已应用") : QStringLiteral("applied"))

							  : (pp.phase() == RobotInstruction::PathPlanPhase::RawReady

									 ? (chinese ? QStringLiteral("已离散") : QStringLiteral("raw_ready"))

									 : (chinese ? QStringLiteral("草稿") : QStringLiteral("draft")));

	return title + QStringLiteral(" · ") + phase;
}

} // namespace

TrajectoryGenerationPageWidget::TrajectoryGenerationPageWidget(QWidget* parent)

	: QWidget(parent)

{
	auto* layout = new QVBoxLayout(this);

	layout->setContentsMargins(0, 0, 0, 0);

	m_pathPlanBar = new QHBoxLayout;

	m_pathPlanBar->setSpacing(4);

	m_pathPlanLabel = new QLabel(this);

	m_pathPlanCombo = new QComboBox(this);

	m_newPathPlanBtn = new QPushButton(QStringLiteral("+"), this);

	m_newPathPlanBtn->setFixedWidth(28);

	m_beginEditBtn = new QPushButton(this);

	m_cancelEditBtn = new QPushButton(this);

	layout->addLayout(m_pathPlanBar);

	m_pathPlanBar->addWidget(m_pathPlanLabel);

	m_pathPlanBar->addWidget(m_pathPlanCombo, 1);

	m_pathPlanBar->addWidget(m_newPathPlanBtn);

	m_pathPlanBar->addWidget(m_beginEditBtn);

	m_pathPlanBar->addWidget(m_cancelEditBtn);

	m_tabs = new QTabWidget(this);

	layout->addWidget(m_tabs, 1);

	m_brepPage = new FeatureTrajectoryPageWidget(m_tabs);

	m_meshPage = new MeshTrajectoryPageWidget(m_tabs);

	m_tabs->addTab(m_brepPage, QStringLiteral("CAD/BREP"));

	m_tabs->addTab(m_meshPage, QStringLiteral("Mesh"));

	connect(m_pathPlanCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,

			&TrajectoryGenerationPageWidget::onPathPlanComboChanged);

	connect(m_newPathPlanBtn, &QPushButton::clicked, this, &TrajectoryGenerationPageWidget::onNewPathPlanClicked);

	connect(m_beginEditBtn, &QPushButton::clicked, this, &TrajectoryGenerationPageWidget::onBeginEditClicked);

	connect(m_cancelEditBtn, &QPushButton::clicked, this, &TrajectoryGenerationPageWidget::onCancelEditClicked);

	connect(m_brepPage, &FeatureTrajectoryPageWidget::featureEditActiveChanged, this,
			[this](bool) { updateEditButtonsState(); });

	UiIconDecorators::apply(m_newPathPlanBtn, UiIconId::NewPathPlan);

	setUseChinese(m_chinese);

	updateEditButtonsState();
}

void TrajectoryGenerationPageWidget::setUseChinese(const bool chinese)

{
	m_chinese = chinese;

	m_brepPage->setUseChinese(chinese);

	m_meshPage->setUseChinese(chinese);

	m_tabs->setTabText(0, chinese ? QStringLiteral("CAD/BREP") : QStringLiteral("CAD/BREP"));

	m_tabs->setTabText(1, chinese ? QStringLiteral("Mesh") : QStringLiteral("Mesh"));

	updatePathPlanBarLabels();
}

void TrajectoryGenerationPageWidget::updatePathPlanBarLabels()

{
	if (m_pathPlanLabel)

	{
		m_pathPlanLabel->setText(m_chinese ? QStringLiteral("路径规划") : QStringLiteral("Path plan"));
	}

	if (m_newPathPlanBtn)

	{
		m_newPathPlanBtn->setToolTip(m_chinese ? QStringLiteral("新建空路径规划")

											   : QStringLiteral("Create empty path plan"));
	}

	if (m_beginEditBtn)

	{
		m_beginEditBtn->setText(m_chinese ? QStringLiteral("开始修改") : QStringLiteral("Edit"));

		m_beginEditBtn->setToolTip(
			m_chinese ? QStringLiteral("从当前路径规划加载特征、离散参数与算子流程；未点击前不会自动重离散或预显示")
					  : QStringLiteral("Load features, discretization params and pipeline from bound path plan"));
	}

	if (m_cancelEditBtn)

	{
		m_cancelEditBtn->setText(m_chinese ? QStringLiteral("取消修改") : QStringLiteral("Cancel Edit"));

		m_cancelEditBtn->setToolTip(
			m_chinese ? QStringLiteral("退出修改态：清空特征表并关闭预览；已写入路径规划的内容保留")
					  : QStringLiteral("Exit edit mode; clear feature table and preview; persisted path plan kept"));
	}
}

void TrajectoryGenerationPageWidget::bindHost(IRobotMainWindowHost* host)

{
	m_brepPage->bindHost(host);

	m_meshPage->bindHost(host);
}

void TrajectoryGenerationPageWidget::bindSession(TrajectoryEditSession* session)

{
	if (m_session)

	{
		disconnect(m_session, nullptr, this, nullptr);
	}

	m_session = session;

	m_brepPage->bindSession(session);

	m_meshPage->bindSession(session);

	if (m_session)

	{
		connect(m_session, &TrajectoryEditSession::pathPlanBound, this,

				&TrajectoryGenerationPageWidget::onPathPlanBound);
	}

	refreshPathPlanCombo();
}

void TrajectoryGenerationPageWidget::bindSimulationController(RobotSimulationController* controller)

{
	m_brepPage->bindSimulationController(controller);

	m_meshPage->bindSimulationController(controller);
}

void TrajectoryGenerationPageWidget::bindStore(RobotProgramStore* store)

{
	m_store = store;

	refreshPathPlanCombo();
}

void TrajectoryGenerationPageWidget::bindEditService(ProgramEditService* service)

{
	if (m_editService)

	{
		disconnect(m_editService, nullptr, this, nullptr);
	}

	m_editService = service;

	if (m_editService)

	{
		connect(m_editService, &ProgramEditService::revisionChanged, this,
				[this](int)
				{
					refreshPathPlanCombo();
				});
	}
}

void TrajectoryGenerationPageWidget::bindCommandPage(SimulationCommandWidget* commandPage)

{
	m_commandPage = commandPage;
}

void TrajectoryGenerationPageWidget::setStepPathResolver(

	std::function<QString(const QString& backendId)> resolver)

{
	m_brepPage->setStepPathResolver(std::move(resolver));
}

void TrajectoryGenerationPageWidget::refreshWorkpieces()
{
	if (m_brepPage)
		m_brepPage->refreshWorkpieces();
	if (m_meshPage)
		m_meshPage->refreshWorkpieces();
}

void TrajectoryGenerationPageWidget::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	refreshWorkpieces();
}

void TrajectoryGenerationPageWidget::resetAfterTrajectoryCommit()

{
	if (m_brepPage)

	{
		m_brepPage->resetAfterTrajectoryCommit();
	}

	if (m_meshPage)

	{
		m_meshPage->resetAfterTrajectoryCommit();
	}
}

void TrajectoryGenerationPageWidget::refreshPathPlanCombo()

{
	if (!m_pathPlanCombo || !m_store)

	{
		return;
	}

	const std::string activeId = m_store->activeProgramIdUtf8();

	const std::vector<RobotInstruction::PathPlanInstruction*> plans =

		m_store->activeCatalog().listPathPlans(activeId);

	QString prevId;

	if (m_session && !m_session->boundPathPlanId().empty())

	{
		prevId = QString::fromStdString(m_session->boundPathPlanId());
	}

	else if (m_pathPlanCombo->currentIndex() >= 0)

	{
		prevId = m_pathPlanCombo->currentData().toString();
	}

	m_pathPlanCombo->blockSignals(true);

	m_pathPlanCombo->clear();

	int selectIdx = -1;

	for (size_t i = 0; i < plans.size(); ++i)

	{
		const RobotInstruction::PathPlanInstruction* pp = plans[i];

		if (!pp)

		{
			continue;
		}

		m_pathPlanCombo->addItem(pathPlanComboLabel(*pp, m_chinese), QString::fromStdString(pp->id()));

		if (!prevId.isEmpty() && prevId == QString::fromStdString(pp->id()))

		{
			selectIdx = static_cast<int>(i);
		}
	}

	if (selectIdx >= 0)

	{
		m_pathPlanCombo->setCurrentIndex(selectIdx);
	}

	m_pathPlanCombo->setEnabled(!m_readOnly && !plans.empty());

	if (m_newPathPlanBtn)

	{
		m_newPathPlanBtn->setEnabled(!m_readOnly);
	}

	m_pathPlanCombo->blockSignals(false);
}

void TrajectoryGenerationPageWidget::onPathPlanComboChanged(int index)

{
	(void)index;

	if (!m_store || !m_pathPlanCombo || !m_session || m_pathPlanCombo->currentIndex() < 0)

	{
		return;
	}

	const std::string pathPlanId = m_pathPlanCombo->currentData().toString().toStdString();

	if (pathPlanId.empty())

	{
		return;
	}

	m_session->bindPathPlan(pathPlanId);
}

void TrajectoryGenerationPageWidget::onPathPlanBound(const std::string& pathPlanId)

{
	(void)pathPlanId;

	refreshPathPlanCombo();
}

void TrajectoryGenerationPageWidget::onNewPathPlanClicked()

{
	if (!m_store || !m_editService || !m_session || m_readOnly)

	{
		return;
	}

	size_t insertIdx = 0;

	for (const std::shared_ptr<RobotInstruction::Base>& step : m_store->activeProgram())

	{
		if (step && step->type() == RobotInstruction::Type::PathPlan)

		{
			++insertIdx;
		}
	}

	auto pathPlan = std::make_shared<RobotInstruction::PathPlanInstruction>();

	const std::string baseName = m_chinese ? "路径规划" : "path_plan";

	pathPlan->setName(baseName + "_" + std::to_string(insertIdx + 1));

	pathPlan->setRawTrajectoryKey(pathPlan->id());

	std::shared_ptr<RobotInstruction::ProgramEditCommand> cmd =

		std::make_shared<RobotInstruction::InsertPathPlanCommand>(pathPlan, insertIdx);

	QString err;

	if (!m_editService->execute(cmd, &err))

	{
		return;
	}

	if (m_commandPage)

	{
		m_commandPage->refreshInstructionList();
	}

	m_session->bindPathPlan(pathPlan->id());

	refreshPathPlanCombo();
}

void TrajectoryGenerationPageWidget::onBeginEditClicked()

{
	if (!m_brepPage)

	{
		return;
	}

	(void)m_brepPage->beginEditBoundPathPlan();
	updateEditButtonsState();
}

void TrajectoryGenerationPageWidget::onCancelEditClicked()

{
	if (!m_brepPage)

	{
		return;
	}

	m_brepPage->cancelEditBoundPathPlan();
	updateEditButtonsState();
}

void TrajectoryGenerationPageWidget::updateEditButtonsState()

{
	const bool editing = m_brepPage && m_brepPage->isFeatureEditActive();
	if (m_cancelEditBtn)

	{
		m_cancelEditBtn->setEnabled(editing);
	}
}
