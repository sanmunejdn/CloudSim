/// @file TrajectoryPlanConfirmDialog.cpp
/// @brief AI 轨迹离散确认对话框：策略/参数/算子同屏编辑

#include "TrajectoryPlanConfirmDialog.h"

#include "FeatureDiscretizerParamPanel.h"
#include "TrajectoryOpParamPanel.h"
#include "TrajectoryPipelineListWidget.h"

#include <GeometryRef.h>
#include <ITrajectoryOp.h>
#include <RecipeBlueprint.h>
#include <TrajectoryOpBridge.h>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSplitter>
#include <QVBoxLayout>

#include <QCoreApplication>

namespace
{
void ensureDiscretizersLoaded()
{
	geometry_backend_ops::ensureFeatureDiscretizersRegistered();
	const std::string appDir = QCoreApplication::applicationDirPath().toStdString();
	(void)geometry_backend_ops::ensureFeatureDiscretizerConfigsLoaded(appDir, nullptr);
}

void ensureOpsLoaded()
{
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();
	const std::string appDir = QCoreApplication::applicationDirPath().toStdString();
	(void)RobotInstruction::ensureTrajectoryOpConfigsLoaded(appDir, nullptr);
}

RobotInstruction::RecipeKind recipeKindFromTemplate(const std::string& t)
{
	if (t.find("glue") != std::string::npos)
		return RobotInstruction::RecipeKind::Glue;
	if (t.find("grind") != std::string::npos)
		return RobotInstruction::RecipeKind::Grind;
	return RobotInstruction::RecipeKind::Weld;
}

geoalgo::GeometryAffinity affinityOfFeature(const nlohmann::json& feat)
{
	if (feat.contains("geometry") && feat["geometry"].is_object())
	{
		const auto& g = feat["geometry"];
		const bool hasFace = g.contains("faceIndices") && g["faceIndices"].is_array() && !g["faceIndices"].empty();
		const bool hasEdge = g.contains("edgeIndices") && g["edgeIndices"].is_array() && !g["edgeIndices"].empty();
		if (hasFace && !hasEdge)
			return geoalgo::GeometryAffinity::Face;
		if (hasEdge && !hasFace)
			return geoalgo::GeometryAffinity::Line;
	}
	const std::string sid = feat.value("strategyId", feat.value("kind", std::string()));
	if (!sid.empty())
		return geometry_backend_ops::featureDiscretizerAffinity(sid);
	return geoalgo::GeometryAffinity::Any;
}
} // namespace

TrajectoryPlanConfirmDialog::TrajectoryPlanConfirmDialog(QWidget* parent) : QDialog(parent)
{
	setWindowTitle(QStringLiteral("确认离散策略与管线算子"));
	setModal(true);
	resize(960, 640);

	auto* root = new QVBoxLayout(this);
	auto* splitter = new QSplitter(Qt::Horizontal, this);

	auto* left = new QWidget(splitter);
	auto* leftLay = new QVBoxLayout(left);
	leftLay->addWidget(new QLabel(QStringLiteral("特征"), left));
	m_featureList = new QListWidget(left);
	leftLay->addWidget(m_featureList, 1);
	auto* stratRow = new QHBoxLayout;
	stratRow->addWidget(new QLabel(QStringLiteral("离散策略"), left));
	m_strategyCombo = new QComboBox(left);
	stratRow->addWidget(m_strategyCombo, 1);
	leftLay->addLayout(stratRow);
	m_discretizePanel = new FeatureDiscretizerParamPanel(left);
	leftLay->addWidget(m_discretizePanel, 2);
	splitter->addWidget(left);

	auto* right = new QWidget(splitter);
	auto* rightLay = new QVBoxLayout(right);
	rightLay->addWidget(new QLabel(QStringLiteral("管线算子"), right));
	m_pipeline = new TrajectoryPipelineListWidget(right);
	rightLay->addWidget(m_pipeline, 2);
	auto* opBtnRow = new QHBoxLayout;
	m_opKindCombo = new QComboBox(right);
	opBtnRow->addWidget(m_opKindCombo, 1);
	auto* addBtn = new QPushButton(QStringLiteral("添加"), right);
	auto* rmBtn = new QPushButton(QStringLiteral("删除"), right);
	auto* upBtn = new QPushButton(QStringLiteral("上移"), right);
	auto* downBtn = new QPushButton(QStringLiteral("下移"), right);
	opBtnRow->addWidget(addBtn);
	opBtnRow->addWidget(rmBtn);
	opBtnRow->addWidget(upBtn);
	opBtnRow->addWidget(downBtn);
	rightLay->addLayout(opBtnRow);
	m_opParamPanel = new TrajectoryOpParamPanel(right);
	m_opParamPanel->setEditingRawCloud(true);
	rightLay->addWidget(m_opParamPanel, 2);
	splitter->addWidget(right);
	splitter->setStretchFactor(0, 1);
	splitter->setStretchFactor(1, 1);
	root->addWidget(splitter, 1);

	auto* btnRow = new QHBoxLayout;
	btnRow->addStretch();
	m_retryBtn = new QPushButton(QStringLiteral("返回重选"), this);
	m_retryBtn->setVisible(false);
	auto* cancelBtn = new QPushButton(QStringLiteral("取消"), this);
	auto* okBtn = new QPushButton(QStringLiteral("确认执行"), this);
	okBtn->setDefault(true);
	btnRow->addWidget(m_retryBtn);
	btnRow->addWidget(cancelBtn);
	btnRow->addWidget(okBtn);
	root->addLayout(btnRow);

	m_pipeline->setDefaultOpFactory([](const RobotInstruction::TrajectoryOpKind kind) {
		RobotInstruction::OpScope scope{};
		scope.kind = RobotInstruction::OpScope::Kind::EntireProgram;
		return RobotInstruction::trajectoryOpDefaultUnified(kind, scope);
	});

	connect(m_featureList, &QListWidget::currentRowChanged, this, &TrajectoryPlanConfirmDialog::onFeatureRowChanged);
	connect(m_strategyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&TrajectoryPlanConfirmDialog::onStrategyChanged);
	connect(m_discretizePanel, &FeatureDiscretizerParamPanel::paramsChanged, this,
			&TrajectoryPlanConfirmDialog::onDiscretizeParamsChanged);
	connect(m_pipeline, &TrajectoryPipelineListWidget::selectedOpChanged, this,
			&TrajectoryPlanConfirmDialog::onPipelineSelectionChanged);
	connect(m_pipeline, &TrajectoryPipelineListWidget::opsChanged, this,
			&TrajectoryPlanConfirmDialog::onPipelineOpsChanged);
	connect(m_opParamPanel, &TrajectoryOpParamPanel::paramsChanged, this,
			&TrajectoryPlanConfirmDialog::onOpParamsChanged);
	connect(addBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onAddOpClicked);
	connect(rmBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onRemoveOpClicked);
	connect(upBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onMoveOpUp);
	connect(downBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onMoveOpDown);
	connect(okBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onAcceptClicked);
	connect(cancelBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onCancelClicked);
	connect(m_retryBtn, &QPushButton::clicked, this, &TrajectoryPlanConfirmDialog::onRetryClicked);
}

void TrajectoryPlanConfirmDialog::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	m_discretizePanel->setUseChinese(chinese);
	m_opParamPanel->setUseChinese(chinese);
	m_pipeline->setUseChinese(chinese);
	setWindowTitle(chinese ? QStringLiteral("确认离散策略与管线算子")
						   : QStringLiteral("Confirm discretize strategy & pipeline"));
	if (m_retryBtn)
		m_retryBtn->setText(chinese ? QStringLiteral("返回重选") : QStringLiteral("Back to selection"));
}

void TrajectoryPlanConfirmDialog::setShowRetry(const bool show)
{
	m_retryBtn->setVisible(show);
}

bool TrajectoryPlanConfirmDialog::loadPlan(const QByteArray& planJsonUtf8, QString* err)
{
	try
	{
		ensureDiscretizersLoaded();
		ensureOpsLoaded();
		m_plan = nlohmann::json::parse(planJsonUtf8.constData(), nullptr, true);
		if (!m_plan.contains("features") || !m_plan["features"].is_array() || m_plan["features"].empty())
		{
			if (err)
				*err = m_useChinese ? QStringLiteral("计划缺少 features") : QStringLiteral("Missing features");
			return false;
		}
		rebuildOpKindCombo();
		rebuildFeatureList();
		std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
		if (m_plan.contains("pipeline") && m_plan["pipeline"].is_array())
		{
			std::string codecErr;
			if (!RobotInstruction::trajectoryPipelineFromJson(m_plan["pipeline"], ops, &codecErr))
			{
				if (err)
					*err = QString::fromStdString(codecErr);
				return false;
			}
		}
		m_loading = true;
		m_pipeline->setOps(ops);
		m_loading = false;
		m_lastFeatureRow = -1;
		if (m_featureList->count() > 0)
		{
			m_featureList->setCurrentRow(0);
			m_lastFeatureRow = 0;
			refreshStrategyComboForCurrent();
			loadDiscretizePanelForCurrent();
		}
		if (!ops.empty())
			onPipelineSelectionChanged(0);
		return true;
	}
	catch (...)
	{
		if (err)
			*err = m_useChinese ? QStringLiteral("计划 JSON 无效") : QStringLiteral("Invalid plan JSON");
		return false;
	}
}

QByteArray TrajectoryPlanConfirmDialog::resultPlanJson() const
{
	nlohmann::json out = m_plan;
	out["version"] = 2;
	out["pipeline"] = RobotInstruction::trajectoryPipelineToJson(m_pipeline->ops());
	return QByteArray::fromStdString(out.dump());
}

void TrajectoryPlanConfirmDialog::rebuildFeatureList()
{
	m_loading = true;
	m_featureList->clear();
	int i = 0;
	for (const auto& feat : m_plan["features"])
	{
		const QString id = QString::fromStdString(feat.value("featureId", std::string("f") + std::to_string(i)));
		const QString sid = QString::fromStdString(feat.value("strategyId", std::string()));
		m_featureList->addItem(QStringLiteral("%1 · %2").arg(id, sid));
		++i;
	}
	m_loading = false;
}

void TrajectoryPlanConfirmDialog::rebuildOpKindCombo()
{
	m_opKindCombo->clear();
	for (const RobotInstruction::TrajectoryOpKind kind : RobotInstruction::trajectoryOpPaletteKinds())
	{
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(kind);
		const QString label =
			algo ? QString::fromUtf8(algo->displayName(m_useChinese)) : QString::number(static_cast<int>(kind));
		m_opKindCombo->addItem(label, static_cast<int>(kind));
	}
}

geoalgo::GeometryAffinity TrajectoryPlanConfirmDialog::affinityForCurrentFeature() const
{
	const int row = m_featureList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_plan["features"].size()))
		return geoalgo::GeometryAffinity::Any;
	return affinityOfFeature(m_plan["features"][row]);
}

void TrajectoryPlanConfirmDialog::refreshStrategyComboForCurrent()
{
	ensureDiscretizersLoaded();
	const geoalgo::GeometryAffinity filter = affinityForCurrentFeature();
	const int row = m_featureList->currentRow();
	QString previous;
	if (row >= 0 && row < static_cast<int>(m_plan["features"].size()))
		previous = QString::fromStdString(m_plan["features"][row].value("strategyId", std::string()));

	m_loading = true;
	m_strategyCombo->clear();
	for (const std::string& id : geometry_backend_ops::featureDiscretizerListStrategyIds())
	{
		const geoalgo::GeometryAffinity a = geometry_backend_ops::featureDiscretizerAffinity(id);
		if (filter != geoalgo::GeometryAffinity::Any && a != filter && a != geoalgo::GeometryAffinity::Any)
			continue;
		const QString label = m_useChinese
								  ? QString::fromStdString(geometry_backend_ops::featureDiscretizerDisplayNameZh(id))
								  : QString::fromStdString(id);
		m_strategyCombo->addItem(label, QString::fromStdString(id));
	}
	int idx = previous.isEmpty() ? -1 : m_strategyCombo->findData(previous);
	if (idx < 0 && m_strategyCombo->count() > 0)
		idx = 0;
	if (idx >= 0)
		m_strategyCombo->setCurrentIndex(idx);
	m_loading = false;
}

void TrajectoryPlanConfirmDialog::loadDiscretizePanelForCurrent()
{
	const int row = m_featureList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_plan["features"].size()))
	{
		m_discretizePanel->clear();
		return;
	}
	auto& feat = m_plan["features"][row];
	const std::string sid = m_strategyCombo->currentData().toString().toStdString();
	if (sid.empty())
	{
		m_discretizePanel->clear();
		return;
	}
	feat["strategyId"] = sid;
	feat["kind"] = sid;
	m_loading = true;
	m_discretizePanel->rebuildForStrategy(sid);
	nlohmann::json params = feat.value("params", nlohmann::json::object());
	if (params.empty() && feat.contains("discretize") && feat["discretize"].is_object())
		params = feat["discretize"];
	if (params.empty())
		params = geometry_backend_ops::featureDiscretizerDefaultParams(sid);
	m_discretizePanel->loadParams(params);
	feat["params"] = params;
	m_loading = false;
}

void TrajectoryPlanConfirmDialog::flushCurrentFeatureParams()
{
	const int row = m_lastFeatureRow;
	if (row < 0 || row >= static_cast<int>(m_plan["features"].size()))
		return;
	nlohmann::json params;
	if (!m_discretizePanel->applyParams(params))
		return;
	auto& feat = m_plan["features"][row];
	const std::string sid = m_strategyCombo->currentData().toString().toStdString();
	if (!sid.empty())
	{
		feat["strategyId"] = sid;
		feat["kind"] = sid;
	}
	feat["params"] = params;
	feat.erase("discretize");
	const QString id = QString::fromStdString(feat.value("featureId", std::string()));
	if (QListWidgetItem* item = m_featureList->item(row))
		item->setText(QStringLiteral("%1 · %2").arg(id, QString::fromStdString(sid)));
}

void TrajectoryPlanConfirmDialog::flushSelectedOpParams()
{
	const int idx = m_pipeline->selectedOpIndex();
	if (idx < 0)
		return;
	RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->opAt(idx);
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
	std::string err;
	if (!m_opParamPanel->applyTo(op, algo, &err))
		return;
	m_loading = true;
	m_pipeline->updateOpAt(idx, op);
	m_loading = false;
}

void TrajectoryPlanConfirmDialog::onFeatureRowChanged(const int row)
{
	if (m_loading)
		return;
	if (m_lastFeatureRow >= 0)
		flushCurrentFeatureParams();
	m_lastFeatureRow = row;
	refreshStrategyComboForCurrent();
	loadDiscretizePanelForCurrent();
}

void TrajectoryPlanConfirmDialog::onStrategyChanged(const int index)
{
	(void)index;
	if (m_loading)
		return;
	loadDiscretizePanelForCurrent();
	flushCurrentFeatureParams();
}

void TrajectoryPlanConfirmDialog::onDiscretizeParamsChanged()
{
	if (m_loading || m_discretizePanel->isRebuilding())
		return;
	flushCurrentFeatureParams();
}

void TrajectoryPlanConfirmDialog::onPipelineSelectionChanged(const int index)
{
	if (index < 0)
	{
		m_opParamPanel->clear();
		return;
	}
	const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->opAt(index);
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
	m_loading = true;
	m_opParamPanel->rebuildForOp(op, algo);
	m_loading = false;
}

void TrajectoryPlanConfirmDialog::onPipelineOpsChanged()
{
	if (m_loading)
		return;
}

void TrajectoryPlanConfirmDialog::onOpParamsChanged()
{
	if (m_loading || m_opParamPanel->isRebuilding())
		return;
	flushSelectedOpParams();
}

void TrajectoryPlanConfirmDialog::onAddOpClicked()
{
	if (m_opKindCombo->currentIndex() < 0)
		return;
	const auto kind = static_cast<RobotInstruction::TrajectoryOpKind>(m_opKindCombo->currentData().toInt());
	RobotInstruction::OpScope scope{};
	scope.kind = RobotInstruction::OpScope::Kind::EntireProgram;
	auto op = RobotInstruction::trajectoryOpDefaultUnified(kind, scope);
	op.enabled = true;
	m_pipeline->appendOp(std::move(op));
}

void TrajectoryPlanConfirmDialog::onRemoveOpClicked()
{
	m_pipeline->removeSelectedOp();
}

void TrajectoryPlanConfirmDialog::onMoveOpUp()
{
	m_pipeline->moveSelectedOp(-1);
}

void TrajectoryPlanConfirmDialog::onMoveOpDown()
{
	m_pipeline->moveSelectedOp(1);
}

void TrajectoryPlanConfirmDialog::onAcceptClicked()
{
	m_lastFeatureRow = m_featureList->currentRow();
	flushCurrentFeatureParams();
	flushSelectedOpParams();
	m_outcome = Outcome::Accepted;
	accept();
}

void TrajectoryPlanConfirmDialog::onCancelClicked()
{
	m_outcome = Outcome::Cancelled;
	reject();
}

void TrajectoryPlanConfirmDialog::onRetryClicked()
{
	m_outcome = Outcome::Retry;
	done(static_cast<int>(Outcome::Retry));
}

bool enrichTrajectoryPlanJsonInPlace(nlohmann::json& plan, QString* err)
{
	ensureDiscretizersLoaded();
	ensureOpsLoaded();
	plan["version"] = 2;
	if (!plan.contains("features") || !plan["features"].is_array() || plan["features"].empty())
	{
		if (err)
			*err = QStringLiteral("计划缺少 features");
		return false;
	}
	for (auto& feat : plan["features"])
	{
		std::string sid = feat.value("strategyId", std::string());
		if (sid.empty())
			sid = feat.value("kind", std::string("EdgeChain"));
		if (sid == "FaceUVGrid")
			sid = "FaceSection";
		feat["strategyId"] = sid;
		feat["kind"] = sid;
		if (!feat.contains("params") || !feat["params"].is_object() || feat["params"].empty())
		{
			nlohmann::json merged = geometry_backend_ops::featureDiscretizerDefaultParams(sid);
			if (feat.contains("discretize") && feat["discretize"].is_object())
			{
				for (auto it = feat["discretize"].begin(); it != feat["discretize"].end(); ++it)
					merged[it.key()] = it.value();
			}
			feat["params"] = merged;
		}
		feat.erase("discretize");
	}
	const bool hasPipeline = plan.contains("pipeline") && plan["pipeline"].is_array() && !plan["pipeline"].empty();
	if (!hasPipeline)
	{
		const std::string tmpl = plan.value("suggestedPipelineTemplate", std::string("weld_default"));
		const auto ops = RobotInstruction::buildRecipePreset(recipeKindFromTemplate(tmpl));
		plan["pipeline"] = RobotInstruction::trajectoryPipelineToJson(ops);
	}
	return true;
}
