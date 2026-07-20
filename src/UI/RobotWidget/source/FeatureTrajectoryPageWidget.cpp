/// @file FeatureTrajectoryPageWidget.cpp
/// @brief FeatureTrajectoryPageWidget 实现

#include "FeatureTrajectoryPageWidget.h"

#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "FeatureDiscretizerParamPanel.h"
#include "FeaturePickTransform.h"
#include "FeatureTableModel.h"
#include "GeometryRef.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "PickTypes.h"
#include "RawTrajectory.h"
#include "RecipeBlueprint.h"
#include "RobotOsgUiTypes.h"
#include "RobotSimulationController.h"
#include "RobotSimulationDockWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "TrajectoryEditSession.h"
#include "UiIconDecorators.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHash>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>

#include <ShapeHandle.h>
#include <json.hpp>

namespace
{
bool isTopLevelWorkpieceBackend(const BackendDataManager& mgr, const std::string& backendId)
{
	return mgr.parentsOf(backendId).empty();
}

bool isStepSourcePath(const QString& stepPath)
{
	const QString ext = QFileInfo(stepPath).suffix().toLower();
	return ext == QStringLiteral("step") || ext == QStringLiteral("stp");
}

struct WorkpieceComboCandidate
{
	QString backendId;
	QString label;
	QString dedupeKey;
	bool isBrepModel = false;
};

bool readGeometryFromCandidateJson(const nlohmann::json& candidate, geoalgo::FeatureGeometry& out)
{
	out = geoalgo::FeatureGeometry{};
	if (candidate.contains("geometry"))
	{
		const auto& g = candidate["geometry"];
		if (g.contains("edgeIndices") && g["edgeIndices"].is_array())
		{
			for (const auto& v : g["edgeIndices"])
			{
				out.edgeIndices.push_back(v.get<int>());
			}
		}
		if (g.contains("faceIndices") && g["faceIndices"].is_array())
		{
			for (const auto& v : g["faceIndices"])
			{
				out.faceIndices.push_back(v.get<int>());
			}
		}
	}
	else if (candidate.contains("refs"))
	{
		const auto& r = candidate["refs"];
		if (r.contains("edgeIndices") && r["edgeIndices"].is_array())
		{
			for (const auto& v : r["edgeIndices"])
			{
				out.edgeIndices.push_back(v.get<int>());
			}
		}
		if (r.contains("faceIndices") && r["faceIndices"].is_array())
		{
			for (const auto& v : r["faceIndices"])
			{
				out.faceIndices.push_back(v.get<int>());
			}
		}
	}
	return !out.edgeIndices.empty() || !out.faceIndices.empty();
}

} // namespace

FeatureTrajectoryPageWidget::FeatureTrajectoryPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->addWidget(new QLabel(QStringLiteral("工件 backend")));
	m_backendCombo = new QComboBox(this);
	layout->addWidget(m_backendCombo);

	m_featureModel = new FeatureTableModel(this);
	m_featureModel->setStrategyDisplayNameResolver([this](const std::string& strategyId)
												   { return strategyDisplayName(strategyId); });
	m_featureTable = new QTableView(this);
	m_featureTable->setModel(m_featureModel);
	m_featureTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_featureTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
	m_featureTable->horizontalHeader()->setStretchLastSection(true);
	m_featureTable->verticalHeader()->setVisible(false);
	m_featureTable->setContextMenuPolicy(Qt::CustomContextMenu);
	m_featureTable->setMinimumHeight(140);
	layout->addWidget(m_featureTable);

	auto* pickRow = new QHBoxLayout;
	m_pickEdgeBtn = new QPushButton(QStringLiteral("拾取线"), this);
	m_pickFaceBtn = new QPushButton(QStringLiteral("拾取面"), this);
	m_cancelPickBtn = new QPushButton(QStringLiteral("取消拾取"), this);
	pickRow->addWidget(m_pickEdgeBtn);
	pickRow->addWidget(m_pickFaceBtn);
	pickRow->addWidget(m_cancelPickBtn);
	layout->addLayout(pickRow);

	m_pickStatusLabel = new QLabel(this);
	layout->addWidget(m_pickStatusLabel);

	auto* strategyRow = new QHBoxLayout;
	strategyRow->addWidget(new QLabel(QStringLiteral("离散策略"), this));
	m_strategyCombo = new QComboBox(this);
	strategyRow->addWidget(m_strategyCombo, 1);
	layout->addLayout(strategyRow);

	m_paramPanel = new FeatureDiscretizerParamPanel(this);
	layout->addWidget(m_paramPanel);

	m_previewGroup = new QGroupBox(QStringLiteral("预览"), this);
	auto* previewLayout = new QVBoxLayout(m_previewGroup);
	auto* axisRow = new QHBoxLayout;
	m_showAxisXCheck = new QCheckBox(QStringLiteral("X"), m_previewGroup);
	m_showAxisYCheck = new QCheckBox(QStringLiteral("Y"), m_previewGroup);
	m_showAxisZCheck = new QCheckBox(QStringLiteral("Z"), m_previewGroup);
	m_showAxisXCheck->setChecked(true);
	m_showAxisYCheck->setChecked(true);
	m_showAxisZCheck->setChecked(true);
	axisRow->addWidget(m_showAxisXCheck);
	axisRow->addWidget(m_showAxisYCheck);
	axisRow->addWidget(m_showAxisZCheck);
	axisRow->addWidget(new QLabel(QStringLiteral("轴间隔"), m_previewGroup));
	m_axisIntervalSpin = new QSpinBox(m_previewGroup);
	m_axisIntervalSpin->setRange(0, 999999);
	m_axisIntervalSpin->setValue(0);
	m_axisIntervalSpin->setToolTip(QStringLiteral("0 = 自动 (约 n/20)，按点索引间隔显示坐标轴"));
	axisRow->addWidget(m_axisIntervalSpin);
	axisRow->addStretch();
	previewLayout->addLayout(axisRow);
	m_brepInfoLabel = new QLabel(m_previewGroup);
	m_brepInfoLabel->setWordWrap(true);
	m_brepInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
	m_brepInfoLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	m_brepInfoLabel->setMinimumHeight(64);
	previewLayout->addWidget(m_brepInfoLabel);
	layout->addWidget(m_previewGroup);

	connect(m_pickEdgeBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onPickEdge);
	connect(m_pickFaceBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onPickFace);
	connect(m_cancelPickBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onCancelPick);
	connect(m_strategyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&FeatureTrajectoryPageWidget::onStrategyComboChanged);

	connect(m_featureTable->selectionModel(), &QItemSelectionModel::currentRowChanged, this,
			[this](const QModelIndex& current, const QModelIndex& previous)
			{
				(void)previous;
				m_featureModel->setSelectedRow(current.isValid() ? current.row() : -1);
				onTableSelectionChanged();
			});
	connect(m_featureModel, &FeatureTableModel::selectionRowChanged, this,
			&FeatureTrajectoryPageWidget::onTableSelectionChanged);
	connect(m_featureTable, &QTableView::customContextMenuRequested, this,
			[this](const QPoint& pos)
			{
				QMenu menu(this);
				menu.addAction(m_chinese ? QStringLiteral("删除选中行") : QStringLiteral("Delete selected"), this,
							   &FeatureTrajectoryPageWidget::onDeleteSelectedRows);
				menu.addAction(m_chinese ? QStringLiteral("删除全部") : QStringLiteral("Delete all"), this,
							   &FeatureTrajectoryPageWidget::onDeleteAllRows);
				menu.exec(m_featureTable->viewport()->mapToGlobal(pos));
			});

	m_rediscretizeTimer = new QTimer(this);
	m_rediscretizeTimer->setSingleShot(true);
	m_rediscretizeTimer->setInterval(400);
	connect(m_rediscretizeTimer, &QTimer::timeout, this, &FeatureTrajectoryPageWidget::onParameterRediscretize);
	connect(m_paramPanel, &FeatureDiscretizerParamPanel::paramsChanged, this,
			[this]()
			{
				applyParamsToSelectedRow();
				scheduleParameterRediscretize();
			});
	connect(m_showAxisXCheck, &QCheckBox::toggled, this, [this]() { refreshPreviewFromSession(); });
	connect(m_showAxisYCheck, &QCheckBox::toggled, this, [this]() { refreshPreviewFromSession(); });
	connect(m_showAxisZCheck, &QCheckBox::toggled, this, [this]() { refreshPreviewFromSession(); });
	connect(m_axisIntervalSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
			[this]() { refreshPreviewFromSession(); });

	UiIconDecorators::apply(m_pickEdgeBtn, UiIconId::PickEdge);
	UiIconDecorators::apply(m_pickFaceBtn, UiIconId::PickFace);

	connect(m_backendCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this]()
			{
				clearCandidatePreview();
				m_cachedCatalogBackendId.clear();
				m_cachedCatalogJsonUtf8.clear();
				m_featureModel->clearAll();
				(void)autoEnumerateCatalogForCurrentWorkpiece(true, nullptr);
				emit workpieceComboChanged();
			});

	setUseChinese(m_chinese);
	ensureDiscretizerRuntimeLoaded();
	refreshStrategyCombo();
	updatePickUiState();
	refreshBrepInfoForSelection();
}

void FeatureTrajectoryPageWidget::ensureDiscretizerRuntimeLoaded() const
{
	if (m_runtimeLoaded)
	{
		return;
	}
	geometry_backend_ops::ensureFeatureDiscretizersRegistered();
	const std::string appDir = QCoreApplication::applicationDirPath().toStdString();
	(void)geometry_backend_ops::ensureFeatureDiscretizerConfigsLoaded(appDir, nullptr);
	m_runtimeLoaded = true;
}

void FeatureTrajectoryPageWidget::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
	m_featureModel->setUseChinese(chinese);
	m_paramPanel->setUseChinese(chinese);
	updateUiLabels();
}

void FeatureTrajectoryPageWidget::updateUiLabels()
{
	const bool zh = m_chinese;
	if (m_pickEdgeBtn)
	{
		m_pickEdgeBtn->setText(zh ? QStringLiteral("拾取线") : QStringLiteral("Pick edge"));
	}
	if (m_pickFaceBtn)
	{
		m_pickFaceBtn->setText(zh ? QStringLiteral("拾取面") : QStringLiteral("Pick face"));
	}
	if (m_cancelPickBtn)
	{
		m_cancelPickBtn->setText(zh ? QStringLiteral("取消拾取") : QStringLiteral("Cancel pick"));
	}
	if (m_previewGroup)
	{
		m_previewGroup->setTitle(zh ? QStringLiteral("预览") : QStringLiteral("Preview"));
	}
	if (m_showAxisXCheck)
	{
		m_showAxisXCheck->setText(zh ? QStringLiteral("X 轴") : QStringLiteral("X axis"));
	}
	if (m_showAxisYCheck)
	{
		m_showAxisYCheck->setText(zh ? QStringLiteral("Y 轴") : QStringLiteral("Y axis"));
	}
	if (m_showAxisZCheck)
	{
		m_showAxisZCheck->setText(zh ? QStringLiteral("Z 轴") : QStringLiteral("Z axis"));
	}
	updatePickUiState();
	refreshBrepInfoForSelection();
}

QString FeatureTrajectoryPageWidget::strategyDisplayName(const std::string& strategyId) const
{
	ensureDiscretizerRuntimeLoaded();
	if (m_chinese)
	{
		return QString::fromStdString(geometry_backend_ops::featureDiscretizerDisplayNameZh(strategyId));
	}
	return QString::fromStdString(strategyId);
}

void FeatureTrajectoryPageWidget::refreshStrategyCombo(const geoalgo::GeometryAffinity filterAffinity)
{
	ensureDiscretizerRuntimeLoaded();
	const QString previous = m_strategyCombo->currentData().toString();
	m_strategyCombo->blockSignals(true);
	m_strategyCombo->clear();
	const std::vector<std::string> ids = geometry_backend_ops::featureDiscretizerListStrategyIds();
	for (const std::string& id : ids)
	{
		const geoalgo::GeometryAffinity affinity = geometry_backend_ops::featureDiscretizerAffinity(id);
		if (filterAffinity != geoalgo::GeometryAffinity::Any && affinity != filterAffinity &&
			affinity != geoalgo::GeometryAffinity::Any)
		{
			continue;
		}
		m_strategyCombo->addItem(strategyDisplayName(id), QString::fromStdString(id));
	}
	int restoreIdx = previous.isEmpty() ? -1 : m_strategyCombo->findData(previous);
	if (restoreIdx < 0 && m_strategyCombo->count() > 0)
	{
		restoreIdx = 0;
	}
	if (restoreIdx >= 0)
	{
		m_strategyCombo->setCurrentIndex(restoreIdx);
	}
	m_strategyCombo->blockSignals(false);
}

void FeatureTrajectoryPageWidget::bindHost(IRobotMainWindowHost* host)
{
	if (m_host)
	{
		m_host->clearMeshPickCommittedHandler();
	}
	m_host = host;
	if (m_host)
	{
		m_host->setMeshPickCommittedHandler([this](const PickResult& pick, const PickKind kind)
											{ onMeshPickCommitted(pick, static_cast<int>(kind)); });
	}
	refreshBackendCombo();
}

void FeatureTrajectoryPageWidget::bindSession(TrajectoryEditSession* session)
{
	if (m_session)
	{
		disconnect(m_session, nullptr, this, nullptr);
	}
	m_session = session;
	if (m_session)
	{
		connect(m_session, &TrajectoryEditSession::pathPlanBound, this, &FeatureTrajectoryPageWidget::onPathPlanBound);
	}
}

void FeatureTrajectoryPageWidget::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
}

void FeatureTrajectoryPageWidget::setStepPathResolver(std::function<QString(const QString& backendId)> resolver)
{
	m_stepPathResolver = std::move(resolver);
	refreshBackendCombo();
}

void FeatureTrajectoryPageWidget::onTableSelectionChanged()
{
	loadParamsForSelectedRow();
}

void FeatureTrajectoryPageWidget::syncStrategyComboToEntry(const geoalgo::FeatureEntry& entry)
{
	if (!m_strategyCombo || entry.strategyId.empty())
	{
		return;
	}
	const bool faceOnly = !entry.geometry.faceIndices.empty() && entry.geometry.edgeIndices.empty();
	const bool lineOnly = !entry.geometry.edgeIndices.empty() && entry.geometry.faceIndices.empty();
	geoalgo::GeometryAffinity filter = geoalgo::GeometryAffinity::Any;
	if (faceOnly)
	{
		filter = geoalgo::GeometryAffinity::Face;
	}
	else if (lineOnly)
	{
		filter = geoalgo::GeometryAffinity::Line;
	}
	else
	{
		const geoalgo::GeometryAffinity affinity = geometry_backend_ops::featureDiscretizerAffinity(entry.strategyId);
		filter = affinity == geoalgo::GeometryAffinity::Any ? geoalgo::GeometryAffinity::Any : affinity;
	}
	const QSignalBlocker blocker(m_strategyCombo);
	refreshStrategyCombo(filter);
	const int idx = m_strategyCombo->findData(QString::fromStdString(entry.strategyId));
	if (idx >= 0)
	{
		m_strategyCombo->setCurrentIndex(idx);
	}
}

std::string FeatureTrajectoryPageWidget::defaultStrategyIdForGeometry(const geoalgo::GeometryAffinity required) const
{
	ensureDiscretizerRuntimeLoaded();
	const char* preferred = required == geoalgo::GeometryAffinity::Face ? "FaceBoundary" : "EdgeChain";
	const std::string preferredId(preferred);
	if (geometry_backend_ops::featureDiscretizerAffinity(preferredId) == required)
	{
		return preferredId;
	}
	for (const std::string& id : geometry_backend_ops::featureDiscretizerListStrategyIds())
	{
		if (geometry_backend_ops::featureDiscretizerAffinity(id) == required)
		{
			return id;
		}
	}
	return {};
}

void FeatureTrajectoryPageWidget::normalizeEntryStrategyForGeometry(geoalgo::FeatureEntry& entry) const
{
	const bool hasFace = !entry.geometry.faceIndices.empty();
	const bool hasEdge = !entry.geometry.edgeIndices.empty();
	if (hasFace == hasEdge)
	{
		return;
	}
	const geoalgo::GeometryAffinity required =
		hasFace ? geoalgo::GeometryAffinity::Face : geoalgo::GeometryAffinity::Line;
	const geoalgo::GeometryAffinity current = geometry_backend_ops::featureDiscretizerAffinity(entry.strategyId);
	const bool strategyKnown =
		std::find(geometry_backend_ops::featureDiscretizerListStrategyIds().begin(),
				  geometry_backend_ops::featureDiscretizerListStrategyIds().end(),
				  entry.strategyId) != geometry_backend_ops::featureDiscretizerListStrategyIds().end();
	const bool mismatch = !strategyKnown ||
						  (current == geoalgo::GeometryAffinity::Line && required == geoalgo::GeometryAffinity::Face) ||
						  (current == geoalgo::GeometryAffinity::Face && required == geoalgo::GeometryAffinity::Line);
	if (!mismatch)
	{
		return;
	}
	const std::string fixedId = defaultStrategyIdForGeometry(required);
	if (fixedId.empty())
	{
		return;
	}
	entry.strategyId = fixedId;
	entry.params = geometry_backend_ops::featureDiscretizerDefaultParams(entry.strategyId);
}

std::string FeatureTrajectoryPageWidget::resolveStrategyIdForPick(const bool pickFace) const
{
	ensureDiscretizerRuntimeLoaded();
	const geoalgo::GeometryAffinity required =
		pickFace ? geoalgo::GeometryAffinity::Face : geoalgo::GeometryAffinity::Line;
	const std::string fallback = defaultStrategyIdForGeometry(required);
	if (m_strategyCombo && m_strategyCombo->currentIndex() >= 0)
	{
		const std::string currentId = m_strategyCombo->currentData().toString().toStdString();
		if (!currentId.empty() && geometry_backend_ops::featureDiscretizerAffinity(currentId) == required)
		{
			return currentId;
		}
	}
	for (const std::string& id : geometry_backend_ops::featureDiscretizerListStrategyIds())
	{
		if (geometry_backend_ops::featureDiscretizerAffinity(id) == required)
		{
			return id;
		}
	}
	return fallback;
}

void FeatureTrajectoryPageWidget::loadParamsForSelectedRow()
{
	const int row = m_featureModel->selectedRow();
	if (row < 0)
	{
		m_paramPanel->clear();
		refreshBrepInfoForSelection();
		return;
	}
	const geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
	++m_strategyRowSyncDepth;
	syncStrategyComboToEntry(entry);
	m_paramPanel->rebuildForStrategy(entry.strategyId);
	m_paramPanel->setLoading(true);
	m_paramPanel->loadParams(entry.params);
	m_paramPanel->setLoading(false);
	--m_strategyRowSyncDepth;
	refreshBrepInfoForSelection();
}

void FeatureTrajectoryPageWidget::refreshBrepInfoForSelection()
{
	if (!m_brepInfoLabel)
	{
		return;
	}
	const bool zh = m_chinese;
	if (!m_featureModel || m_featureModel->selectedRow() < 0)
	{
		m_brepInfoLabel->setText(zh ? QStringLiteral("未选择特征") : QStringLiteral("No feature selected"));
		return;
	}
	const geoalgo::FeatureEntry entry = m_featureModel->entryAt(m_featureModel->selectedRow());
	const bool hasFace = !entry.geometry.faceIndices.empty();
	const bool hasEdge = !entry.geometry.edgeIndices.empty();
	QString typeText;
	if (hasFace && hasEdge)
	{
		typeText = zh ? QStringLiteral("面+边") : QStringLiteral("Face+Edge");
	}
	else if (hasFace)
	{
		typeText = zh ? QStringLiteral("面") : QStringLiteral("Face");
	}
	else if (hasEdge)
	{
		typeText = zh ? QStringLiteral("边") : QStringLiteral("Edge");
	}
	else
	{
		typeText = zh ? QStringLiteral("未知") : QStringLiteral("Unknown");
	}
	auto formatIndices = [zh](const std::vector<int>& indices) -> QString
	{
		if (indices.empty())
		{
			return zh ? QStringLiteral("(无)") : QStringLiteral("(none)");
		}
		QStringList parts;
		parts.reserve(static_cast<int>(indices.size()));
		for (const int idx : indices)
		{
			parts.append(QString::number(idx));
		}
		return parts.join(QStringLiteral(", "));
	};
	const QString featureId = entry.featureId.empty() ? (zh ? QStringLiteral("(无)") : QStringLiteral("(none)"))
													  : QString::fromStdString(entry.featureId);
	m_brepInfoLabel->setText(zh ? QStringLiteral("特征: %1\n类型: %2\nfaceIndices: %3\nedgeIndices: %4")
									  .arg(featureId, typeText, formatIndices(entry.geometry.faceIndices),
										   formatIndices(entry.geometry.edgeIndices))
								: QStringLiteral("Feature: %1\nType: %2\nfaceIndices: %3\nedgeIndices: %4")
									  .arg(featureId, typeText, formatIndices(entry.geometry.faceIndices),
										   formatIndices(entry.geometry.edgeIndices)));
}

void FeatureTrajectoryPageWidget::applyParamsToSelectedRow()
{
	const int row = m_featureModel->selectedRow();
	if (row < 0 || m_paramPanel->isRebuilding())
	{
		return;
	}
	geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
	nlohmann::json params = entry.params.is_object() ? entry.params : nlohmann::json::object();
	(void)m_paramPanel->applyParams(params);
	entry.params = params;
	(void)m_featureModel->updateEntry(row, entry);
}

void FeatureTrajectoryPageWidget::onStrategyComboChanged()
{
	if (m_strategyRowSyncDepth > 0)
	{
		return;
	}
	const int row = m_featureModel->selectedRow();
	if (row < 0)
	{
		return;
	}
	geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
	const QString strategyId = m_strategyCombo->currentData().toString();
	if (strategyId.isEmpty())
	{
		return;
	}
	const std::string strategyIdUtf8 = strategyId.toStdString();
	const geoalgo::GeometryAffinity affinity = geometry_backend_ops::featureDiscretizerAffinity(strategyIdUtf8);
	const bool hasFace = !entry.geometry.faceIndices.empty();
	const bool hasEdge = !entry.geometry.edgeIndices.empty();
	if (affinity == geoalgo::GeometryAffinity::Line && hasFace && !hasEdge)
	{
		setStatus(m_chinese ? QStringLiteral("线策略不能用于面特征，请选择面离散策略")
							: QStringLiteral("Line strategy cannot discretize face features"));
		syncStrategyComboToEntry(entry);
		return;
	}
	if (affinity == geoalgo::GeometryAffinity::Face && hasEdge && !hasFace)
	{
		setStatus(m_chinese ? QStringLiteral("面策略不能用于线特征，请选择线离散策略")
							: QStringLiteral("Face strategy cannot discretize edge features"));
		syncStrategyComboToEntry(entry);
		return;
	}
	entry.strategyId = strategyIdUtf8;
	entry.params = geometry_backend_ops::featureDiscretizerDefaultParams(entry.strategyId);
	(void)m_featureModel->updateEntry(row, entry);
	loadParamsForSelectedRow();
	scheduleParameterRediscretize();
}

void FeatureTrajectoryPageWidget::onDeleteSelectedRows()
{
	const QModelIndexList selected = m_featureTable->selectionModel()->selectedRows();
	QList<int> rows;
	rows.reserve(selected.size());
	for (const QModelIndex& idx : selected)
	{
		rows.push_back(idx.row());
	}
	m_featureModel->removeRows(rows);
	syncDiscretizationAfterFeatureTableChange();
}

void FeatureTrajectoryPageWidget::onDeleteAllRows()
{
	m_featureModel->clearAll();
	m_paramPanel->clear();
	syncDiscretizationAfterFeatureTableChange();
}

void FeatureTrajectoryPageWidget::setFeatureEditActive(const bool active)
{
	if (m_featureEditActive == active)
	{
		return;
	}
	m_featureEditActive = active;
	emit featureEditActiveChanged(active);
}

void FeatureTrajectoryPageWidget::resetAfterTrajectoryCommit()
{
	exitPickMode();
	clearCandidatePreview();
	setFeatureEditActive(false);
	m_suppressParamRediscretize = true;
	m_featureModel->clearAll();
	m_paramPanel->clear();
	m_suppressParamRediscretize = false;
	m_lastLoadedPathPlanId.clear();
	m_lastLoadedSourceJson.clear();
	if (m_simController)
	{
		m_simController->clearBoundPathPlanPreview();
	}
	refreshBrepInfoForSelection();
	setStatus(m_chinese ? QStringLiteral("轨迹已提交，请重新拾取或导入特征")
						: QStringLiteral("Trajectory committed; pick or import features again"));
}

bool FeatureTrajectoryPageWidget::beginEditBoundPathPlan(QString* err)
{
	if (!m_session || m_session->boundPathPlanId().empty())
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("请先选择路径规划") : QStringLiteral("Select a path plan first");
		}
		setStatus(err ? *err : QString());
		return false;
	}
	const std::string pathPlanId = m_session->boundPathPlanId();
	const std::string json = m_session->boundSourceFeatureJson();

	if (m_simController && m_simController->simulationDock())
	{
		if (TrajectoryEditPageWidget* edit = m_simController->simulationDock()->trajectoryEditPage())
		{
			edit->restoreBoundPathPlanForEdit();
		}
	}

	if (json.empty())
	{
		setFeatureEditActive(true);
		m_lastLoadedPathPlanId = pathPlanId;
		m_lastLoadedSourceJson.clear();
		refreshPreviewFromSession();
		setStatus(m_chinese ? QStringLiteral("已加载算子流程，请拾取或导入特征")
							: QStringLiteral("Pipeline loaded; pick or import features"));
		return true;
	}

	if (pathPlanId == m_lastLoadedPathPlanId && json == m_lastLoadedSourceJson && m_featureEditActive)
	{
		refreshPreviewFromSession();
		setStatus(m_chinese ? QStringLiteral("已在修改模式") : QStringLiteral("Already in edit mode"));
		return true;
	}
	QString loadErr;
	if (!loadFeatureListFromJson(json, &loadErr))
	{
		if (err)
		{
			*err = loadErr.isEmpty() ? (m_chinese ? QStringLiteral("特征 JSON 解析失败")
												  : QStringLiteral("Failed to parse feature JSON"))
									 : loadErr;
		}
		setStatus(err ? *err : loadErr);
		setFeatureEditActive(false);
		return false;
	}
	setFeatureEditActive(true);
	m_lastLoadedPathPlanId = pathPlanId;
	m_lastLoadedSourceJson = json;
	refreshPreviewFromSession();
	const int featureCount = static_cast<int>(m_featureModel->entries().size());
	const int pipelineCount = m_session ? static_cast<int>(m_session->boundPipelineOpCount()) : 0;
	setStatus(m_chinese ? QStringLiteral("已加载 %1 个特征与 %2 个算子；修改离散参数将自动更新预览")
							  .arg(featureCount)
							  .arg(pipelineCount)
						: QStringLiteral("Loaded %1 features and %2 ops; param edits refresh preview")
							  .arg(featureCount)
							  .arg(pipelineCount));
	return true;
}

void FeatureTrajectoryPageWidget::cancelEditBoundPathPlan()
{
	if (!m_featureEditActive)
	{
		setStatus(m_chinese ? QStringLiteral("当前未在修改模式") : QStringLiteral("Not in edit mode"));
		return;
	}
	exitPickMode();
	clearCandidatePreview();
	setFeatureEditActive(false);
	m_suppressParamRediscretize = true;
	m_featureModel->clearAll();
	m_paramPanel->clear();
	m_suppressParamRediscretize = false;
	// 下次「开始修改」强制从 PathPlan 重载特征 JSON
	m_lastLoadedSourceJson.clear();
	if (m_session)
	{
		m_session->abandonPreview();
		(void)m_session->reloadBoundPathPlanFromStore();
	}
	if (m_simController)
	{
		m_simController->clearBoundPathPlanPreview();
		if (m_simController->simulationDock())
		{
			if (TrajectoryEditPageWidget* edit = m_simController->simulationDock()->trajectoryEditPage())
			{
				edit->syncBoundPathPlanFromSession();
			}
		}
	}
	refreshBrepInfoForSelection();
	setStatus(m_chinese ? QStringLiteral("已取消修改；特征表已清空，预览已关闭。已落盘内容保留，可再次「开始修改」")
						: QStringLiteral("Edit cancelled; table cleared and preview closed. Persisted path plan kept"));
}

void FeatureTrajectoryPageWidget::scheduleParameterRediscretize()
{
	if (!m_featureEditActive || m_suppressParamRediscretize || m_featureModel->entries().empty() ||
		!m_rediscretizeTimer)
	{
		return;
	}
	m_rediscretizeTimer->start();
}

void FeatureTrajectoryPageWidget::onParameterRediscretize()
{
	if (m_featureModel->entries().empty())
	{
		return;
	}
	m_suppressParamRediscretize = true;
	(void)discretizeFromTable(true);
	m_suppressParamRediscretize = false;
}

void FeatureTrajectoryPageWidget::syncDiscretizationAfterFeatureTableChange()
{
	refreshBrepInfoForSelection();
	if (m_featureModel->entries().empty())
	{
		if (m_session)
		{
			m_session->clearRawTrajectory();
		}
		if (m_simController)
		{
			m_simController->clearBoundPathPlanPreview();
		}
		setStatus(m_chinese ? QStringLiteral("特征已清空，已清除离散预览")
							: QStringLiteral("Features cleared; discretization preview removed"));
		return;
	}
	setFeatureEditActive(true);
	(void)discretizeFromTable(true);
}

void FeatureTrajectoryPageWidget::refreshPreviewFromSession()
{
	if (!m_simController)
	{
		return;
	}
	if (!m_session || !m_session->hasRawTrajectory())
	{
		m_simController->refreshBoundPathPlanPreview();
		return;
	}
	const RobotInstruction::RawTrajectory* traj = m_session->rawTrajectory();
	if (traj)
	{
		showTrajectoryPreview(*traj);
	}
}

bool FeatureTrajectoryPageWidget::selectBackendComboById(const QString& backendId)
{
	if (!m_backendCombo || backendId.isEmpty())
	{
		return false;
	}
	for (int i = 0; i < m_backendCombo->count(); ++i)
	{
		if (m_backendCombo->itemData(i).toString() == backendId)
		{
			if (m_backendCombo->currentIndex() != i)
			{
				m_backendCombo->setCurrentIndex(i);
			}
			return true;
		}
	}
	return false;
}

bool FeatureTrajectoryPageWidget::applyFeatureListDocument(const geoalgo::FeatureListDocument& doc,
														   const bool restoreWorkpiece)
{
	if (doc.features.empty())
	{
		return false;
	}

	geoalgo::FeatureListDocument normalized = doc;
	if (restoreWorkpiece && !normalized.workpiece.backendIdUtf8.empty())
	{
		const QString backendId = QString::fromStdString(normalized.workpiece.backendIdUtf8);
		if (!selectBackendComboById(backendId))
		{
			setStatus(m_chinese
						  ? QStringLiteral("未找到工件 %1，特征表已加载但请手动选择工件").arg(backendId)
						  : QStringLiteral("Workpiece %1 not found; feature table loaded, select workpiece manually")
								.arg(backendId));
		}
	}
	else
	{
		QString backendId;
		QString stepPath;
		if (currentWorkpiece(backendId, stepPath))
		{
			if (normalized.workpiece.backendIdUtf8.empty())
			{
				normalized.workpiece.backendIdUtf8 = backendId.toStdString();
			}
			if (normalized.workpiece.stepPathUtf8.empty() && !stepPath.isEmpty())
			{
				normalized.workpiece.stepPathUtf8 = stepPath.toStdString();
			}
		}
	}

	m_suppressParamRediscretize = true;
	++m_strategyRowSyncDepth;
	for (geoalgo::FeatureEntry& feature : normalized.features)
	{
		normalizeEntryStrategyForGeometry(feature);
	}
	m_featureModel->setEntries(normalized.features);
	if (!m_featureModel->entries().empty())
	{
		m_featureTable->selectRow(0);
		loadParamsForSelectedRow();
	}
	--m_strategyRowSyncDepth;
	m_suppressParamRediscretize = false;
	return true;
}

bool FeatureTrajectoryPageWidget::loadFeatureListFromJson(const std::string& jsonUtf8, QString* err)
{
	geoalgo::FeatureListDocument doc{};
	std::string parseErr;
	if (!geometry_backend_ops::featureListFromJson(jsonUtf8, doc, &parseErr))
	{
		if (err)
		{
			*err = QString::fromStdString(parseErr);
		}
		return false;
	}
	if (doc.features.empty())
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("特征 JSON 无 features") : QStringLiteral("Feature JSON has no features");
		}
		return false;
	}
	if (!applyFeatureListDocument(doc, true))
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("无法应用特征表") : QStringLiteral("Failed to apply feature table");
		}
		return false;
	}
	return true;
}

void FeatureTrajectoryPageWidget::onPathPlanBound(const std::string& pathPlanId)
{
	if (!m_session || pathPlanId.empty())
	{
		return;
	}
	const std::string json = m_session->boundSourceFeatureJson();
	const bool planChanged = pathPlanId != m_lastLoadedPathPlanId;
	if (planChanged)
	{
		exitPickMode();
		clearCandidatePreview();
		setFeatureEditActive(false);
		m_suppressParamRediscretize = true;
		m_featureModel->clearAll();
		m_paramPanel->clear();
		m_suppressParamRediscretize = false;
		m_lastLoadedPathPlanId = pathPlanId;
		m_lastLoadedSourceJson.clear();
		refreshBrepInfoForSelection();
	}
	if (json.empty())
	{
		setStatus(m_chinese ? QStringLiteral("新建路径规划，请拾取或导入特征")
							: QStringLiteral("New path plan; pick or import features"));
	}
	else if (planChanged || !m_featureEditActive)
	{
		setStatus(m_chinese ? QStringLiteral("已绑定路径规划，点击「开始修改」加载特征、离散参数与算子流程")
							: QStringLiteral("Path plan bound; click Edit to load features, params and pipeline"));
	}
	if (m_simController && !m_featureEditActive)
	{
		m_simController->clearBoundPathPlanPreview();
	}
}

void FeatureTrajectoryPageWidget::updatePickUiState()
{
	const bool hasWorkpiece = m_backendCombo && m_backendCombo->currentIndex() >= 0;
	if (m_pickEdgeBtn)
	{
		m_pickEdgeBtn->setEnabled(hasWorkpiece && m_pickSession == PickSessionKind::None);
	}
	if (m_pickFaceBtn)
	{
		m_pickFaceBtn->setEnabled(hasWorkpiece && m_pickSession == PickSessionKind::None);
	}
	if (m_cancelPickBtn)
	{
		m_cancelPickBtn->setEnabled(m_pickSession != PickSessionKind::None);
	}
	if (!m_pickStatusLabel)
	{
		return;
	}
	if (m_pickSession == PickSessionKind::Edge)
	{
		m_pickStatusLabel->setText(m_chinese ? QStringLiteral("请在视口中点击一条边…")
											 : QStringLiteral("Click an edge in the 3D view…"));
	}
	else if (m_pickSession == PickSessionKind::Face)
	{
		m_pickStatusLabel->setText(m_chinese ? QStringLiteral("请在视口中点击一个面…")
											 : QStringLiteral("Click a face in the 3D view…"));
	}
	else
	{
		m_pickStatusLabel->setText(m_chinese ? QStringLiteral("3D 拾取未激活") : QStringLiteral("3D pick inactive"));
	}
}

void FeatureTrajectoryPageWidget::exitPickMode()
{
	m_pickSession = PickSessionKind::None;
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->setMeshLinePickMode(false);
		osg->setMeshFacePickMode(false);
	}
	updatePickUiState();
}

void FeatureTrajectoryPageWidget::onPickEdge()
{
	if (!m_host || m_backendCombo->currentIndex() < 0)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const QString backendId = m_backendCombo->currentData().toString();
	osg->setObjectSelectionMode(false);
	osg->setMeshFacePickMode(false);
	osg->setMeshLinePickMode(true);
	osg->setMeshPickScopeBackendId(backendId.toStdString());
	m_pickSession = PickSessionKind::Edge;
	m_lastPickAffinity = geoalgo::GeometryAffinity::Line;
	refreshStrategyCombo(geoalgo::GeometryAffinity::Line);
	updatePickUiState();
	setStatus(m_chinese ? QStringLiteral("边拾取模式：在视口左键点击确认")
						: QStringLiteral("Edge pick: left-click in viewport"));
}

void FeatureTrajectoryPageWidget::onPickFace()
{
	if (!m_host || m_backendCombo->currentIndex() < 0)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const QString backendId = m_backendCombo->currentData().toString();
	osg->setObjectSelectionMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
	osg->setMeshPickScopeBackendId(backendId.toStdString());
	m_pickSession = PickSessionKind::Face;
	m_lastPickAffinity = geoalgo::GeometryAffinity::Face;
	refreshStrategyCombo(geoalgo::GeometryAffinity::Face);
	updatePickUiState();
	setStatus(m_chinese ? QStringLiteral("面拾取模式：在视口左键点击确认")
						: QStringLiteral("Face pick: left-click in viewport"));
}

void FeatureTrajectoryPageWidget::onCancelPick()
{
	exitPickMode();
	refreshStrategyCombo();
	setStatus(m_chinese ? QStringLiteral("已取消 3D 拾取") : QStringLiteral("3D pick cancelled"));
}

bool FeatureTrajectoryPageWidget::buildFeatureEntryFromPick(const bool pickFace, const geoalgo::Point3d& modelA,
															const geoalgo::Point3d& modelB, const int knownFaceIndex,
															const int knownEdgeIndex, geoalgo::FeatureEntry& out,
															QString* err) const
{
	if (m_backendCombo->currentIndex() < 0)
	{
		if (err)
		{
			*err = QStringLiteral("未选择工件");
		}
		return false;
	}
	const QString backendId = m_backendCombo->currentData().toString();
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	if (!resolveWorkpieceShapeForBackend(backendId, shape, wp, err))
	{
		return false;
	}

	const std::string strategyId = resolveStrategyIdForPick(pickFace);
	if (strategyId.empty())
	{
		if (err)
		{
			*err =
				m_chinese ? QStringLiteral("未找到匹配的离散策略") : QStringLiteral("No matching discretize strategy");
		}
		return false;
	}

	out = geoalgo::FeatureEntry{};
	out.strategyId = strategyId;
	out.params = geometry_backend_ops::featureDiscretizerDefaultParams(out.strategyId);

	std::string stdErr;
	if (!geometry_backend_ops::buildFeatureEntryFromModelPick(wp, shape, out.strategyId, pickFace, modelA, modelB, out,
															  &stdErr, knownFaceIndex, knownEdgeIndex))
	{
		if (err)
		{
			*err = QString::fromStdString(stdErr);
		}
		return false;
	}
	normalizeEntryStrategyForGeometry(out);
	return true;
}

void FeatureTrajectoryPageWidget::onMeshPickCommitted(const PickResult& pick, const int pickKindInt)
{
	if (m_pickSession == PickSessionKind::None || !m_host || !pick.hit)
	{
		return;
	}
	const PickKind kind = static_cast<PickKind>(pickKindInt);
	if ((m_pickSession == PickSessionKind::Edge && kind != PickKind::MeshEdge) ||
		(m_pickSession == PickSessionKind::Face && kind != PickKind::MeshFace))
	{
		return;
	}
	if (m_backendCombo->currentIndex() < 0)
	{
		exitPickMode();
		return;
	}
	const QString expectedBackendId = m_backendCombo->currentData().toString();
	if (QString::fromStdString(pick.backendId) != expectedBackendId)
	{
		QMessageBox::warning(this, QStringLiteral("Pick"),
							 m_chinese ? QStringLiteral("拾取 backend 与当前工件不一致")
									   : QStringLiteral("Picked backend mismatch"));
		exitPickMode();
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	geoalgo::Point3d modelA{};
	geoalgo::Point3d modelB{};
	std::string xformErr;
	if (!feature_pick_transform::worldPointToStepModelMm(osg, pick.backendId, pick.worldPoint, modelA, &xformErr))
	{
		QMessageBox::warning(this, QStringLiteral("Pick"), QString::fromStdString(xformErr));
		exitPickMode();
		return;
	}
	if (kind == PickKind::MeshEdge)
	{
		if (!feature_pick_transform::worldPointToStepModelMm(osg, pick.backendId, pick.meshEdgeA, modelA, &xformErr) ||
			!feature_pick_transform::worldPointToStepModelMm(osg, pick.backendId, pick.meshEdgeB, modelB, &xformErr))
		{
			QMessageBox::warning(this, QStringLiteral("Pick"), QString::fromStdString(xformErr));
			exitPickMode();
			return;
		}
	}
	else
	{
		modelB = modelA;
	}

	geoalgo::FeatureEntry entry;
	QString pickErr;
	const int knownFaceIndex = pick.brepNativePick && kind == PickKind::MeshFace ? pick.brepFaceIndex : -1;
	const int knownEdgeIndex = pick.brepNativePick && kind == PickKind::MeshEdge ? pick.brepEdgeIndex : -1;
	if (!buildFeatureEntryFromPick(kind == PickKind::MeshFace, modelA, modelB, knownFaceIndex, knownEdgeIndex, entry,
								   &pickErr))
	{
		QMessageBox::warning(this, QStringLiteral("Pick"), pickErr);
		exitPickMode();
		return;
	}

	++m_strategyRowSyncDepth;
	m_suppressParamRediscretize = true;
	m_featureModel->appendEntry(entry);
	exitPickMode();
	syncStrategyComboToEntry(entry);
	loadParamsForSelectedRow();
	m_suppressParamRediscretize = false;
	setFeatureEditActive(true);
	setStatus(m_chinese
				  ? QStringLiteral("已添加特征 %1，正在离散…").arg(QString::fromStdString(entry.featureId))
				  : QStringLiteral("Added feature %1, discretizing…").arg(QString::fromStdString(entry.featureId)));
	const bool ok = discretizeFromTable(true);
	--m_strategyRowSyncDepth;
	if (ok && m_session && m_session->hasRawTrajectory())
	{
		const int n = static_cast<int>(m_session->rawTrajectory()->points.size());
		QString msg =
			m_chinese
				? QStringLiteral("已离散特征 %1 → 共 %2 点").arg(QString::fromStdString(entry.featureId)).arg(n)
				: QStringLiteral("Discretized %1 → %2 points").arg(QString::fromStdString(entry.featureId)).arg(n);
		if (!m_session->boundPathPlanId().empty())
		{
			msg += m_chinese ? QStringLiteral("；已写入当前路径规划") : QStringLiteral("; saved to path plan");
		}
		setStatus(msg);
	}
	else if (!ok)
	{
		setFeatureEditActive(false);
	}
}

void FeatureTrajectoryPageWidget::refreshBackendCombo()
{
	m_backendCombo->clear();
	if (!m_host)
	{
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		return;
	}
	BackendDataManager& mgr = doc->backend();
	const auto all = mgr.listData();
	QVector<WorkpieceComboCandidate> candidates;
	candidates.reserve(static_cast<int>(all.size()));
	for (const auto& data : all)
	{
		if (!data)
		{
			continue;
		}
		if (!isTopLevelWorkpieceBackend(mgr, data->id()))
		{
			continue;
		}
		const QString cn = QString::fromStdString(data->className());
		if (cn != QStringLiteral("Model") && cn != QStringLiteral("BrepModel"))
		{
			continue;
		}
		const QString backendId = QString::fromStdString(data->id());
		if (backendId.startsWith(QStringLiteral("RobotURDF_")))
		{
			continue;
		}
		QString stepPath;
		if (m_stepPathResolver)
		{
			stepPath = m_stepPathResolver(backendId);
		}
		const bool isBrepModel = cn == QStringLiteral("BrepModel");
		if (isBrepModel)
		{
			if (!std::dynamic_pointer_cast<BrepBackendData>(data) || !data->hasGeometry())
			{
				continue;
			}
		}
		else if (stepPath.isEmpty() || !isStepSourcePath(stepPath))
		{
			continue;
		}
		const QString fileName = QFileInfo(stepPath).fileName();
		const QString label = fileName.isEmpty() ? backendId : QStringLiteral("%1 (%2)").arg(backendId, fileName);
		WorkpieceComboCandidate candidate;
		candidate.backendId = backendId;
		candidate.label = label;
		candidate.dedupeKey = stepPath.isEmpty() ? backendId : stepPath.toLower();
		candidate.isBrepModel = isBrepModel;
		candidates.append(candidate);
	}

	QHash<QString, int> bestIndexByKey;
	for (int i = 0; i < candidates.size(); ++i)
	{
		const WorkpieceComboCandidate& candidate = candidates[i];
		if (!bestIndexByKey.contains(candidate.dedupeKey))
		{
			bestIndexByKey.insert(candidate.dedupeKey, i);
			continue;
		}
		const int prev = bestIndexByKey.value(candidate.dedupeKey);
		if (!candidates[prev].isBrepModel && candidate.isBrepModel)
		{
			bestIndexByKey[candidate.dedupeKey] = i;
		}
	}

	QSet<int> keepIndices;
	keepIndices.reserve(bestIndexByKey.size());
	for (int idx : bestIndexByKey)
	{
		keepIndices.insert(idx);
	}

	int stepCount = 0;
	for (int i = 0; i < candidates.size(); ++i)
	{
		if (!keepIndices.contains(i))
		{
			continue;
		}
		m_backendCombo->addItem(candidates[i].label, candidates[i].backendId);
		++stepCount;
	}
	if (stepCount == 0)
	{
		setStatus(m_chinese ? QStringLiteral("请先导入 STEP 工件") : QStringLiteral("Import a STEP workpiece first"));
	}
	else if (m_backendCombo->currentIndex() >= 0)
	{
		QTimer::singleShot(0, this, [this]() { (void)autoEnumerateCatalogForCurrentWorkpiece(true, nullptr); });
	}
	updatePickUiState();
}

bool FeatureTrajectoryPageWidget::resolveWorkpieceShapeForBackend(const QString& backendId,
																  geoalgo::ShapeHandle& outShape,
																  geoalgo::WorkpieceRef& outRef, QString* err) const
{
	outShape = geoalgo::ShapeHandle{};
	outRef = geoalgo::WorkpieceRef{};
	if (!m_host)
	{
		if (err)
		{
			*err = QStringLiteral("Host 未绑定");
		}
		return false;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		if (err)
		{
			*err = QStringLiteral("文档未就绪");
		}
		return false;
	}
	const QString stepPath = m_stepPathResolver ? m_stepPathResolver(backendId) : QString();
	std::string stdErr;
	const auto src = geometry_backend_ops::resolveWorkpieceShape(backendId.toStdString(), doc->backend(),
																 stepPath.toStdString(), outShape, outRef, &stdErr);
	if (src == geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		if (err)
		{
			*err = stdErr.empty() ? QStringLiteral("无法解析工件 B-rep") : QString::fromStdString(stdErr);
		}
		return false;
	}
	return true;
}

bool FeatureTrajectoryPageWidget::enumerateCatalogForBackend(const QString& backendId, geoalgo::FeatureCatalog& catalog,
															 QString* err) const
{
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	if (!resolveWorkpieceShapeForBackend(backendId, shape, wp, err))
	{
		return false;
	}
	std::string stdErr;
	if (!geometry_backend_ops::enumerateFeatureCatalog(wp, shape, catalog, &stdErr))
	{
		if (err)
		{
			*err = QString::fromStdString(stdErr);
		}
		return false;
	}
	return true;
}

bool FeatureTrajectoryPageWidget::autoEnumerateCatalogForCurrentWorkpiece(const bool quiet, QString* err)
{
	QString backendId;
	QString stepPath;
	if (!currentWorkpiece(backendId, stepPath))
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("请先在轨迹生成页选择 STEP 工件")
							 : QStringLiteral("Select a STEP workpiece on Trajectory Generation tab");
		}
		return false;
	}
	if (backendId == m_cachedCatalogBackendId && !m_cachedCatalogJsonUtf8.empty())
	{
		return true;
	}

	geoalgo::FeatureCatalog catalog;
	if (!enumerateCatalogForBackend(backendId, catalog, err))
	{
		return false;
	}

	m_cachedCatalogBackendId = backendId;
	m_cachedCatalogJsonUtf8 = geometry_backend_ops::featureCatalogToJson(catalog);

	if (!quiet)
	{
		setStatus(m_chinese
					  ? QStringLiteral("特征目录已加载（%1 个候选）").arg(static_cast<int>(catalog.candidates.size()))
					  : QStringLiteral("Feature catalog loaded (%1 candidates)")
							.arg(static_cast<int>(catalog.candidates.size())));
	}
	return true;
}

bool FeatureTrajectoryPageWidget::ensureFeatureCatalogEnumerated(QString* err)
{
	return autoEnumerateCatalogForCurrentWorkpiece(true, err);
}

void FeatureTrajectoryPageWidget::setStatus(const QString& text)
{
	if (m_host)
	{
		m_host->appendRunInfo(text);
	}
}

std::string FeatureTrajectoryPageWidget::resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const
{
	const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(traj);
	if (!backendId.empty())
	{
		return backendId;
	}
	return {};
}

RobotOsgUi::RawTrajectoryPreviewOptions FeatureTrajectoryPageWidget::currentPreviewOptions() const
{
	RobotOsgUi::RawTrajectoryPreviewOptions opt;
	opt.showAxisX = m_showAxisXCheck && m_showAxisXCheck->isChecked();
	opt.showAxisY = m_showAxisYCheck && m_showAxisYCheck->isChecked();
	opt.showAxisZ = m_showAxisZCheck && m_showAxisZCheck->isChecked();
	opt.showAxes = opt.showAxisX || opt.showAxisY || opt.showAxisZ;
	opt.axisInterval = m_axisIntervalSpin ? m_axisIntervalSpin->value() : 0;
	return opt;
}

RobotOsgUi::RawTrajectoryPreviewOptions FeatureTrajectoryPageWidget::previewOptions() const
{
	return currentPreviewOptions();
}

void FeatureTrajectoryPageWidget::showTrajectoryPreview(const RobotInstruction::RawTrajectory& traj)
{
	if (m_simController)
	{
		m_simController->refreshBoundPathPlanPreview(&traj);
	}
}

bool FeatureTrajectoryPageWidget::buildFeatureListDocument(geoalgo::FeatureListDocument& out, QString* err) const
{
	if (m_backendCombo->currentIndex() < 0)
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("请选择工件") : QStringLiteral("Select a workpiece");
		}
		return false;
	}
	if (m_featureModel->entries().empty())
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("特征表为空，请 3D 拾取或从 AI 导入")
							 : QStringLiteral("Feature table is empty; pick in 3D or import from AI");
		}
		return false;
	}
	const QString backendId = m_backendCombo->currentData().toString();
	QString stepPath;
	if (m_stepPathResolver)
	{
		stepPath = m_stepPathResolver(backendId);
	}
	out = geoalgo::FeatureListDocument{};
	out.schemaVersion = 2;
	out.workpiece.backendIdUtf8 = backendId.toStdString();
	out.workpiece.stepPathUtf8 = stepPath.toStdString();
	out.features = m_featureModel->entries();
	for (geoalgo::FeatureEntry& feature : out.features)
	{
		normalizeEntryStrategyForGeometry(feature);
	}
	return true;
}

bool FeatureTrajectoryPageWidget::discretizeFromTable(const bool quiet)
{
	for (int i = 0; i < static_cast<int>(m_featureModel->entries().size()); ++i)
	{
		geoalgo::FeatureEntry entry = m_featureModel->entryAt(i);
		const std::string before = entry.strategyId;
		normalizeEntryStrategyForGeometry(entry);
		if (entry.strategyId != before)
		{
			(void)m_featureModel->updateEntry(i, entry);
		}
	}

	geoalgo::FeatureListDocument doc;
	QString prepErr;
	if (!buildFeatureListDocument(doc, &prepErr))
	{
		if (!quiet)
		{
			QMessageBox::warning(this, QStringLiteral("离散"), prepErr);
		}
		else if (!prepErr.isEmpty())
		{
			setStatus(prepErr);
		}
		return false;
	}

	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	QString shapeErr;
	const QString backendId = m_backendCombo ? m_backendCombo->currentData().toString() : QString();
	if (!resolveWorkpieceShapeForBackend(backendId, shape, wp, &shapeErr))
	{
		if (!quiet)
		{
			QMessageBox::warning(this, QStringLiteral("离散"), shapeErr);
		}
		else if (!shapeErr.isEmpty())
		{
			setStatus(shapeErr);
		}
		return false;
	}
	doc.workpiece = wp;

	geoalgo::RawPath path;
	std::string err;
	if (!geometry_backend_ops::discretizeFeatureList(doc, shape, path, &err))
	{
		for (int i = 0; i < static_cast<int>(m_featureModel->entries().size()); ++i)
		{
			m_featureModel->setRowStatus(i, m_chinese ? QStringLiteral("离散失败") : QStringLiteral("failed"));
		}
		const QString errText = err.empty()
									? (m_chinese ? QStringLiteral("离散失败") : QStringLiteral("Discretization failed"))
									: QString::fromStdString(err);
		if (!quiet)
		{
			QMessageBox::warning(this, QStringLiteral("离散"), errText);
		}
		else
		{
			setStatus(errText);
		}
		return false;
	}

	for (int i = 0; i < static_cast<int>(m_featureModel->entries().size()); ++i)
	{
		m_featureModel->setRowStatus(i, m_chinese ? QStringLiteral("就绪") : QStringLiteral("ok"));
	}

	RobotInstruction::RawTrajectory traj;
	if (!RobotInstruction::importRawPathToTrajectory(path, RobotInstruction::FrameStrategy::SurfaceNormalZ, traj, &err))
	{
		const QString errText =
			err.empty() ? (m_chinese ? QStringLiteral("轨迹导入失败") : QStringLiteral("Failed to import trajectory"))
						: QString::fromStdString(err);
		if (!quiet)
		{
			QMessageBox::warning(this, QStringLiteral("导入"), errText);
		}
		else
		{
			setStatus(errText);
		}
		return false;
	}
	traj.sourceFeatureJson = geometry_backend_ops::featureListToJson(doc);
	if (m_session)
	{
		m_session->setRawTrajectory(traj);
	}
	setFeatureEditActive(true);
	m_lastLoadedPathPlanId = m_session ? m_session->boundPathPlanId() : std::string{};
	m_lastLoadedSourceJson = traj.sourceFeatureJson;
	showTrajectoryPreview(traj);

	const int n = static_cast<int>(traj.points.size());
	QString msg = m_chinese ? QStringLiteral("已离散 %1 个特征 → 共 %2 点；请在「轨迹编辑」应用配方")
								  .arg(static_cast<int>(doc.features.size()))
								  .arg(n)
							: QStringLiteral("Discretized %1 features → %2 points; apply recipe on Trajectory Edit tab")
								  .arg(static_cast<int>(doc.features.size()))
								  .arg(n);
	if (m_session)
	{
		if (!m_session->boundPathPlanId().empty())
		{
			msg += m_chinese ? QStringLiteral("；已写入当前选中的路径规划")
							 : QStringLiteral("; saved to selected path plan");
		}
		else
		{
			msg += m_chinese ? QStringLiteral("；已新建路径规划") : QStringLiteral("; new path plan created");
		}
	}
	if (!quiet)
	{
		setStatus(msg);
	}
	return true;
}

bool FeatureTrajectoryPageWidget::currentWorkpiece(QString& backendId, QString& stepPath) const
{
	backendId.clear();
	stepPath.clear();
	if (!m_backendCombo || m_backendCombo->currentIndex() < 0)
	{
		return false;
	}
	backendId = m_backendCombo->currentData().toString();
	if (backendId.isEmpty())
	{
		return false;
	}
	if (m_stepPathResolver)
	{
		stepPath = m_stepPathResolver(backendId);
	}
	if (m_host && m_host->document())
	{
		if (const auto data = m_host->document()->backend().getData(backendId.toStdString()))
		{
			if (data->className() == "BrepModel" && data->hasGeometry())
			{
				return true;
			}
		}
	}
	return !stepPath.isEmpty();
}

bool FeatureTrajectoryPageWidget::buildPreviewOverlayJson(const QByteArray& catalogSliceUtf8,
														  QByteArray& outPreviewJson, QString* err) const
{
	QString backendId;
	QString stepPath;
	if (!currentWorkpiece(backendId, stepPath))
	{
		if (err)
		{
			*err = QStringLiteral("请先在轨迹生成页选择 STEP 工件");
		}
		return false;
	}
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!osg)
	{
		if (err)
		{
			*err = QStringLiteral("3D 视口未就绪");
		}
		return false;
	}

	try
	{
		const nlohmann::json slice = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		nlohmann::json preview;
		preview["backendIdUtf8"] = backendId.toStdString();
		nlohmann::json items = nlohmann::json::array();
		geoalgo::WorkpieceRef wp;
		wp.backendIdUtf8 = backendId.toStdString();
		wp.stepPathUtf8 = stepPath.toStdString();

		if (!slice.contains("candidates") || !slice["candidates"].is_array())
		{
			if (err)
			{
				*err = QStringLiteral("catalog 切片无 candidates");
			}
			return false;
		}

		for (const auto& c : slice["candidates"])
		{
			geoalgo::FeatureGeometry geometry;
			if (!readGeometryFromCandidateJson(c, geometry))
			{
				continue;
			}
			geoalgo::FeatureAnchor anchor;
			std::string anchorErr;
			if (!geometry_backend_ops::computeFeatureAnchor(wp, geometry, anchor, &anchorErr))
			{
				continue;
			}

			auto toWorld = [&](const double modelMm[3], nlohmann::json& outPt)
			{
				const geoalgo::Point3d modelPt{modelMm[0], modelMm[1], modelMm[2]};
				osg::Vec3f worldPt;
				std::string tfErr;
				if (!feature_pick_transform::stepModelPointToWorldMm(osg, backendId.toStdString(), modelPt, worldPt,
																	 &tfErr))
				{
					worldPt.set(static_cast<float>(modelPt.x), static_cast<float>(modelPt.y),
								static_cast<float>(modelPt.z));
				}
				outPt = nlohmann::json::array({static_cast<double>(worldPt.x()), static_cast<double>(worldPt.y()),
											   static_cast<double>(worldPt.z())});
			};

			nlohmann::json item;
			item["displayIndex"] = c.value("displayIndex", 0);
			toWorld(anchor.anchorXyzMm, item["anchorWorldMm"]);
			toWorld(anchor.labelOffsetXyzMm, item["labelWorldMm"]);
			item["hasEdgeSegment"] = anchor.hasEdgeSegment;
			if (anchor.hasEdgeSegment)
			{
				toWorld(anchor.edgeEndAXyzMm, item["edgeAWorldMm"]);
				toWorld(anchor.edgeEndBXyzMm, item["edgeBWorldMm"]);
			}
			items.push_back(item);
		}
		preview["items"] = items;
		outPreviewJson = QByteArray::fromStdString(preview.dump());
		return !items.empty();
	}
	catch (...)
	{
		if (err)
		{
			*err = QStringLiteral("catalog 切片 JSON 无效");
		}
		return false;
	}
}

bool FeatureTrajectoryPageWidget::buildAndShowCandidatePreview(const QByteArray& catalogSliceUtf8)
{
	QByteArray previewJson;
	QString err;
	if (!buildPreviewOverlayJson(catalogSliceUtf8, previewJson, &err))
	{
		setStatus(err);
		return false;
	}
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!osg)
	{
		return false;
	}

	try
	{
		const nlohmann::json preview = nlohmann::json::parse(previewJson.constData(), nullptr, true);
		std::vector<RobotOsgUi::FeatureCatalogOverlayItem> items;
		for (const auto& item : preview["items"])
		{
			RobotOsgUi::FeatureCatalogOverlayItem o;
			o.displayIndex = item.value("displayIndex", 0);
			const auto readVec = [](const nlohmann::json& arr, osg::Vec3f& v)
			{
				if (arr.is_array() && arr.size() >= 3)
				{
					v.set(static_cast<float>(arr[0].get<double>()), static_cast<float>(arr[1].get<double>()),
						  static_cast<float>(arr[2].get<double>()));
				}
			};
			readVec(item["anchorWorldMm"], o.anchorWorldMm);
			readVec(item["labelWorldMm"], o.labelWorldMm);
			o.hasEdgeSegment = item.value("hasEdgeSegment", false);
			if (o.hasEdgeSegment)
			{
				readVec(item["edgeAWorldMm"], o.edgeAWorldMm);
				readVec(item["edgeBWorldMm"], o.edgeBWorldMm);
			}
			items.push_back(o);
		}
		osg->setFeatureCatalogOverlay(items);
		osg->requestRedraw();
		setStatus(m_chinese ? QStringLiteral("已在 3D 视口显示 %1 个编号特征").arg(items.size())
							: QStringLiteral("Showing %1 numbered features in 3D").arg(items.size()));
		return true;
	}
	catch (...)
	{
		return false;
	}
}

void FeatureTrajectoryPageWidget::clearCandidatePreview()
{
	if (m_host && m_host->osgView())
	{
		m_host->osgView()->clearFeatureCatalogOverlay();
		m_host->osgView()->requestRedraw();
	}
}

bool FeatureTrajectoryPageWidget::commitFeaturePlanFromAi(const QByteArray& planJsonUtf8, QString* summary,
														  QString* err)
{
	try
	{
		const nlohmann::json plan = nlohmann::json::parse(planJsonUtf8.constData(), nullptr, true);
		geoalgo::FeatureListDocument doc;
		std::string parseErr;

		if (plan.contains("features") && plan["features"].is_array())
		{
			doc.schemaVersion = 2;
			doc.defaultStrategyId = plan.value("defaultStrategyId", std::string("EdgeChain"));
			if (plan.contains("workpiece"))
			{
				const auto& wp = plan["workpiece"];
				doc.workpiece.backendIdUtf8 = wp.value("backendIdUtf8", std::string());
				doc.workpiece.stepPathUtf8 = wp.value("stepPathUtf8", std::string());
			}
			for (const auto& item : plan["features"])
			{
				geoalgo::FeatureEntry entry{};
				entry.featureId = item.value("featureId", std::string());
				if (item.contains("strategyId"))
				{
					entry.strategyId = item["strategyId"].get<std::string>();
				}
				else if (item.contains("kind"))
				{
					const std::string kind = item["kind"].get<std::string>();
					if (kind == "FaceUVGrid")
					{
						entry.strategyId = "FaceSection";
					}
					else if (kind == "FaceBoundary" || kind == "FaceParamSurface" || kind == "EdgeChain")
					{
						entry.strategyId = kind;
					}
					else
					{
						entry.strategyId = doc.defaultStrategyId;
					}
				}
				else
				{
					entry.strategyId = doc.defaultStrategyId;
				}
				if (item.contains("geometry"))
				{
					const auto& g = item["geometry"];
					if (g.contains("edgeIndices") && g["edgeIndices"].is_array())
					{
						for (const auto& v : g["edgeIndices"])
						{
							entry.geometry.edgeIndices.push_back(v.get<int>());
						}
					}
					if (g.contains("faceIndices") && g["faceIndices"].is_array())
					{
						for (const auto& v : g["faceIndices"])
						{
							entry.geometry.faceIndices.push_back(v.get<int>());
						}
					}
				}
				else if (item.contains("refs"))
				{
					readGeometryFromCandidateJson(item, entry.geometry);
				}
				if (item.contains("params") && item["params"].is_object())
				{
					entry.params = item["params"];
				}
				else
				{
					nlohmann::json merged = geometry_backend_ops::featureDiscretizerDefaultParams(entry.strategyId);
					if (item.contains("discretize") && item["discretize"].is_object())
					{
						for (auto it = item["discretize"].begin(); it != item["discretize"].end(); ++it)
						{
							merged[it.key()] = it.value();
						}
					}
					if (item.contains("refs") && item["refs"].is_object())
					{
						for (auto it = item["refs"].begin(); it != item["refs"].end(); ++it)
						{
							merged[it.key()] = it.value();
						}
					}
					entry.params = merged;
				}
				doc.features.push_back(std::move(entry));
			}
		}
		else
		{
			if (!geometry_backend_ops::featureListFromJson(planJsonUtf8.constData(), doc, &parseErr))
			{
				if (err)
				{
					*err = QString::fromStdString(parseErr);
				}
				return false;
			}
		}

		if (doc.features.empty())
		{
			if (err)
			{
				*err = QStringLiteral("特征计划缺少 features");
			}
			return false;
		}

		if (!applyFeatureListDocument(doc, false))
		{
			if (err)
			{
				*err = QStringLiteral("无法应用特征表");
			}
			return false;
		}

		const std::string pipelineTemplate = plan.value("suggestedPipelineTemplate", std::string("weld_default"));
		if (!discretizeFromTable(false))
		{
			if (err)
			{
				*err = QStringLiteral("离散失败，请检查特征表");
			}
			return false;
		}

		if (m_simController && m_simController->simulationDock())
		{
			if (TrajectoryEditPageWidget* edit = m_simController->simulationDock()->trajectoryEditPage())
			{
				RobotInstruction::RecipeKind recipeKind = RobotInstruction::RecipeKind::Weld;
				if (pipelineTemplate.find("glue") != std::string::npos)
				{
					recipeKind = RobotInstruction::RecipeKind::Glue;
				}
				else if (pipelineTemplate.find("grind") != std::string::npos)
				{
					recipeKind = RobotInstruction::RecipeKind::Grind;
				}
				edit->applyRecipePresetByKind(recipeKind);
				m_simController->simulationDock()->tabWidget()->setCurrentIndex(
					RobotSimulationDockWidget::kTabIndexTrajectoryEdit);
			}
		}

		if (summary)
		{
			*summary = m_chinese ? QStringLiteral("已离散特征并填充默认工艺流水线，请到轨迹编辑页预览。")
								 : QStringLiteral("Feature discretized; default recipe pipeline filled.");
		}
		return true;
	}
	catch (...)
	{
		if (err)
		{
			*err = QStringLiteral("特征计划 JSON 无效");
		}
		return false;
	}
}
