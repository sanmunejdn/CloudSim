/// @file FeatureTrajectoryPageWidget.cpp
/// @brief 特征轨迹页

#include "FeatureTrajectoryPageWidget.h"

#include "BackendDataManager.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "BrepImportArtifacts.h"
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
#include "TrajectoryOpBridge.h"
#include "TrajectoryPlanConfirmDialog.h"
#include "UiIconDecorators.h"
#include "UserTemplateLibrary.h"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QCursor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHash>
#include <QHeaderView>
#include <QInputDialog>
#include <QItemSelectionModel>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QSpinBox>
#include <QStringList>
#include <QStyle>
#include <QTableView>
#include <QTimer>
#include <QToolTip>
#include <QVBoxLayout>
#include <algorithm>
#include <memory>
#include <unordered_map>

#include <ShapeHandle.h>
#include <json.hpp>

namespace
{
void applyBtnRole(QPushButton* btn, const char* role)
{
	if (!btn)
	{
		return;
	}
	btn->setProperty("btnRole", QLatin1String(role));
	if (btn->style())
	{
		btn->style()->unpolish(btn);
		btn->style()->polish(btn);
	}
}

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

void appendModelXyzToWorldPolyline(IRobotOsgViewHost* osg, const std::string& backendId, const std::vector<float>& xyz,
								   std::vector<cloudsim::core::Vec3>& outWorld)
{
	if (!osg || xyz.size() < 6 || (xyz.size() % 3U) != 0U)
		return;
	outWorld.reserve(outWorld.size() + xyz.size() / 3U);
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3)
	{
		const geoalgo::Point3d modelPt{xyz[i], xyz[i + 1], xyz[i + 2]};
		osg::Vec3f worldPt;
		if (!feature_pick_transform::stepModelPointToWorldMm(osg, backendId, modelPt, worldPt, nullptr))
			worldPt.set(static_cast<float>(modelPt.x), static_cast<float>(modelPt.y), static_cast<float>(modelPt.z));
		outWorld.push_back({static_cast<double>(worldPt.x()), static_cast<double>(worldPt.y()),
							static_cast<double>(worldPt.z())});
	}
}

/// 全量候选仅标号；选中后（条目少）再叠边折线/面片，避免几十个面同时染色
constexpr int kFeatureBodyHighlightMaxCandidates = 16;

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
	m_pickModeAppendBtn = new QPushButton(QStringLiteral("追加到选中"), this);
	m_pickModeNewBtn = new QPushButton(QStringLiteral("新建特征"), this);
	m_pickModeAppendBtn->setCheckable(true);
	m_pickModeNewBtn->setCheckable(true);
	m_pickModeNewBtn->setChecked(true);
	m_pickWriteModeGroup = new QButtonGroup(this);
	m_pickWriteModeGroup->setExclusive(true);
	m_pickWriteModeGroup->addButton(m_pickModeAppendBtn, 0);
	m_pickWriteModeGroup->addButton(m_pickModeNewBtn, 1);
	connect(m_pickWriteModeGroup, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this,
			[this](QAbstractButton*)
			{
				updatePickUiState();
				if (m_pickStatusLabel)
				{
					QToolTip::showText(QCursor::pos(), m_pickStatusLabel->text(), this);
				}
			});
	applyBtnRole(m_pickModeAppendBtn, "secondary");
	applyBtnRole(m_pickModeNewBtn, "primary");
	applyBtnRole(m_pickEdgeBtn = new QPushButton(QStringLiteral("拾取线"), this), "secondary");
	applyBtnRole(m_pickFaceBtn = new QPushButton(QStringLiteral("拾取面"), this), "secondary");
	applyBtnRole(m_cancelPickBtn = new QPushButton(QStringLiteral("取消拾取"), this), "secondary");
	pickRow->addWidget(m_pickModeAppendBtn);
	pickRow->addWidget(m_pickModeNewBtn);
	pickRow->addWidget(m_pickEdgeBtn);
	pickRow->addWidget(m_pickFaceBtn);
	pickRow->addWidget(m_cancelPickBtn);
	layout->addLayout(pickRow);

	m_pickStatusLabel = new QLabel(this);
	m_pickStatusLabel->setWordWrap(true);
	m_pickStatusLabel->setStyleSheet(QStringLiteral("color: #288cf0;"));
	layout->addWidget(m_pickStatusLabel);

	auto* strategyRow = new QHBoxLayout;
	strategyRow->addWidget(new QLabel(QStringLiteral("离散策略"), this));
	m_strategyCombo = new QComboBox(this);
	strategyRow->addWidget(m_strategyCombo, 1);
	layout->addLayout(strategyRow);

	auto* templateRow = new QHBoxLayout;
	m_discretizeTemplateCombo = new QComboBox(this);
	m_discretizeTemplateCombo->setMaxVisibleItems(16);
	m_saveDiscretizeTemplateBtn = new QPushButton(QStringLiteral("保存"), this);
	m_loadDiscretizeTemplateBtn = new QPushButton(QStringLiteral("加载"), this);
	m_deleteDiscretizeTemplateBtn = new QPushButton(QStringLiteral("删除"), this);
	m_importDiscretizeTemplateBtn = new QPushButton(QStringLiteral("导入"), this);
	m_exportDiscretizeTemplateBtn = new QPushButton(QStringLiteral("导出"), this);
	templateRow->addWidget(m_discretizeTemplateCombo, 2);
	templateRow->addWidget(m_saveDiscretizeTemplateBtn);
	templateRow->addWidget(m_loadDiscretizeTemplateBtn);
	templateRow->addWidget(m_deleteDiscretizeTemplateBtn);
	templateRow->addWidget(m_importDiscretizeTemplateBtn);
	templateRow->addWidget(m_exportDiscretizeTemplateBtn);
	layout->addLayout(templateRow);

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
	connect(m_saveDiscretizeTemplateBtn, &QPushButton::clicked, this,
			&FeatureTrajectoryPageWidget::onSaveDiscretizeTemplateClicked);
	connect(m_loadDiscretizeTemplateBtn, &QPushButton::clicked, this,
			&FeatureTrajectoryPageWidget::onLoadDiscretizeTemplateClicked);
	connect(m_deleteDiscretizeTemplateBtn, &QPushButton::clicked, this,
			&FeatureTrajectoryPageWidget::onDeleteDiscretizeTemplateClicked);
	connect(m_importDiscretizeTemplateBtn, &QPushButton::clicked, this,
			&FeatureTrajectoryPageWidget::onImportDiscretizeTemplateClicked);
	connect(m_exportDiscretizeTemplateBtn, &QPushButton::clicked, this,
			&FeatureTrajectoryPageWidget::onExportDiscretizeTemplateClicked);

	UiIconDecorators::apply(m_saveDiscretizeTemplateBtn, UiIconId::SaveTemplate);
	UiIconDecorators::apply(m_loadDiscretizeTemplateBtn, UiIconId::LoadTemplate);
	UiIconDecorators::apply(m_deleteDiscretizeTemplateBtn, UiIconId::Delete);
	UiIconDecorators::apply(m_importDiscretizeTemplateBtn, UiIconId::OpenProject);
	UiIconDecorators::apply(m_exportDiscretizeTemplateBtn, UiIconId::Export);
	refreshDiscretizeTemplateCombo();

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
				const QModelIndex under = m_featureTable->indexAt(pos);
				if (under.isValid())
				{
					m_featureTable->selectRow(under.row());
					m_featureModel->setSelectedRow(under.row());
				}
				const int row = m_featureModel ? m_featureModel->selectedRow() : -1;
				const geoalgo::FeatureEntry entry =
					(row >= 0 && m_featureModel) ? m_featureModel->entryAt(row) : geoalgo::FeatureEntry{};

				QMenu menu(this);
				QAction* removeFacesAct =
					menu.addAction(m_chinese ? QStringLiteral("移除面…") : QStringLiteral("Remove faces…"), this,
								   &FeatureTrajectoryPageWidget::onRemoveFacesFromFeature);
				removeFacesAct->setEnabled(row >= 0 && !entry.geometry.faceIndices.empty());
				QAction* removeEdgesAct =
					menu.addAction(m_chinese ? QStringLiteral("移除边…") : QStringLiteral("Remove edges…"), this,
								   &FeatureTrajectoryPageWidget::onRemoveEdgesFromFeature);
				removeEdgesAct->setEnabled(row >= 0 && !entry.geometry.edgeIndices.empty());
				menu.addSeparator();
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
				const QString backendId =
					m_backendCombo ? m_backendCombo->currentData().toString() : QString();
				if (backendId == m_lastWorkpieceBackendId)
				{
					return;
				}
				m_lastWorkpieceBackendId = backendId;
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
	if (m_pickModeAppendBtn)
	{
		m_pickModeAppendBtn->setText(zh ? QStringLiteral("追加到选中") : QStringLiteral("Append to selected"));
	}
	if (m_pickModeNewBtn)
	{
		m_pickModeNewBtn->setText(zh ? QStringLiteral("新建特征") : QStringLiteral("New feature"));
	}
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
	if (m_saveDiscretizeTemplateBtn)
	{
		m_saveDiscretizeTemplateBtn->setText(zh ? QStringLiteral("保存") : QStringLiteral("Save"));
	}
	if (m_loadDiscretizeTemplateBtn)
	{
		m_loadDiscretizeTemplateBtn->setText(zh ? QStringLiteral("加载") : QStringLiteral("Load"));
	}
	if (m_deleteDiscretizeTemplateBtn)
	{
		m_deleteDiscretizeTemplateBtn->setText(zh ? QStringLiteral("删除") : QStringLiteral("Delete"));
	}
	if (m_importDiscretizeTemplateBtn)
	{
		m_importDiscretizeTemplateBtn->setText(zh ? QStringLiteral("导入") : QStringLiteral("Import"));
	}
	if (m_exportDiscretizeTemplateBtn)
	{
		m_exportDiscretizeTemplateBtn->setText(zh ? QStringLiteral("导出") : QStringLiteral("Export"));
	}
	if (m_discretizeTemplateCombo)
	{
		m_discretizeTemplateCombo->setToolTip(zh ? QStringLiteral("离散策略参数模板")
												 : QStringLiteral("Discretize strategy/param templates"));
		refreshDiscretizeTemplateCombo();
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

void FeatureTrajectoryPageWidget::refreshWorkpieces()
{
	refreshBackendCombo();
}

void FeatureTrajectoryPageWidget::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	refreshBackendCombo();
}

void FeatureTrajectoryPageWidget::onTableSelectionChanged()
{
	loadParamsForSelectedRow();
	updatePickUiState();
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

bool FeatureTrajectoryPageWidget::isAppendPickMode() const
{
	return m_pickModeAppendBtn && m_pickModeAppendBtn->isChecked();
}

void FeatureTrajectoryPageWidget::onRemoveFacesFromFeature()
{
	removeGeometryIndicesFromRow(m_featureModel ? m_featureModel->selectedRow() : -1, true);
}

void FeatureTrajectoryPageWidget::onRemoveEdgesFromFeature()
{
	removeGeometryIndicesFromRow(m_featureModel ? m_featureModel->selectedRow() : -1, false);
}

void FeatureTrajectoryPageWidget::removeGeometryIndicesFromRow(const int row, const bool removeFaces)
{
	if (!m_featureModel || row < 0 || row >= m_featureModel->rowCount())
	{
		return;
	}
	geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
	std::vector<int>& indices = removeFaces ? entry.geometry.faceIndices : entry.geometry.edgeIndices;
	if (indices.empty())
	{
		return;
	}

	QDialog dlg(this);
	dlg.setWindowTitle(removeFaces ? (m_chinese ? QStringLiteral("移除面") : QStringLiteral("Remove faces"))
								   : (m_chinese ? QStringLiteral("移除边") : QStringLiteral("Remove edges")));
	auto* layout = new QVBoxLayout(&dlg);
	layout->addWidget(new QLabel(m_chinese ? QStringLiteral("勾选要移除的项：") : QStringLiteral("Check items to remove:"),
								 &dlg));
	auto* list = new QListWidget(&dlg);
	list->setSelectionMode(QAbstractItemView::NoSelection);
	for (const int idx : indices)
	{
		const QString text =
			removeFaces ? (m_chinese ? QStringLiteral("面 %1").arg(idx) : QStringLiteral("face %1").arg(idx))
						: (m_chinese ? QStringLiteral("边 %1").arg(idx) : QStringLiteral("edge %1").arg(idx));
		auto* item = new QListWidgetItem(text, list);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Unchecked);
		item->setData(Qt::UserRole, idx);
	}
	layout->addWidget(list);
	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	layout->addWidget(buttons);
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	if (dlg.exec() != QDialog::Accepted)
	{
		return;
	}

	QSet<int> toRemove;
	for (int i = 0; i < list->count(); ++i)
	{
		const QListWidgetItem* item = list->item(i);
		if (item && item->checkState() == Qt::Checked)
		{
			toRemove.insert(item->data(Qt::UserRole).toInt());
		}
	}
	if (toRemove.isEmpty())
	{
		return;
	}

	std::vector<int> kept;
	kept.reserve(indices.size());
	for (const int idx : indices)
	{
		if (!toRemove.contains(idx))
		{
			kept.push_back(idx);
		}
	}
	indices = std::move(kept);

	if (entry.geometry.faceIndices.empty() && entry.geometry.edgeIndices.empty())
	{
		m_featureModel->removeRows(QList<int>{row});
	}
	else
	{
		(void)m_featureModel->updateEntry(row, entry);
		refreshBrepInfoForSelection();
	}
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
	const bool appendMode = isAppendPickMode();
	applyBtnRole(m_pickModeAppendBtn, appendMode ? "primary" : "secondary");
	applyBtnRole(m_pickModeNewBtn, appendMode ? "secondary" : "primary");
	if (m_pickModeAppendBtn)
	{
		m_pickModeAppendBtn->setToolTip(
			m_chinese ? QStringLiteral("拾取结果追加到当前选中特征行")
					  : QStringLiteral("Append pick result to the selected feature row"));
	}
	if (m_pickModeNewBtn)
	{
		m_pickModeNewBtn->setToolTip(m_chinese ? QStringLiteral("拾取结果新建一行特征")
											  : QStringLiteral("Create a new feature row from the pick"));
	}
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
	if (!hasWorkpiece)
	{
		m_pickStatusLabel->setText(emptyWorkpieceHint());
		return;
	}
	const int sel = m_featureModel ? m_featureModel->selectedRow() : -1;
	if (m_pickSession == PickSessionKind::Edge)
	{
		if (appendMode && sel >= 0)
		{
			m_pickStatusLabel->setText(
				m_chinese ? QStringLiteral("追加边到当前特征（行 %1）：在视口点击…").arg(sel + 1)
						  : QStringLiteral("Append edge to feature row %1…").arg(sel + 1));
		}
		else if (appendMode)
		{
			m_pickStatusLabel->setText(m_chinese ? QStringLiteral("追加模式：请先选中特征行，再在视口点击边…")
												 : QStringLiteral("Append mode: select a feature row, then click an edge…"));
		}
		else
		{
			m_pickStatusLabel->setText(m_chinese ? QStringLiteral("新建特征：请在视口中点击一条边…")
												 : QStringLiteral("New feature: click an edge in the 3D view…"));
		}
	}
	else if (m_pickSession == PickSessionKind::Face)
	{
		if (appendMode && sel >= 0)
		{
			m_pickStatusLabel->setText(
				m_chinese ? QStringLiteral("追加面到当前特征（行 %1）：在视口点击…").arg(sel + 1)
						  : QStringLiteral("Append face to feature row %1…").arg(sel + 1));
		}
		else if (appendMode)
		{
			m_pickStatusLabel->setText(m_chinese ? QStringLiteral("追加模式：请先选中特征行，再在视口点击面…")
												 : QStringLiteral("Append mode: select a feature row, then click a face…"));
		}
		else
		{
			m_pickStatusLabel->setText(m_chinese ? QStringLiteral("新建特征：请在视口中点击一个面…")
												 : QStringLiteral("New feature: click a face in the 3D view…"));
		}
	}
	else if (appendMode)
	{
		m_pickStatusLabel->setText(
			m_chinese ? QStringLiteral("写入模式：追加到选中 — 选中特征行后点「拾取线/面」")
					  : QStringLiteral("Write mode: Append — select a row, then Pick edge/face"));
	}
	else
	{
		m_pickStatusLabel->setText(m_chinese ? QStringLiteral("写入模式：新建特征 — 点「拾取线/面」将新建一行")
											 : QStringLiteral("Write mode: New feature — Pick edge/face creates a row"));
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
		setStatus(emptyWorkpieceHint());
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
	if (isAppendPickMode())
	{
		const int sel = m_featureModel ? m_featureModel->selectedRow() : -1;
		setStatus(sel >= 0 ? (m_chinese ? QStringLiteral("边拾取：追加到当前特征")
										: QStringLiteral("Edge pick: append to selected feature"))
						   : (m_chinese ? QStringLiteral("边拾取（追加模式）：请先选中特征行")
										: QStringLiteral("Edge pick (append): select a feature row first")));
	}
	else
	{
		setStatus(m_chinese ? QStringLiteral("边拾取：将新建特征行") : QStringLiteral("Edge pick: will create a new feature"));
	}
}

void FeatureTrajectoryPageWidget::onPickFace()
{
	if (!m_host || m_backendCombo->currentIndex() < 0)
	{
		setStatus(emptyWorkpieceHint());
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
	if (isAppendPickMode())
	{
		const int sel = m_featureModel ? m_featureModel->selectedRow() : -1;
		setStatus(sel >= 0 ? (m_chinese ? QStringLiteral("面拾取：追加到当前特征")
										: QStringLiteral("Face pick: append to selected feature"))
						   : (m_chinese ? QStringLiteral("面拾取（追加模式）：请先选中特征行")
										: QStringLiteral("Face pick (append): select a feature row first")));
	}
	else
	{
		setStatus(m_chinese ? QStringLiteral("面拾取：将新建特征行") : QStringLiteral("Face pick: will create a new feature"));
	}
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

bool FeatureTrajectoryPageWidget::canAppendPickToFeature(const geoalgo::FeatureEntry& entry, const bool pickFace,
														QString* err)
{
	const geoalgo::GeometryAffinity affinity = geometry_backend_ops::featureDiscretizerAffinity(entry.strategyId);
	if (pickFace)
	{
		if (affinity == geoalgo::GeometryAffinity::Line)
		{
			if (err)
			{
				*err = QStringLiteral("当前行为线策略，无法追加面；请取消表选中后新建，或先改策略");
			}
			return false;
		}
		return true;
	}
	if (affinity == geoalgo::GeometryAffinity::Face)
	{
		if (err)
		{
			*err = QStringLiteral("当前行为面策略，无法追加边；FaceOffsetCurve 等可用「任意」策略追加边");
		}
		return false;
	}
	return true;
}

bool FeatureTrajectoryPageWidget::featureNeedsMoreGeometry(const geoalgo::FeatureEntry& entry)
{
	if (entry.strategyId == "FaceIntersection")
	{
		return entry.geometry.faceIndices.size() < 2U;
	}
	if (entry.strategyId == "FaceOffsetCurve")
	{
		return entry.geometry.faceIndices.empty() || entry.geometry.edgeIndices.empty();
	}
	return false;
}

bool FeatureTrajectoryPageWidget::tryAppendPickToSelectedFeature(const bool pickFace, const geoalgo::Point3d& modelA,
																 const geoalgo::Point3d& modelB,
																 const int knownFaceIndex, const int knownEdgeIndex,
																 QString* err)
{
	if (!m_featureModel)
	{
		return false;
	}
	const int row = m_featureModel->selectedRow();
	if (row < 0)
	{
		return false;
	}
	geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
	if (!canAppendPickToFeature(entry, pickFace, err))
	{
		return false;
	}
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

	geoalgo::FeatureEntry picked{};
	picked.strategyId = entry.strategyId;
	picked.params = entry.params;
	std::string stdErr;
	if (!geometry_backend_ops::buildFeatureEntryFromModelPick(wp, shape, entry.strategyId, pickFace, modelA, modelB,
															  picked, &stdErr, knownFaceIndex, knownEdgeIndex))
	{
		if (err)
		{
			*err = QString::fromStdString(stdErr);
		}
		return false;
	}

	if (pickFace)
	{
		if (picked.geometry.faceIndices.empty())
		{
			if (err)
			{
				*err = QStringLiteral("未解析到面索引");
			}
			return false;
		}
		const int faceIdx = picked.geometry.faceIndices.front();
		for (const int existing : entry.geometry.faceIndices)
		{
			if (existing == faceIdx)
			{
				if (err)
				{
					*err = m_chinese ? QStringLiteral("面 %1 已在当前特征中").arg(faceIdx)
									 : QStringLiteral("Face %1 already in feature").arg(faceIdx);
				}
				return false;
			}
		}
		entry.geometry.faceIndices.push_back(faceIdx);
	}
	else
	{
		if (picked.geometry.edgeIndices.empty())
		{
			if (err)
			{
				*err = QStringLiteral("未解析到边索引");
			}
			return false;
		}
		const int edgeIdx = picked.geometry.edgeIndices.front();
		for (const int existing : entry.geometry.edgeIndices)
		{
			if (existing == edgeIdx)
			{
				if (err)
				{
					*err = m_chinese ? QStringLiteral("边 %1 已在当前特征中").arg(edgeIdx)
									 : QStringLiteral("Edge %1 already in feature").arg(edgeIdx);
				}
				return false;
			}
		}
		entry.geometry.edgeIndices.push_back(edgeIdx);
	}

	if (!m_featureModel->updateEntry(row, entry))
	{
		if (err)
		{
			*err = QStringLiteral("更新特征失败");
		}
		return false;
	}
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

	const bool pickFace = kind == PickKind::MeshFace;
	const int knownFaceIndex = pick.brepNativePick && pickFace ? pick.brepFaceIndex : -1;
	const int knownEdgeIndex = pick.brepNativePick && !pickFace ? pick.brepEdgeIndex : -1;
	QString pickErr;

	++m_strategyRowSyncDepth;
	m_suppressParamRediscretize = true;

	const bool appendMode = isAppendPickMode();
	const int selectedRow = m_featureModel ? m_featureModel->selectedRow() : -1;
	if (appendMode && selectedRow < 0)
	{
		m_suppressParamRediscretize = false;
		--m_strategyRowSyncDepth;
		QMessageBox::warning(this, QStringLiteral("Pick"),
							 m_chinese ? QStringLiteral("追加模式：请先在特征表中选中一行")
									   : QStringLiteral("Append mode: select a feature row first"));
		return;
	}

	const bool wantAppend = appendMode && selectedRow >= 0;
	bool appended = false;
	if (wantAppend)
	{
		if (!tryAppendPickToSelectedFeature(pickFace, modelA, modelB, knownFaceIndex, knownEdgeIndex, &pickErr))
		{
			m_suppressParamRediscretize = false;
			--m_strategyRowSyncDepth;
			QMessageBox::warning(this, QStringLiteral("Pick"), pickErr);
			// 保持拾取态，便于换目标再点
			return;
		}
		appended = true;
	}
	else
	{
		geoalgo::FeatureEntry entry;
		if (!buildFeatureEntryFromPick(pickFace, modelA, modelB, knownFaceIndex, knownEdgeIndex, entry, &pickErr))
		{
			m_suppressParamRediscretize = false;
			--m_strategyRowSyncDepth;
			QMessageBox::warning(this, QStringLiteral("Pick"), pickErr);
			exitPickMode();
			return;
		}
		m_featureModel->appendEntry(entry);
	}

	const int row = m_featureModel->selectedRow();
	geoalgo::FeatureEntry entry = row >= 0 ? m_featureModel->entryAt(row) : geoalgo::FeatureEntry{};
	syncStrategyComboToEntry(entry);
	loadParamsForSelectedRow();
	refreshBrepInfoForSelection();
	m_suppressParamRediscretize = false;
	setFeatureEditActive(true);

	const bool needMore = featureNeedsMoreGeometry(entry);
	if (needMore)
	{
		QString hint;
		if (entry.strategyId == "FaceIntersection")
		{
			hint = m_chinese ? QStringLiteral("两面交线还需再拾取一面（继续点视口）")
							 : QStringLiteral("FaceIntersection needs another face (keep clicking)");
		}
		else if (entry.strategyId == "FaceOffsetCurve")
		{
			hint = entry.geometry.faceIndices.empty()
					   ? (m_chinese ? QStringLiteral("面内偏置还需拾取面")
									: QStringLiteral("FaceOffsetCurve still needs a face"))
					   : (m_chinese ? QStringLiteral("面内偏置还需拾取边")
									: QStringLiteral("FaceOffsetCurve still needs an edge"));
		}
		setStatus((appended ? (m_chinese ? QStringLiteral("已追加到 %1；") : QStringLiteral("Appended to %1; "))
							: (m_chinese ? QStringLiteral("已添加 %1；") : QStringLiteral("Added %1; ")))
					  .arg(QString::fromStdString(entry.featureId)) +
				  hint);
		// FaceOffsetCurve 缺边/面时切换拾取种类；交线保持面拾取
		if (entry.strategyId == "FaceOffsetCurve")
		{
			if (entry.geometry.faceIndices.empty() && m_pickSession != PickSessionKind::Face)
			{
				--m_strategyRowSyncDepth;
				onPickFace();
				return;
			}
			if (entry.geometry.edgeIndices.empty() && m_pickSession != PickSessionKind::Edge)
			{
				--m_strategyRowSyncDepth;
				onPickEdge();
				return;
			}
		}
		updatePickUiState();
		--m_strategyRowSyncDepth;
		return;
	}

	exitPickMode();
	setStatus(m_chinese
				  ? QStringLiteral("%1特征 %2，正在离散…")
						.arg(appended ? QStringLiteral("已更新") : QStringLiteral("已添加"),
							 QString::fromStdString(entry.featureId))
				  : QStringLiteral("%1 feature %2, discretizing…")
						.arg(appended ? QStringLiteral("Updated") : QStringLiteral("Added"),
							 QString::fromStdString(entry.featureId)));
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
	const QString prevBackendId =
		m_backendCombo ? m_backendCombo->currentData().toString() : QString();
	const QSignalBlocker blocker(m_backendCombo);
	m_backendCombo->clear();
	if (!m_host)
	{
		m_lastWorkpieceBackendId.clear();
		updatePickUiState();
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		m_lastWorkpieceBackendId.clear();
		updatePickUiState();
		return;
	}
	BackendDataManager& mgr = doc->backend();
	const auto all = doc->listObjects();
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
		const std::string cn = data->className();
		if (!backend_type::isMeshClassName(cn) && !backend_type::isBrepWorkpieceClassName(cn))
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
		const bool isBrepModel = backend_type::isBrepWorkpieceClassName(cn);
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
		const QString displayName = QString::fromStdString(data->name());
		QString label;
		if (isBrepModel)
		{
			// 内存 B-rep / AI 基本体无磁盘 STEP 名，用显示名
			label = displayName.isEmpty() ? backendId : QStringLiteral("%1 (%2)").arg(displayName, backendId);
		}
		else
		{
			const QString fileName = QFileInfo(stepPath).fileName();
			label = fileName.isEmpty() ? backendId : QStringLiteral("%1 (%2)").arg(backendId, fileName);
		}
		WorkpieceComboCandidate candidate;
		candidate.backendId = backendId;
		candidate.label = label;
		// 仅对真实 STEP 路径去重；ai:// 等虚拟路径按 backendId 保留各自条目
		candidate.dedupeKey =
			(!isBrepModel && isStepSourcePath(stepPath)) ? stepPath.toLower() : backendId;
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
		m_lastWorkpieceBackendId.clear();
	}
	else
	{
		if (!prevBackendId.isEmpty() && selectBackendComboById(prevBackendId))
		{
			m_lastWorkpieceBackendId = prevBackendId;
		}
		else if (m_backendCombo->currentIndex() < 0)
		{
			m_backendCombo->setCurrentIndex(0);
			m_lastWorkpieceBackendId = m_backendCombo->currentData().toString();
		}
		else
		{
			m_lastWorkpieceBackendId = m_backendCombo->currentData().toString();
		}
		// Open Model 后 bindHost 会刷新 combo；全量边/面目录含二面角，大型装配可达数分钟，不能排队到导入路径上
		// 目录在用户切换工件或 AI 调用 ensureFeatureCatalogEnumerated 时再算
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
			*err = emptyWorkpieceHint();
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

QString FeatureTrajectoryPageWidget::emptyWorkpieceHint() const
{
	return m_chinese ? QStringLiteral("请导入 STEP 或用 AI 创建基本体（CAD/BREP 工件）")
					 : QStringLiteral("Import STEP or create an AI primitive (CAD/BREP workpiece)");
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
			*err = emptyWorkpieceHint();
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
		if (const auto data = m_host->document()->findObject(backendId.toStdString()))
		{
			if (isBrepWorkpieceClassName(data->className()) && data->hasGeometry())
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
			*err = QStringLiteral("请先在轨迹生成页选择工件");
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

	// 内存 BrepModel（含 AI 基本体）无磁盘 STEP，须用 ShapeHandle 算锚点
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	if (!resolveWorkpieceShapeForBackend(backendId, shape, wp, err))
	{
		return false;
	}
	wp.backendIdUtf8 = backendId.toStdString();
	wp.stepPathUtf8 = stepPath.toStdString();

	try
	{
		const nlohmann::json slice = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		nlohmann::json preview;
		preview["backendIdUtf8"] = backendId.toStdString();
		nlohmann::json items = nlohmann::json::array();

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
			if (!geometry_backend_ops::computeFeatureAnchor(wp, shape, geometry, anchor, &anchorErr))
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
		if (items.empty())
		{
			if (err)
			{
				*err = QStringLiteral("无法计算特征锚点（高亮/标号为空）");
			}
			return false;
		}
		return true;
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

	QString backendId;
	QString stepPath;
	(void)currentWorkpiece(backendId, stepPath);

	try
	{
		const nlohmann::json preview = nlohmann::json::parse(previewJson.constData(), nullptr, true);
		std::vector<RobotOsgUi::FeatureCatalogOverlayItem> items;
		for (const auto& item : preview["items"])
		{
			RobotOsgUi::FeatureCatalogOverlayItem o;
			o.displayIndex = item.value("displayIndex", 0);
			const auto readVec = [](const nlohmann::json& arr, cloudsim::core::Vec3& v)
			{
				if (arr.is_array() && arr.size() >= 3)
				{
					v.x = arr[0].get<double>();
					v.y = arr[1].get<double>();
					v.z = arr[2].get<double>();
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

		// 边：始终叠完整折线；面：仅选中后（条目少）叠半透明面片，避免全量候选染色
		const bool highlightFaceBodies =
			static_cast<int>(items.size()) > 0 && static_cast<int>(items.size()) <= kFeatureBodyHighlightMaxCandidates;
		if (!backendId.isEmpty())
		{
			geoalgo::ShapeHandle shape;
			geoalgo::WorkpieceRef wp;
			if (resolveWorkpieceShapeForBackend(backendId, shape, wp, nullptr) && !shape.isNull())
			{
				std::string artErr;
				const std::shared_ptr<geoalgo::BrepImportArtifacts> artifacts =
					geoalgo::getOrBuildBrepImportArtifacts(shape, &artErr);
				if (artifacts)
				{
					(void)geoalgo::ensureBrepImportPickArtifacts(shape, *artifacts, nullptr);
					const nlohmann::json slice =
						nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
					std::unordered_map<int, geoalgo::FeatureGeometry> geomByDisplay;
					if (slice.contains("candidates") && slice["candidates"].is_array())
					{
						for (const auto& c : slice["candidates"])
						{
							geoalgo::FeatureGeometry geometry;
							if (!readGeometryFromCandidateJson(c, geometry))
								continue;
							geomByDisplay[c.value("displayIndex", 0)] = std::move(geometry);
						}
					}
					const std::string backendStd = backendId.toStdString();
					for (RobotOsgUi::FeatureCatalogOverlayItem& o : items)
					{
						const auto it = geomByDisplay.find(o.displayIndex);
						if (it == geomByDisplay.end())
							continue;
						const geoalgo::FeatureGeometry& geometry = it->second;
						if (!geometry.edgeIndices.empty() && !artifacts->edgePolylines.empty())
						{
							const int edgeIdx = geometry.edgeIndices.front();
							if (edgeIdx >= 0 &&
								static_cast<std::size_t>(edgeIdx) < artifacts->edgePolylines.size())
							{
								appendModelXyzToWorldPolyline(osg, backendStd,
															  artifacts->edgePolylines[static_cast<std::size_t>(edgeIdx)],
															  o.edgePolylineWorldMm);
							}
						}
						if (highlightFaceBodies && !geometry.faceIndices.empty() && !artifacts->faceSoups.empty())
						{
							const int faceIdx = geometry.faceIndices.front();
							if (faceIdx >= 0 && static_cast<std::size_t>(faceIdx) < artifacts->faceSoups.size())
							{
								appendModelXyzToWorldPolyline(osg, backendStd,
															  artifacts->faceSoups[static_cast<std::size_t>(faceIdx)],
															  o.faceTrianglesWorldMm);
							}
						}
					}
				}
			}
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
				(void)readGeometryFromCandidateJson(item, entry.geometry);
				normalizeEntryStrategyForGeometry(entry);
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
		for (const geoalgo::FeatureEntry& f : doc.features)
		{
			if (f.geometry.edgeIndices.empty() && f.geometry.faceIndices.empty())
			{
				if (err)
				{
					*err = m_chinese ? QStringLiteral("特征缺少边/面索引（edgeIndices/faceIndices），无法离散")
									 : QStringLiteral("Feature missing edgeIndices/faceIndices");
				}
				return false;
			}
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

		const bool hasPipeline = plan.contains("pipeline") && plan["pipeline"].is_array() && !plan["pipeline"].empty();
		if (m_simController && m_simController->simulationDock())
		{
			if (TrajectoryEditPageWidget* edit = m_simController->simulationDock()->trajectoryEditPage())
			{
				if (hasPipeline)
				{
					std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
					std::string codecErr;
					if (!RobotInstruction::trajectoryPipelineFromJson(plan["pipeline"], ops, &codecErr))
					{
						if (err)
						{
							*err = QString::fromStdString(codecErr.empty() ? "pipeline 无效" : codecErr);
						}
						return false;
					}
					edit->applyPipelineOps(std::move(ops));
				}
				else
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
				}
				m_simController->showRobotDockTab(RobotSimulationDockWidget::kTabIndexTrajectoryEdit);
			}
		}

		if (summary)
		{
			*summary = m_chinese ? QStringLiteral("已离散特征并写入管线算子，请到轨迹编辑页预览。")
								 : QStringLiteral("Feature discretized; pipeline ops applied.");
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

int FeatureTrajectoryPageWidget::proposeAndConfirmTrajectoryPlan(const QByteArray& planInUtf8, QByteArray& planOutUtf8,
																QString* err, const bool showRetry)
{
	planOutUtf8.clear();
	try
	{
		nlohmann::json plan = nlohmann::json::parse(planInUtf8.constData(), nullptr, true);
		QString enrichErr;
		if (!enrichTrajectoryPlanJsonInPlace(plan, &enrichErr))
		{
			if (err)
				*err = enrichErr.isEmpty() ? QStringLiteral("无法补全离散计划") : enrichErr;
			return static_cast<int>(TrajectoryPlanConfirmDialog::Outcome::Cancelled);
		}
		TrajectoryPlanConfirmDialog dlg(this);
		dlg.setUseChinese(m_chinese);
		dlg.setShowRetry(showRetry);
		QString loadErr;
		if (!dlg.loadPlan(QByteArray::fromStdString(plan.dump()), &loadErr))
		{
			if (err)
				*err = loadErr;
			return static_cast<int>(TrajectoryPlanConfirmDialog::Outcome::Cancelled);
		}
		dlg.exec();
		const auto outcome = dlg.outcome();
		if (outcome == TrajectoryPlanConfirmDialog::Outcome::Accepted)
			planOutUtf8 = dlg.resultPlanJson();
		return static_cast<int>(outcome);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("特征计划 JSON 无效");
		return static_cast<int>(TrajectoryPlanConfirmDialog::Outcome::Cancelled);
	}
}

bool FeatureTrajectoryPageWidget::loadBoundTrajectoryPlanJson(QByteArray& planOutUtf8, QString* err)
{
	planOutUtf8.clear();
	if (!m_session)
	{
		if (err)
			*err = m_chinese ? QStringLiteral("轨迹会话未就绪") : QStringLiteral("Session not ready");
		return false;
	}
	const std::string src = m_session->boundSourceFeatureJson();
	if (src.empty())
	{
		if (err)
			*err = m_chinese ? QStringLiteral("当前 PathPlan 无特征离散数据，请先完成离散")
							: QStringLiteral("No sourceFeatureJson on bound PathPlan");
		return false;
	}
	try
	{
		geoalgo::FeatureListDocument doc;
		std::string parseErr;
		if (!geometry_backend_ops::featureListFromJson(src, doc, &parseErr) || doc.features.empty())
		{
			if (err)
				*err = parseErr.empty() ? QStringLiteral("无法解析 sourceFeatureJson") : QString::fromStdString(parseErr);
			return false;
		}
		nlohmann::json plan;
		plan["version"] = 2;
		plan["mode"] = "revise";
		plan["schemaVersion"] = doc.schemaVersion;
		plan["defaultStrategyId"] = doc.defaultStrategyId;
		plan["workpiece"] = {{"backendIdUtf8", doc.workpiece.backendIdUtf8},
							 {"stepPathUtf8", doc.workpiece.stepPathUtf8}};
		nlohmann::json feats = nlohmann::json::array();
		for (const auto& entry : doc.features)
		{
			nlohmann::json f;
			f["featureId"] = entry.featureId;
			f["strategyId"] = entry.strategyId;
			f["kind"] = entry.strategyId;
			f["params"] = entry.params;
			f["geometry"] = {{"faceIndices", entry.geometry.faceIndices}, {"edgeIndices", entry.geometry.edgeIndices}};
			feats.push_back(std::move(f));
		}
		plan["features"] = feats;
		plan["pipeline"] = RobotInstruction::trajectoryPipelineToJson(m_session->pipelineOps());
		planOutUtf8 = QByteArray::fromStdString(plan.dump());
		return true;
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("组装 revise 计划失败");
		return false;
	}
}

bool FeatureTrajectoryPageWidget::reviseFeaturePlanFromAi(const QByteArray& planJsonUtf8, QString* summary, QString* err)
{
	try
	{
		const nlohmann::json plan = nlohmann::json::parse(planJsonUtf8.constData(), nullptr, true);
		if (!commitFeaturePlanFromAi(planJsonUtf8, summary, err))
			return false;
		if (summary)
		{
			*summary = m_chinese ? QStringLiteral("已按确认结果重离散并更新管线算子。")
								 : QStringLiteral("Re-discretized and pipeline updated.");
		}
		(void)plan;
		return true;
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("特征计划 JSON 无效");
		return false;
	}
}

void FeatureTrajectoryPageWidget::refreshDiscretizeTemplateCombo()
{
	if (!m_discretizeTemplateCombo)
	{
		return;
	}
	const QString prevId = m_discretizeTemplateCombo->currentData().toString();
	m_discretizeTemplateCombo->blockSignals(true);
	m_discretizeTemplateCombo->clear();
	m_discretizeTemplateCombo->addItem(m_chinese ? QStringLiteral("（选择模板）") : QStringLiteral("(Select template)"),
									   QString());
	for (const UserTemplateEntry& e : UserTemplateLibrary::list(UserTemplateKind::Discretize))
	{
		m_discretizeTemplateCombo->addItem(e.name, e.id);
	}
	const int idx = prevId.isEmpty() ? 0 : m_discretizeTemplateCombo->findData(prevId);
	m_discretizeTemplateCombo->setCurrentIndex(idx >= 0 ? idx : 0);
	m_discretizeTemplateCombo->blockSignals(false);
}

bool FeatureTrajectoryPageWidget::applyDiscretizeTemplatePayload(const nlohmann::json& payload, QString* err)
{
	if (!payload.is_object() || !payload.contains("strategyId"))
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("模板缺少 strategyId") : QStringLiteral("Missing strategyId");
		}
		return false;
	}
	const std::string strategyId = payload.value("strategyId", std::string());
	if (strategyId.empty() || !m_strategyCombo)
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("策略无效") : QStringLiteral("Invalid strategy");
		}
		return false;
	}
	nlohmann::json params =
		payload.contains("params") && payload["params"].is_object() ? payload["params"] : nlohmann::json::object();

	if (m_strategyCombo->findData(QString::fromStdString(strategyId)) < 0)
	{
		refreshStrategyCombo(geoalgo::GeometryAffinity::Any);
	}
	const int idx2 = m_strategyCombo->findData(QString::fromStdString(strategyId));
	if (idx2 < 0)
	{
		if (err)
		{
			*err = m_chinese ? QStringLiteral("当前环境无此离散策略") : QStringLiteral("Strategy not available");
		}
		return false;
	}

	m_suppressParamRediscretize = true;
	{
		const QSignalBlocker blocker(m_strategyCombo);
		m_strategyCombo->setCurrentIndex(idx2);
	}
	m_paramPanel->rebuildForStrategy(strategyId);
	m_paramPanel->loadParams(params);

	const int row = m_featureModel ? m_featureModel->selectedRow() : -1;
	if (row >= 0)
	{
		geoalgo::FeatureEntry entry = m_featureModel->entryAt(row);
		entry.strategyId = strategyId;
		entry.params = params;
		(void)m_featureModel->updateEntry(row, entry);
	}
	m_suppressParamRediscretize = false;
	if (m_featureEditActive)
	{
		scheduleParameterRediscretize();
	}
	return true;
}

void FeatureTrajectoryPageWidget::onSaveDiscretizeTemplateClicked()
{
	if (!m_strategyCombo || !m_paramPanel)
	{
		return;
	}
	const QString strategyId = m_strategyCombo->currentData().toString();
	if (strategyId.isEmpty())
	{
		QMessageBox::information(this, m_chinese ? QStringLiteral("保存") : QStringLiteral("Save"),
								 m_chinese ? QStringLiteral("请先选择离散策略") : QStringLiteral("Select a strategy"));
		return;
	}
	bool ok = false;
	const QString name = QInputDialog::getText(
		this, m_chinese ? QStringLiteral("保存离散模板") : QStringLiteral("Save Discretize Template"),
		m_chinese ? QStringLiteral("模板名称") : QStringLiteral("Template name"), QLineEdit::Normal,
		m_discretizeTemplateCombo && m_discretizeTemplateCombo->currentIndex() > 0
			? m_discretizeTemplateCombo->currentText()
			: QString(),
		&ok);
	if (!ok || name.trimmed().isEmpty())
	{
		return;
	}
	nlohmann::json params = nlohmann::json::object();
	(void)m_paramPanel->applyParams(params);
	nlohmann::json payload =
		nlohmann::json::object({{"strategyId", strategyId.toStdString()}, {"params", std::move(params)}});
	QString err;
	QString id;
	if (!UserTemplateLibrary::save(UserTemplateKind::Discretize, name.trimmed(), payload, &id, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("保存") : QStringLiteral("Save"), err);
		return;
	}
	refreshDiscretizeTemplateCombo();
	if (m_discretizeTemplateCombo)
	{
		const int idx = m_discretizeTemplateCombo->findData(id);
		if (idx >= 0)
		{
			m_discretizeTemplateCombo->setCurrentIndex(idx);
		}
	}
}

void FeatureTrajectoryPageWidget::onLoadDiscretizeTemplateClicked()
{
	if (!m_discretizeTemplateCombo || m_discretizeTemplateCombo->currentIndex() <= 0)
	{
		QMessageBox::information(this, m_chinese ? QStringLiteral("加载") : QStringLiteral("Load"),
								 m_chinese ? QStringLiteral("请先选择模板") : QStringLiteral("Select a template"));
		return;
	}
	nlohmann::json payload;
	QString err;
	if (!UserTemplateLibrary::load(UserTemplateKind::Discretize, m_discretizeTemplateCombo->currentData().toString(),
								   &payload, nullptr, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("加载") : QStringLiteral("Load"), err);
		return;
	}
	if (!applyDiscretizeTemplatePayload(payload, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("加载") : QStringLiteral("Load"), err);
	}
}

void FeatureTrajectoryPageWidget::onDeleteDiscretizeTemplateClicked()
{
	if (!m_discretizeTemplateCombo || m_discretizeTemplateCombo->currentIndex() <= 0)
	{
		return;
	}
	const QString id = m_discretizeTemplateCombo->currentData().toString();
	const QString name = m_discretizeTemplateCombo->currentText();
	const auto ret = QMessageBox::question(
		this, m_chinese ? QStringLiteral("删除模板") : QStringLiteral("Delete Template"),
		m_chinese ? QStringLiteral("删除「%1」？").arg(name) : QStringLiteral("Delete \"%1\"?").arg(name));
	if (ret != QMessageBox::Yes)
	{
		return;
	}
	QString err;
	if (!UserTemplateLibrary::remove(UserTemplateKind::Discretize, id, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("删除") : QStringLiteral("Delete"), err);
		return;
	}
	refreshDiscretizeTemplateCombo();
}

void FeatureTrajectoryPageWidget::onImportDiscretizeTemplateClicked()
{
	const QString path = QFileDialog::getOpenFileName(
		this, m_chinese ? QStringLiteral("导入离散模板") : QStringLiteral("Import Discretize Template"), QString(),
		QStringLiteral("JSON (*.json)"));
	if (path.isEmpty())
	{
		return;
	}
	QString err;
	QString id;
	if (!UserTemplateLibrary::importFile(UserTemplateKind::Discretize, path, &id, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("导入") : QStringLiteral("Import"), err);
		return;
	}
	refreshDiscretizeTemplateCombo();
	if (m_discretizeTemplateCombo)
	{
		const int idx = m_discretizeTemplateCombo->findData(id);
		if (idx >= 0)
		{
			m_discretizeTemplateCombo->setCurrentIndex(idx);
		}
	}
}

void FeatureTrajectoryPageWidget::onExportDiscretizeTemplateClicked()
{
	if (!m_discretizeTemplateCombo || m_discretizeTemplateCombo->currentIndex() <= 0)
	{
		QMessageBox::information(this, m_chinese ? QStringLiteral("导出") : QStringLiteral("Export"),
								 m_chinese ? QStringLiteral("请先选择模板") : QStringLiteral("Select a template"));
		return;
	}
	const QString id = m_discretizeTemplateCombo->currentData().toString();
	const QString suggested = m_discretizeTemplateCombo->currentText() + QStringLiteral(".json");
	const QString path = QFileDialog::getSaveFileName(
		this, m_chinese ? QStringLiteral("导出离散模板") : QStringLiteral("Export Discretize Template"), suggested,
		QStringLiteral("JSON (*.json)"));
	if (path.isEmpty())
	{
		return;
	}
	QString err;
	if (!UserTemplateLibrary::exportFile(UserTemplateKind::Discretize, id, path, &err))
	{
		QMessageBox::warning(this, m_chinese ? QStringLiteral("导出") : QStringLiteral("Export"), err);
	}
}
