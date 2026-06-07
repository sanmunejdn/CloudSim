#include "FeatureTrajectoryPageWidget.h"

#include "FeaturePickTransform.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RecipeBlueprint.h"
#include "RobotOsgUiTypes.h"
#include "RobotSimulationController.h"
#include "RobotSimulationDockWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "TrajectoryEditSession.h"
#include "BackendDataManager.h"
#include "FeatureSpec.h"
#include "BrepBackendData.h"
#include "GeometryRef.h"
#include <ShapeHandle.h>
#include "PickTypes.h"
#include "RawTrajectory.h"

#include "UiIconDecorators.h"

#include <json.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHash>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTimer>
#include <QVector>
#include <QVBoxLayout>

namespace
{

constexpr double kDefaultStepMm = 2.0;
constexpr double kDefaultLinearDeflectionMm = 0.01;
constexpr int kDefaultUvCount = 16;

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

} // namespace

FeatureTrajectoryPageWidget::FeatureTrajectoryPageWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->addWidget(new QLabel(QStringLiteral("工件 backend")));
	m_backendCombo = new QComboBox(this);
	layout->addWidget(m_backendCombo);

	auto* pickRow = new QHBoxLayout;
	m_pickEdgeBtn = new QPushButton(QStringLiteral("拾取边"), this);
	m_pickFaceBtn = new QPushButton(QStringLiteral("拾取面"), this);
	m_cancelPickBtn = new QPushButton(QStringLiteral("取消拾取"), this);
	pickRow->addWidget(m_pickEdgeBtn);
	pickRow->addWidget(m_pickFaceBtn);
	pickRow->addWidget(m_cancelPickBtn);
	layout->addLayout(pickRow);

	m_faceKindCombo = new QComboBox(this);
	m_faceKindCombo->addItem(QStringLiteral("面外轮廓 (FaceBoundary)"), QStringLiteral("FaceBoundary"));
	m_faceKindCombo->addItem(QStringLiteral("面内网格 (FaceUVGrid)"), QStringLiteral("FaceUVGrid"));
	layout->addWidget(m_faceKindCombo);

	m_pickStatusLabel = new QLabel(this);
	layout->addWidget(m_pickStatusLabel);

	m_activeFeatureLabel = new QLabel(this);
	m_activeFeatureLabel->setWordWrap(true);
	layout->addWidget(m_activeFeatureLabel);

	m_discretizeGroup = new QGroupBox(QStringLiteral("离散参数"), this);
	auto* discLayout = new QVBoxLayout(m_discretizeGroup);
	m_discretizeStack = new QStackedWidget(m_discretizeGroup);

	auto* linePage = new QWidget(m_discretizeStack);
	auto* lineForm = new QFormLayout(linePage);
	m_stepMmSpin = new QDoubleSpinBox(linePage);
	m_stepMmSpin->setRange(0.1, 500.0);
	m_stepMmSpin->setDecimals(2);
	m_stepMmSpin->setSingleStep(0.5);
	m_stepMmSpin->setValue(kDefaultStepMm);
	lineForm->addRow(QStringLiteral("步距 (mm)"), m_stepMmSpin);

	m_linearDeflectionSpin = new QDoubleSpinBox(linePage);
	m_linearDeflectionSpin->setRange(0.001, 10.0);
	m_linearDeflectionSpin->setDecimals(3);
	m_linearDeflectionSpin->setSingleStep(0.001);
	m_linearDeflectionSpin->setValue(kDefaultLinearDeflectionMm);
	lineForm->addRow(QStringLiteral("曲线精度 (mm)"), m_linearDeflectionSpin);
	m_discretizeStack->addWidget(linePage);

	auto* gridPage = new QWidget(m_discretizeStack);
	auto* gridForm = new QFormLayout(gridPage);
	m_uvCountUSpin = new QSpinBox(gridPage);
	m_uvCountUSpin->setRange(2, 256);
	m_uvCountUSpin->setValue(kDefaultUvCount);
	gridForm->addRow(QStringLiteral("U 向点数"), m_uvCountUSpin);

	m_uvCountVSpin = new QSpinBox(gridPage);
	m_uvCountVSpin->setRange(2, 256);
	m_uvCountVSpin->setValue(kDefaultUvCount);
	gridForm->addRow(QStringLiteral("V 向点数"), m_uvCountVSpin);

	m_gridAngleSpin = new QDoubleSpinBox(gridPage);
	m_gridAngleSpin->setRange(-180.0, 180.0);
	m_gridAngleSpin->setDecimals(1);
	m_gridAngleSpin->setValue(0.0);
	gridForm->addRow(QStringLiteral("网格旋转 (°)"), m_gridAngleSpin);
	m_discretizeStack->addWidget(gridPage);

	discLayout->addWidget(m_discretizeStack);

	auto* previewRow = new QHBoxLayout;
	m_showAxesCheck = new QCheckBox(QStringLiteral("显示坐标轴"), m_discretizeGroup);
	m_showAxesCheck->setChecked(true);
	previewRow->addWidget(m_showAxesCheck);
	previewRow->addWidget(new QLabel(QStringLiteral("轴间隔"), m_discretizeGroup));
	m_axisIntervalSpin = new QSpinBox(m_discretizeGroup);
	m_axisIntervalSpin->setRange(0, 9999);
	m_axisIntervalSpin->setValue(0);
	m_axisIntervalSpin->setToolTip(QStringLiteral("0 = 自动 (约 n/20)"));
	previewRow->addWidget(m_axisIntervalSpin);
	previewRow->addStretch();
	discLayout->addLayout(previewRow);

	layout->addWidget(m_discretizeGroup);

	m_catalogBtn = new QPushButton(QStringLiteral("刷新特征目录"), this);
	layout->addWidget(m_catalogBtn);

	layout->addWidget(new QLabel(QStringLiteral("FeatureSpec JSON")));
	m_specEditor = new QPlainTextEdit(this);
	m_specEditor->setMinimumHeight(160);
	layout->addWidget(m_specEditor);

	m_discretizeBtn = new QPushButton(QStringLiteral("离散预览"), this);
	layout->addWidget(m_discretizeBtn);

	connect(m_catalogBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onLoadCatalog);
	connect(m_discretizeBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onDiscretize);
	connect(m_pickEdgeBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onPickEdge);
	connect(m_pickFaceBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onPickFace);
	connect(m_cancelPickBtn, &QPushButton::clicked, this, &FeatureTrajectoryPageWidget::onCancelPick);

	UiIconDecorators::apply(m_pickEdgeBtn, UiIconId::PickEdge);
	UiIconDecorators::apply(m_pickFaceBtn, UiIconId::PickFace);
	UiIconDecorators::apply(m_catalogBtn, UiIconId::Refresh);
	UiIconDecorators::apply(m_discretizeBtn, UiIconId::Discretize);

	connect(m_faceKindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
		updateDiscretizeParamMode();
	});
	connect(m_specEditor, &QPlainTextEdit::textChanged, this, [this]() {
		syncDiscretizeUiFromSpecJson();
		if (editorHasValidFeatureSpec())
		{
			commitLastFeatureSpec(m_specEditor->toPlainText().toStdString());
		}
	});

	m_rediscretizeTimer = new QTimer(this);
	m_rediscretizeTimer->setSingleShot(true);
	m_rediscretizeTimer->setInterval(400);
	connect(m_rediscretizeTimer, &QTimer::timeout, this, &FeatureTrajectoryPageWidget::onParameterRediscretize);

	const auto paramChanged = [this]() { scheduleParameterRediscretize(); };
	connect(m_stepMmSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, paramChanged);
	connect(m_linearDeflectionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, paramChanged);
	connect(m_uvCountUSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, paramChanged);
	connect(m_uvCountVSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, paramChanged);
	connect(m_gridAngleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, paramChanged);
	connect(m_showAxesCheck, &QCheckBox::toggled, this, [this]() { refreshPreviewFromSession(); });
	connect(m_axisIntervalSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() { refreshPreviewFromSession(); });

	setUseChinese(m_chinese);
	updateDiscretizeParamMode();
	updateActiveFeatureLabel();

	connect(m_backendCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
		clearCandidatePreview();
		m_cachedCatalogBackendId.clear();
		m_cachedCatalogJsonUtf8.clear();
		if (m_backendCombo && m_backendCombo->currentIndex() >= 0)
		{
			(void)autoEnumerateCatalogForCurrentWorkpiece(true, true, nullptr);
		}
		emit workpieceComboChanged();
	});
}

void FeatureTrajectoryPageWidget::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
	updateUiLabels();
}

void FeatureTrajectoryPageWidget::updateUiLabels()
{
	const bool zh = m_chinese;
	if (m_catalogBtn)
	{
		m_catalogBtn->setText(zh ? QStringLiteral("刷新特征目录") : QStringLiteral("Refresh catalog"));
	}
	if (m_discretizeBtn)
	{
		m_discretizeBtn->setText(zh ? QStringLiteral("离散预览") : QStringLiteral("Discretize preview"));
	}
	if (m_pickEdgeBtn)
	{
		m_pickEdgeBtn->setText(zh ? QStringLiteral("拾取边") : QStringLiteral("Pick edge"));
	}
	if (m_pickFaceBtn)
	{
		m_pickFaceBtn->setText(zh ? QStringLiteral("拾取面") : QStringLiteral("Pick face"));
	}
	if (m_cancelPickBtn)
	{
		m_cancelPickBtn->setText(zh ? QStringLiteral("取消拾取") : QStringLiteral("Cancel pick"));
	}
	if (m_discretizeGroup)
	{
		m_discretizeGroup->setTitle(zh ? QStringLiteral("离散参数") : QStringLiteral("Discretize params"));
	}
	if (m_showAxesCheck)
	{
		m_showAxesCheck->setText(zh ? QStringLiteral("显示坐标轴") : QStringLiteral("Show axes"));
	}
	updatePickUiState();
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
		m_host->setMeshPickCommittedHandler([this](const PickResult& pick, const PickKind kind) {
			onMeshPickCommitted(pick, static_cast<int>(kind));
		});
	}
	refreshBackendCombo();
}

void FeatureTrajectoryPageWidget::bindSession(TrajectoryEditSession* session)
{
	m_session = session;
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

bool FeatureTrajectoryPageWidget::editorHasValidFeatureSpec() const
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(m_specEditor->toPlainText().toStdString());
		return j.contains("kind") && j["kind"].is_string();
	}
	catch (...)
	{
		return false;
	}
}

bool FeatureTrajectoryPageWidget::resolveDiscretizeBaseJson(std::string& outJson) const
{
	if (editorHasValidFeatureSpec())
	{
		outJson = m_specEditor->toPlainText().toStdString();
		return true;
	}
	if (m_hasLastFeatureSpec && !m_lastFeatureSpecJson.empty())
	{
		outJson = m_lastFeatureSpecJson;
		return true;
	}
	return false;
}

void FeatureTrajectoryPageWidget::commitLastFeatureSpec(const std::string& jsonText)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonText);
		if (!j.contains("kind") || !j["kind"].is_string())
		{
			return;
		}
	}
	catch (...)
	{
		return;
	}
	m_lastFeatureSpecJson = jsonText;
	m_hasLastFeatureSpec = true;
	updateActiveFeatureLabel();
}

void FeatureTrajectoryPageWidget::updateActiveFeatureLabel()
{
	if (!m_activeFeatureLabel)
	{
		return;
	}
	if (!m_hasLastFeatureSpec)
	{
		m_activeFeatureLabel->setText(m_chinese ? QStringLiteral("当前特征：未选择（请 3D 拾取或编辑 FeatureSpec）")
			: QStringLiteral("Active feature: none (pick in 3D or edit FeatureSpec)"));
		return;
	}
	try
	{
		const nlohmann::json j = nlohmann::json::parse(m_lastFeatureSpecJson);
		const std::string featureId = j.value("featureId", "");
		const std::string kind = j.value("kind", "");
		const QString idText = featureId.empty() ? QStringLiteral("—") : QString::fromStdString(featureId);
		m_activeFeatureLabel->setText(m_chinese
			? QStringLiteral("当前特征：%1（%2）— 参数调整将自动重新离散")
				.arg(idText, QString::fromStdString(kind))
			: QStringLiteral("Active feature: %1 (%2) — param changes re-discretize automatically")
				.arg(idText, QString::fromStdString(kind)));
	}
	catch (...)
	{
		m_activeFeatureLabel->clear();
	}
}

void FeatureTrajectoryPageWidget::scheduleParameterRediscretize()
{
	if (m_suppressParamRediscretize || !m_hasLastFeatureSpec || !m_rediscretizeTimer)
	{
		return;
	}
	m_rediscretizeTimer->start();
}

void FeatureTrajectoryPageWidget::onParameterRediscretize()
{
	if (!m_hasLastFeatureSpec)
	{
		return;
	}
	m_suppressParamRediscretize = true;
	(void)discretizeFromEditor();
	m_suppressParamRediscretize = false;
}

void FeatureTrajectoryPageWidget::refreshPreviewFromSession()
{
	if (!m_session || !m_session->hasRawTrajectory())
	{
		return;
	}
	const RobotInstruction::RawTrajectory* traj = m_session->rawTrajectory();
	if (traj)
	{
		showTrajectoryPreview(*traj);
	}
}

bool FeatureTrajectoryPageWidget::isFaceUvGridKind() const
{
	std::string baseJson;
	if (resolveDiscretizeBaseJson(baseJson))
	{
		try
		{
			const nlohmann::json j = nlohmann::json::parse(baseJson);
			if (j.contains("kind") && j["kind"].is_string())
			{
				return j["kind"].get<std::string>() == "FaceUVGrid";
			}
		}
		catch (...)
		{
		}
	}
	if (m_faceKindCombo)
	{
		return m_faceKindCombo->currentData().toString() == QStringLiteral("FaceUVGrid");
	}
	return false;
}

void FeatureTrajectoryPageWidget::updateDiscretizeParamMode()
{
	if (!m_discretizeStack)
	{
		return;
	}
	m_discretizeStack->setCurrentIndex(isFaceUvGridKind() ? 1 : 0);
}

void FeatureTrajectoryPageWidget::syncDiscretizeUiFromSpecJson()
{
	std::string baseJson;
	if (!resolveDiscretizeBaseJson(baseJson))
	{
		updateDiscretizeParamMode();
		return;
	}
	try
	{
		const nlohmann::json j = nlohmann::json::parse(baseJson);
		if (j.contains("discretize") && j["discretize"].is_object())
		{
			const auto& d = j["discretize"];
			if (m_stepMmSpin && d.contains("stepMm"))
			{
				m_stepMmSpin->blockSignals(true);
				m_stepMmSpin->setValue(d["stepMm"].get<double>());
				m_stepMmSpin->blockSignals(false);
			}
			if (m_linearDeflectionSpin && d.contains("linearDeflectionMm"))
			{
				m_linearDeflectionSpin->blockSignals(true);
				m_linearDeflectionSpin->setValue(d["linearDeflectionMm"].get<double>());
				m_linearDeflectionSpin->blockSignals(false);
			}
		}
		if (j.contains("refs") && j["refs"].is_object())
		{
			const auto& r = j["refs"];
			if (m_uvCountUSpin && r.contains("uvCountU"))
			{
				m_uvCountUSpin->blockSignals(true);
				m_uvCountUSpin->setValue(r["uvCountU"].get<int>());
				m_uvCountUSpin->blockSignals(false);
			}
			if (m_uvCountVSpin && r.contains("uvCountV"))
			{
				m_uvCountVSpin->blockSignals(true);
				m_uvCountVSpin->setValue(r["uvCountV"].get<int>());
				m_uvCountVSpin->blockSignals(false);
			}
			if (m_gridAngleSpin && r.contains("gridAngleDeg"))
			{
				m_gridAngleSpin->blockSignals(true);
				m_gridAngleSpin->setValue(r["gridAngleDeg"].get<double>());
				m_gridAngleSpin->blockSignals(false);
			}
		}
		if (j.contains("kind") && j["kind"].is_string() && m_faceKindCombo)
		{
			const std::string kind = j["kind"].get<std::string>();
			const int idx = m_faceKindCombo->findData(QString::fromStdString(kind));
			if (idx >= 0)
			{
				m_faceKindCombo->blockSignals(true);
				m_faceKindCombo->setCurrentIndex(idx);
				m_faceKindCombo->blockSignals(false);
			}
		}
	}
	catch (...)
	{
	}
	updateDiscretizeParamMode();
}

bool FeatureTrajectoryPageWidget::applyDiscretizeUiToJson(std::string& jsonText)
{
	try
	{
		nlohmann::json j = nlohmann::json::parse(jsonText);
		if (!j.contains("discretize") || !j["discretize"].is_object())
		{
			j["discretize"] = nlohmann::json::object();
		}
		if (!j.contains("refs") || !j["refs"].is_object())
		{
			j["refs"] = nlohmann::json::object();
		}
		if (isFaceUvGridKind())
		{
			j["refs"]["uvCountU"] = m_uvCountUSpin ? m_uvCountUSpin->value() : kDefaultUvCount;
			j["refs"]["uvCountV"] = m_uvCountVSpin ? m_uvCountVSpin->value() : kDefaultUvCount;
			j["refs"]["gridAngleDeg"] = m_gridAngleSpin ? m_gridAngleSpin->value() : 0.0;
		}
		else
		{
			j["discretize"]["stepMm"] = m_stepMmSpin ? m_stepMmSpin->value() : kDefaultStepMm;
			j["discretize"]["linearDeflectionMm"] =
				m_linearDeflectionSpin ? m_linearDeflectionSpin->value() : kDefaultLinearDeflectionMm;
		}
		jsonText = j.dump(2);
		return true;
	}
	catch (const std::exception& ex)
	{
		QMessageBox::warning(this, QStringLiteral("离散参数"), QString::fromStdString(ex.what()));
		return false;
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
	if (m_faceKindCombo)
	{
		m_faceKindCombo->setEnabled(m_pickSession == PickSessionKind::None);
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
		m_pickStatusLabel->setText(m_chinese ? QStringLiteral("3D 拾取未激活")
			: QStringLiteral("3D pick inactive"));
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
	updatePickUiState();
	setStatus(m_chinese ? QStringLiteral("面拾取模式：在视口左键点击确认")
		: QStringLiteral("Face pick: left-click in viewport"));
}

void FeatureTrajectoryPageWidget::onCancelPick()
{
	exitPickMode();
	setStatus(m_chinese ? QStringLiteral("已取消 3D 拾取") : QStringLiteral("3D pick cancelled"));
}

void FeatureTrajectoryPageWidget::onMeshPickCommitted(const PickResult& pick, const int pickKindInt)
{
	if (m_pickSession == PickSessionKind::None || !m_host || !pick.hit)
	{
		return;
	}
	const PickKind kind = static_cast<PickKind>(pickKindInt);
	if ((m_pickSession == PickSessionKind::Edge && kind != PickKind::MeshEdge)
		|| (m_pickSession == PickSessionKind::Face && kind != PickKind::MeshFace))
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
			m_chinese ? QStringLiteral("拾取 backend 与当前工件不一致") : QStringLiteral("Picked backend mismatch"));
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
		if (!feature_pick_transform::worldPointToStepModelMm(osg, pick.backendId, pick.meshEdgeA, modelA, &xformErr)
			|| !feature_pick_transform::worldPointToStepModelMm(osg, pick.backendId, pick.meshEdgeB, modelB, &xformErr))
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

	geoalgo::WorkpieceRef wp;
	QString pickErr;
	std::string err;
	geoalgo::FeatureKind faceKind = geoalgo::FeatureKind::FaceBoundary;
	if (kind == PickKind::MeshFace && m_faceKindCombo)
	{
		const QString fk = m_faceKindCombo->currentData().toString();
		if (fk == QStringLiteral("FaceUVGrid"))
		{
			faceKind = geoalgo::FeatureKind::FaceUVGrid;
		}
	}

	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wpResolved;
	if (!resolveWorkpieceShapeForBackend(expectedBackendId, shape, wpResolved, &pickErr))
	{
		QMessageBox::warning(this, QStringLiteral("Pick"), pickErr);
		exitPickMode();
		return;
	}
	wp = wpResolved;

	geoalgo::FeatureSpec spec;
	const int knownFaceIndex = pick.brepNativePick && kind == PickKind::MeshFace ? pick.brepFaceIndex : -1;
	const int knownEdgeIndex = pick.brepNativePick && kind == PickKind::MeshEdge ? pick.brepEdgeIndex : -1;
	if (!geometry_backend_ops::buildFeatureSpecFromModelPick(
			wp, shape, kind == PickKind::MeshFace, faceKind, modelA, modelB, spec, &err,
			knownFaceIndex, knownEdgeIndex))
	{
		QMessageBox::warning(this, QStringLiteral("Pick"), QString::fromStdString(err));
		exitPickMode();
		return;
	}

	if (faceKind == geoalgo::FeatureKind::FaceUVGrid)
	{
		spec.refs.uvCountU = m_uvCountUSpin ? m_uvCountUSpin->value() : kDefaultUvCount;
		spec.refs.uvCountV = m_uvCountVSpin ? m_uvCountVSpin->value() : kDefaultUvCount;
		spec.refs.gridAngleDeg = m_gridAngleSpin ? m_gridAngleSpin->value() : 0.0;
	}
	else
	{
		spec.discretize.stepMm = m_stepMmSpin ? m_stepMmSpin->value() : kDefaultStepMm;
		spec.discretize.linearDeflectionMm =
			m_linearDeflectionSpin ? m_linearDeflectionSpin->value() : kDefaultLinearDeflectionMm;
	}

	std::string specJson = geometry_backend_ops::featureSpecToJson(spec);
	(void)applyDiscretizeUiToJson(specJson);
	m_specEditor->blockSignals(true);
	m_specEditor->setPlainText(QString::fromStdString(specJson));
	m_specEditor->blockSignals(false);
	commitLastFeatureSpec(specJson);
	syncDiscretizeUiFromSpecJson();
	exitPickMode();
	setStatus(m_chinese
		? QStringLiteral("已选取 %1，正在离散…").arg(QString::fromStdString(spec.featureId))
		: QStringLiteral("Selected %1, discretizing…").arg(QString::fromStdString(spec.featureId)));
	(void)discretizeFromEditor();
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
		const QString label = fileName.isEmpty()
			? backendId
			: QStringLiteral("%1 (%2)").arg(backendId, fileName);
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
		setStatus(m_chinese ? QStringLiteral("请先导入 STEP 工件")
			: QStringLiteral("Import a STEP workpiece first"));
	}
	else if (m_backendCombo->currentIndex() >= 0)
	{
		QTimer::singleShot(0, this, [this]() {
			(void)autoEnumerateCatalogForCurrentWorkpiece(true, true, nullptr);
		});
	}
	updatePickUiState();
}

bool FeatureTrajectoryPageWidget::resolveWorkpieceShapeForBackend(const QString& backendId,
	geoalgo::ShapeHandle& outShape, geoalgo::WorkpieceRef& outRef, QString* err) const
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
	const auto src = geometry_backend_ops::resolveWorkpieceShape(
		backendId.toStdString(), doc->backend(), stepPath.toStdString(), outShape, outRef, &stdErr);
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

bool FeatureTrajectoryPageWidget::enumerateCatalogForBackend(const QString& backendId,
	geoalgo::FeatureCatalog& catalog, QString* err) const
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

bool FeatureTrajectoryPageWidget::shouldReplaceEditorWithCatalog() const
{
	if (!m_specEditor)
	{
		return false;
	}
	if (editorHasValidFeatureSpec())
	{
		return false;
	}
	const QString text = m_specEditor->toPlainText().trimmed();
	if (text.isEmpty())
	{
		return true;
	}
	try
	{
		const nlohmann::json j = nlohmann::json::parse(text.toStdString());
		if (j.contains("kind"))
		{
			return false;
		}
		if (j.contains("candidates"))
		{
			return true;
		}
	}
	catch (...)
	{
	}
	return false;
}

bool FeatureTrajectoryPageWidget::autoEnumerateCatalogForCurrentWorkpiece(const bool updateEditor,
	const bool quiet, QString* err)
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

	if (updateEditor && shouldReplaceEditorWithCatalog())
	{
		m_specEditor->blockSignals(true);
		m_specEditor->setPlainText(QString::fromStdString(m_cachedCatalogJsonUtf8));
		m_specEditor->blockSignals(false);
	}

	if (!quiet)
	{
		setStatus(m_chinese
			? QStringLiteral("特征目录已加载（%1 个候选）").arg(static_cast<int>(catalog.candidates.size()))
			: QStringLiteral("Feature catalog loaded (%1 candidates)").arg(static_cast<int>(catalog.candidates.size())));
	}
	return true;
}

bool FeatureTrajectoryPageWidget::ensureFeatureCatalogEnumerated(QString* err)
{
	return autoEnumerateCatalogForCurrentWorkpiece(true, true, err);
}

void FeatureTrajectoryPageWidget::setStatus(const QString& text)
{
	if (m_host)
	{
		m_host->appendRunInfo(text);
	}
}

void FeatureTrajectoryPageWidget::onLoadCatalog()
{
	QString err;
	m_cachedCatalogBackendId.clear();
	m_cachedCatalogJsonUtf8.clear();
	if (!autoEnumerateCatalogForCurrentWorkpiece(true, false, &err))
	{
		if (!err.isEmpty())
		{
			QMessageBox::warning(this, QStringLiteral("Catalog"), err);
		}
		return;
	}
	if (m_chinese)
	{
		setStatus(QStringLiteral("目录已刷新，可复制 candidate 填入 FeatureSpec 或使用 3D 拾取"));
	}
	else
	{
		setStatus(QStringLiteral("Catalog refreshed; copy a candidate into FeatureSpec or use 3D pick"));
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
	opt.showAxes = m_showAxesCheck && m_showAxesCheck->isChecked();
	opt.axisInterval = m_axisIntervalSpin ? m_axisIntervalSpin->value() : 0;
	opt.maxAxes = 50;
	return opt;
}

void FeatureTrajectoryPageWidget::showTrajectoryPreview(const RobotInstruction::RawTrajectory& traj)
{
	if (!m_host)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const std::string backendId = resolvePreviewBackendId(traj);
	if (backendId.empty())
	{
		m_host->appendRunWarning(m_chinese ? QStringLiteral("轨迹预览：FeatureSpec 缺少 workpiece.backendIdUtf8")
			: QStringLiteral("Trajectory preview: missing workpiece.backendIdUtf8"));
		return;
	}
	if (traj.points.empty())
	{
		return;
	}
	std::string err;
	feature_pick_transform::applyRawTrajectoryPreviewToOsg(
		osg, backendId, traj, currentPreviewOptions(), &err);
	if (!err.empty())
	{
		m_host->appendRunWarning(QString::fromStdString(err));
		if (m_simController)
		{
			m_simController->setRawTrajectoryPreviewActive(false);
		}
		return;
	}
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(true);
	}
	osg->requestRedraw();
}

bool FeatureTrajectoryPageWidget::prepareSpecForDiscretize(std::string& jsonText, std::string* errMsg)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonText);
		if (j.contains("candidates") && !j.contains("kind"))
		{
			if (errMsg)
			{
				*errMsg = m_chinese
					? "当前为特征目录 JSON，请从 candidate 组装 FeatureSpec 或使用 3D 拾取"
					: "Editor has FeatureCatalog JSON; build a FeatureSpec or use 3D pick";
			}
			return false;
		}
	}
	catch (...)
	{
	}

	if (m_backendCombo->currentIndex() >= 0)
	{
		const QString backendId = m_backendCombo->currentData().toString();
		QString stepPath;
		if (m_stepPathResolver)
		{
			stepPath = m_stepPathResolver(backendId);
		}
		try
		{
			nlohmann::json j = nlohmann::json::parse(jsonText);
			if (!j.contains("workpiece") || !j["workpiece"].is_object())
			{
				j["workpiece"] = nlohmann::json::object();
			}
			if (j["workpiece"].value("backendIdUtf8", "").empty())
			{
				j["workpiece"]["backendIdUtf8"] = backendId.toStdString();
			}
			if (j["workpiece"].value("stepPathUtf8", "").empty() && !stepPath.isEmpty())
			{
				j["workpiece"]["stepPathUtf8"] = stepPath.toStdString();
			}
			if (!j.contains("schemaVersion"))
			{
				j["schemaVersion"] = 1;
			}
			if (!j.contains("kind"))
			{
				j["kind"] = "EdgeChain";
			}
			jsonText = j.dump();
		}
		catch (...)
		{
		}
	}
	return true;
}

bool FeatureTrajectoryPageWidget::discretizeFromEditor()
{
	std::string jsonText;
	if (!resolveDiscretizeBaseJson(jsonText))
	{
		QMessageBox::warning(this, QStringLiteral("离散"),
			m_chinese ? QStringLiteral("请先 3D 拾取特征或编辑 FeatureSpec JSON")
				: QStringLiteral("Pick a feature in 3D or edit FeatureSpec JSON first"));
		return false;
	}
	std::string prepErr;
	if (!prepareSpecForDiscretize(jsonText, &prepErr))
	{
		QMessageBox::warning(this, QStringLiteral("离散"), QString::fromStdString(prepErr));
		return false;
	}
	if (!applyDiscretizeUiToJson(jsonText))
	{
		return false;
	}
	commitLastFeatureSpec(jsonText);
	if (editorHasValidFeatureSpec())
	{
		m_specEditor->blockSignals(true);
		m_specEditor->setPlainText(QString::fromStdString(jsonText));
		m_specEditor->blockSignals(false);
	}

	geoalgo::FeatureSpec spec;
	std::string err;
	if (!geometry_backend_ops::featureSpecFromJson(jsonText, spec, &err))
	{
		QMessageBox::warning(this, QStringLiteral("离散"), QString::fromStdString(err));
		return false;
	}

	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	QString shapeErr;
	const QString backendId = m_backendCombo ? m_backendCombo->currentData().toString() : QString();
	if (!resolveWorkpieceShapeForBackend(backendId, shape, wp, &shapeErr))
	{
		QMessageBox::warning(this, QStringLiteral("离散"), shapeErr);
		return false;
	}

	geoalgo::RawPath path;
	if (!geometry_backend_ops::discretizeFeature(spec, shape, path, &err))
	{
		QMessageBox::warning(this, QStringLiteral("离散"), QString::fromStdString(err));
		return false;
	}
	RobotInstruction::RawTrajectory traj;
	if (!RobotInstruction::importRawPathToTrajectory(path, RobotInstruction::FrameStrategy::SurfaceNormalZ, traj, &err))
	{
		QMessageBox::warning(this, QStringLiteral("导入"), QString::fromStdString(err));
		return false;
	}
	if (m_session)
	{
		m_session->setRawTrajectory(traj);
	}
	showTrajectoryPreview(traj);

	const int n = static_cast<int>(traj.points.size());
	QString msg;
	if (spec.kind == geoalgo::FeatureKind::FaceUVGrid)
	{
		msg = m_chinese
			? QStringLiteral("UV 网格 %1×%2 → 共 %3 点；请在「轨迹编辑」应用配方")
				.arg(spec.refs.uvCountU).arg(spec.refs.uvCountV).arg(n)
			: QStringLiteral("UV grid %1×%2 → %3 points; apply recipe on Trajectory Edit tab")
				.arg(spec.refs.uvCountU).arg(spec.refs.uvCountV).arg(n);
	}
	else
	{
		msg = m_chinese
			? QStringLiteral("步距 %1 mm → 共 %2 点；请在「轨迹编辑」应用配方")
				.arg(spec.discretize.stepMm, 0, 'f', 2).arg(n)
			: QStringLiteral("Step %1 mm → %2 points; apply recipe on Trajectory Edit tab")
				.arg(spec.discretize.stepMm, 0, 'f', 2).arg(n);
	}
	if (m_session)
	{
		if (!m_session->boundPathPlanId().empty())
		{
			msg += m_chinese
				? QStringLiteral("；已写入当前选中的路径规划")
				: QStringLiteral("; saved to selected path plan");
		}
		else
		{
			msg += m_chinese ? QStringLiteral("；已新建路径规划") : QStringLiteral("; new path plan created");
		}
	}
	setStatus(msg);
	return true;
}

void FeatureTrajectoryPageWidget::onDiscretize()
{
	(void)discretizeFromEditor();
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
			geoalgo::FeatureRefs refs;
			if (c.contains("refs") && c["refs"].is_object())
			{
				const auto& r = c["refs"];
				if (r.contains("edgeIndices") && r["edgeIndices"].is_array())
				{
					for (const auto& ei : r["edgeIndices"])
					{
						if (ei.is_number_integer())
						{
							refs.edgeIndices.push_back(ei.get<int>());
						}
					}
				}
				if (r.contains("faceIndices") && r["faceIndices"].is_array())
				{
					for (const auto& fi : r["faceIndices"])
					{
						if (fi.is_number_integer())
						{
							refs.faceIndices.push_back(fi.get<int>());
						}
					}
				}
			}
			geoalgo::FeatureAnchor anchor;
			std::string anchorErr;
			if (!geometry_backend_ops::computeFeatureAnchor(wp, refs, anchor, &anchorErr))
			{
				continue;
			}

			auto toWorld = [&](const double modelMm[3], nlohmann::json& outPt) {
				const geoalgo::Point3d modelPt{modelMm[0], modelMm[1], modelMm[2]};
				osg::Vec3f worldPt;
				std::string tfErr;
				if (!feature_pick_transform::stepModelPointToWorldMm(osg, backendId.toStdString(), modelPt, worldPt, &tfErr))
				{
					worldPt.set(static_cast<float>(modelPt.x), static_cast<float>(modelPt.y), static_cast<float>(modelPt.z));
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
			const auto readVec = [](const nlohmann::json& arr, osg::Vec3f& v) {
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
		if (!plan.contains("features") || !plan["features"].is_array() || plan["features"].empty())
		{
			if (err)
			{
				*err = QStringLiteral("特征计划缺少 features");
			}
			return false;
		}

		const nlohmann::json& first = plan["features"][0];
		const std::string specJson = first.dump();
		m_specEditor->blockSignals(true);
		m_specEditor->setPlainText(QString::fromStdString(specJson));
		m_specEditor->blockSignals(false);
		commitLastFeatureSpec(specJson);
		syncDiscretizeUiFromSpecJson();

		const std::string pipelineTemplate = plan.value("suggestedPipelineTemplate", std::string("weld_default"));
		if (!discretizeFromEditor())
		{
			if (err)
			{
				*err = QStringLiteral("离散失败，请检查 FeatureSpec");
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
