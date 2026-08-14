/// @file MeshTrajectoryPageWidget.cpp
/// @brief MeshTrajectoryPageWidget 实现

#include "MeshTrajectoryPageWidget.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "FeaturePickTransform.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "MeshTrajectoryIngress.h"
#include "MeshTriangleSelectionUtil.h"
#include "TrajectoryEditSession.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStatusBar>
#include <QVBoxLayout>
#include <memory>

#include <BackendDataManager.h>
#include <MeshBackendData.h>
#include <MeshSurfaceReconstruction.h>
#include <MeshTrajectory.h>
#include <MeshTrajectoryTypes.h>

namespace
{
bool isTopLevelWorkpieceBackend(const BackendDataManager& mgr, const std::string& backendId)
{
	return mgr.parentsOf(backendId).empty();
}

} // namespace

MeshTrajectoryPageWidget::MeshTrajectoryPageWidget(QWidget* parent)
	: QWidget(parent), m_meshSession(std::make_unique<MeshTrajectorySession>())
{
	auto* layout = new QVBoxLayout(this);

	m_backendCombo = new QComboBox(this);
	layout->addWidget(m_backendCombo);

	m_methodCombo = new QComboBox(this);
	layout->addWidget(m_methodCombo);

	m_selGroup = new QGroupBox(this);
	auto* selLayout = new QHBoxLayout(m_selGroup);
	m_pickClickBtn = new QPushButton(m_selGroup);
	m_pickBrushBtn = new QPushButton(m_selGroup);
	m_pickPolylineBtn = new QPushButton(m_selGroup);
	m_cancelPickBtn = new QPushButton(m_selGroup);
	m_clearSelBtn = new QPushButton(m_selGroup);
	m_invertSelBtn = new QPushButton(m_selGroup);
	selLayout->addWidget(m_pickClickBtn);
	selLayout->addWidget(m_pickBrushBtn);
	selLayout->addWidget(m_pickPolylineBtn);
	selLayout->addWidget(m_cancelPickBtn);
	selLayout->addWidget(m_clearSelBtn);
	selLayout->addWidget(m_invertSelBtn);
	layout->addWidget(m_selGroup);

	m_selectionLabel = new QLabel(this);
	layout->addWidget(m_selectionLabel);

	m_methodStack = new QStackedWidget(this);
	layout->addWidget(m_methodStack);

	m_crossPage = new QWidget(m_methodStack);
	auto* crossForm = new QFormLayout(m_crossPage);
	m_planeOx = new QDoubleSpinBox(m_crossPage);
	m_planeOy = new QDoubleSpinBox(m_crossPage);
	m_planeOz = new QDoubleSpinBox(m_crossPage);
	m_planeNx = new QDoubleSpinBox(m_crossPage);
	m_planeNy = new QDoubleSpinBox(m_crossPage);
	m_planeNz = new QDoubleSpinBox(m_crossPage);
	for (QDoubleSpinBox* spin : {m_planeOx, m_planeOy, m_planeOz, m_planeNx, m_planeNy, m_planeNz})
	{
		spin->setRange(-1e6, 1e6);
		spin->setDecimals(3);
	}
	m_planeNz->setValue(1.0);
	m_showSectionCheck = new QCheckBox(m_crossPage);
	m_showSectionCheck->setChecked(false);
	m_fromCameraNormalBtn = new QPushButton(m_crossPage);
	m_editSectionBtn = new QPushButton(m_crossPage);
	crossForm->addRow(QStringLiteral("Origin X"), m_planeOx);
	crossForm->addRow(QStringLiteral("Origin Y"), m_planeOy);
	crossForm->addRow(QStringLiteral("Origin Z"), m_planeOz);
	crossForm->addRow(QStringLiteral("Normal X"), m_planeNx);
	crossForm->addRow(QStringLiteral("Normal Y"), m_planeNy);
	crossForm->addRow(QStringLiteral("Normal Z"), m_planeNz);
	crossForm->addRow(m_showSectionCheck);
	crossForm->addRow(m_fromCameraNormalBtn);
	crossForm->addRow(m_editSectionBtn);

	m_bsplinePage = new QWidget(m_methodStack);
	auto* bsplineForm = new QFormLayout(m_bsplinePage);
	m_uvCountU = new QSpinBox(m_bsplinePage);
	m_uvCountV = new QSpinBox(m_bsplinePage);
	m_gridAngleDeg = new QDoubleSpinBox(m_bsplinePage);
	m_fitUvSpacingMm = new QDoubleSpinBox(m_bsplinePage);
	m_nurbsFitModeCombo = new QComboBox(m_bsplinePage);
	m_traceModeCombo = new QComboBox(m_bsplinePage);
	m_uvCountU->setRange(4, 256);
	m_uvCountV->setRange(4, 256);
	m_uvCountU->setValue(16);
	m_uvCountV->setValue(16);
	m_gridAngleDeg->setRange(-180.0, 180.0);
	m_fitUvSpacingMm->setRange(0.0, 1000.0);
	m_fitUvSpacingMm->setDecimals(2);
	m_fitUvSpacingMm->setSpecialValueText(QStringLiteral("—"));
	bsplineForm->addRow(QStringLiteral("U"), m_uvCountU);
	bsplineForm->addRow(QStringLiteral("V"), m_uvCountV);
	bsplineForm->addRow(QStringLiteral("Grid angle"), m_gridAngleDeg);
	bsplineForm->addRow(QStringLiteral("UV spacing"), m_fitUvSpacingMm);
	bsplineForm->addRow(QStringLiteral("NURBS fit"), m_nurbsFitModeCombo);
	bsplineForm->addRow(QStringLiteral("Trace"), m_traceModeCombo);

	m_methodStack->addWidget(m_crossPage);
	m_methodStack->addWidget(m_bsplinePage);

	m_crossDiscGroup = new QGroupBox(this);
	auto* crossDiscForm = new QFormLayout(m_crossDiscGroup);
	m_stepMm = new QDoubleSpinBox(m_crossDiscGroup);
	m_stepMm->setRange(0.01, 1000.0);
	m_stepMm->setValue(2.0);
	m_outputNormalCheck = new QCheckBox(m_crossDiscGroup);
	m_outputTangentCheck = new QCheckBox(m_crossDiscGroup);
	m_outputNormalCheck->setChecked(true);
	m_outputTangentCheck->setChecked(true);
	crossDiscForm->addRow(QStringLiteral("Step mm"), m_stepMm);
	crossDiscForm->addRow(m_outputNormalCheck);
	crossDiscForm->addRow(m_outputTangentCheck);
	layout->addWidget(m_crossDiscGroup);

	m_bsplineDiscGroup = new QGroupBox(this);
	auto* bsplineDiscForm = new QFormLayout(m_bsplineDiscGroup);
	m_bsplineOutputNormalCheck = new QCheckBox(m_bsplineDiscGroup);
	m_bsplineOutputTangentCheck = new QCheckBox(m_bsplineDiscGroup);
	m_bsplineOutputNormalCheck->setChecked(true);
	m_bsplineOutputTangentCheck->setChecked(true);
	bsplineDiscForm->addRow(m_bsplineOutputNormalCheck);
	bsplineDiscForm->addRow(m_bsplineOutputTangentCheck);
	m_bsplineDiscGroup->hide();
	layout->addWidget(m_bsplineDiscGroup);

	m_generateBtn = new QPushButton(this);
	layout->addWidget(m_generateBtn);
	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	layout->addWidget(m_statusLabel);
	layout->addStretch();

	m_methodCombo->addItem(QString(), static_cast<int>(geoalgo::MeshTrajectoryMethod::CrossSection));
	m_methodCombo->addItem(QString(), static_cast<int>(geoalgo::MeshTrajectoryMethod::BsplineRegion));
	m_traceModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshTrajectoryUvTraceMode::USerpentine));
	m_traceModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshTrajectoryUvTraceMode::VSerpentine));
	m_traceModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshTrajectoryUvTraceMode::UvGrid));
	m_nurbsFitModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshSurfaceNurbsFitMode::ApproxFixedCtrlpts));
	m_nurbsFitModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshSurfaceNurbsFitMode::ApproxCentripetal));
	m_nurbsFitModeCombo->addItem(QString(),
								 static_cast<int>(geoalgo::MeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts));
	m_nurbsFitModeCombo->addItem(QString(), static_cast<int>(geoalgo::MeshSurfaceNurbsFitMode::Interpolate));
	m_nurbsFitModeCombo->setCurrentIndex(2);

	connect(m_backendCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&MeshTrajectoryPageWidget::onBackendChanged);
	connect(m_methodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&MeshTrajectoryPageWidget::onMethodChanged);
	connect(m_generateBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onGenerateClicked);
	connect(m_clearSelBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onClearSelectionClicked);
	connect(m_invertSelBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onInvertSelectionClicked);
	connect(m_pickClickBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onPickClickClicked);
	connect(m_pickBrushBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onPickBrushClicked);
	connect(m_pickPolylineBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onPickPolylineClicked);
	connect(m_cancelPickBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onCancelPickClicked);
	connect(m_fromCameraNormalBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onFromCameraNormalClicked);
	connect(m_showSectionCheck, &QCheckBox::toggled, this, &MeshTrajectoryPageWidget::onShowSectionToggled);
	connect(m_editSectionBtn, &QPushButton::clicked, this, &MeshTrajectoryPageWidget::onEditSectionClicked);
	for (QDoubleSpinBox* spin : {m_planeOx, m_planeOy, m_planeOz, m_planeNx, m_planeNy, m_planeNz})
	{
		connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&MeshTrajectoryPageWidget::onPlaneSpinChanged);
	}
	connect(m_uvCountU, QOverload<int>::of(&QSpinBox::valueChanged), this,
			&MeshTrajectoryPageWidget::onBsplineParamChanged);
	connect(m_uvCountV, QOverload<int>::of(&QSpinBox::valueChanged), this,
			&MeshTrajectoryPageWidget::onBsplineParamChanged);
	connect(m_gridAngleDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&MeshTrajectoryPageWidget::onBsplineParamChanged);
	connect(m_fitUvSpacingMm, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&MeshTrajectoryPageWidget::onBsplineParamChanged);
	connect(m_nurbsFitModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&MeshTrajectoryPageWidget::onBsplineParamChanged);

	setUseChinese(true);
	onMethodChanged(0);
}

MeshTrajectoryPageWidget::~MeshTrajectoryPageWidget()
{
	// 退出阶段 DocumentPage/OSG 可能已析构，勿经 host 访问视图
	m_shuttingDown = true;
	m_sectionEditActive = false;
	m_host = nullptr;
}

void MeshTrajectoryPageWidget::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
	updateUiLabels();
}

void MeshTrajectoryPageWidget::updateUiLabels()
{
	const int methodIdx = m_methodCombo->currentIndex();
	if (methodIdx >= 0)
	{
		m_methodCombo->setItemText(0, m_chinese ? QStringLiteral("截面法") : QStringLiteral("Cross section"));
		m_methodCombo->setItemText(1, m_chinese ? QStringLiteral("B 样条曲面拟合")
												: QStringLiteral("B-spline surface fit"));
	}
	m_traceModeCombo->setItemText(0, m_chinese ? QStringLiteral("U 扫描") : QStringLiteral("U scan"));
	m_traceModeCombo->setItemText(1, m_chinese ? QStringLiteral("V 扫描") : QStringLiteral("V scan"));
	m_traceModeCombo->setItemText(2, m_chinese ? QStringLiteral("栅格点") : QStringLiteral("Grid points"));
	if (m_nurbsFitModeCombo)
	{
		const int fitIdx = m_nurbsFitModeCombo->currentIndex();
		m_nurbsFitModeCombo->setItemText(0,
										 m_chinese ? QStringLiteral("最小二乘+控制点") : QStringLiteral("LSQ+ctrlpts"));
		m_nurbsFitModeCombo->setItemText(1, m_chinese ? QStringLiteral("Centripetal") : QStringLiteral("Centripetal"));
		m_nurbsFitModeCombo->setItemText(2, m_chinese ? QStringLiteral("Centripetal+控制点")
													  : QStringLiteral("Centripetal+ctrlpts"));
		m_nurbsFitModeCombo->setItemText(3, m_chinese ? QStringLiteral("插值") : QStringLiteral("Interpolate"));
		m_nurbsFitModeCombo->setCurrentIndex(fitIdx);
	}
	m_pickClickBtn->setText(m_chinese ? QStringLiteral("点选") : QStringLiteral("Click"));
	m_pickBrushBtn->setText(m_chinese ? QStringLiteral("刷选") : QStringLiteral("Brush"));
	m_pickPolylineBtn->setText(m_chinese ? QStringLiteral("套索") : QStringLiteral("Lasso"));
	m_cancelPickBtn->setText(m_chinese ? QStringLiteral("取消拾取") : QStringLiteral("Cancel pick"));
	m_clearSelBtn->setText(m_chinese ? QStringLiteral("清除选择") : QStringLiteral("Clear"));
	m_invertSelBtn->setText(m_chinese ? QStringLiteral("反选") : QStringLiteral("Invert"));
	m_generateBtn->setText(m_chinese ? QStringLiteral("生成轨迹") : QStringLiteral("Generate trajectory"));
	m_fromCameraNormalBtn->setText(m_chinese ? QStringLiteral("相机法向") : QStringLiteral("Camera normal"));
	m_showSectionCheck->setText(m_chinese ? QStringLiteral("显示截面") : QStringLiteral("Show section"));
	m_editSectionBtn->setText(m_sectionEditActive
								  ? (m_chinese ? QStringLiteral("结束截面编辑") : QStringLiteral("End section edit"))
								  : (m_chinese ? QStringLiteral("编辑截面") : QStringLiteral("Edit section")));
	m_outputNormalCheck->setText(m_chinese ? QStringLiteral("输出法向") : QStringLiteral("Output normal"));
	m_outputTangentCheck->setText(m_chinese ? QStringLiteral("输出切向") : QStringLiteral("Output tangent"));
	m_bsplineOutputNormalCheck->setText(m_chinese ? QStringLiteral("输出法向") : QStringLiteral("Output normal"));
	m_bsplineOutputTangentCheck->setText(m_chinese ? QStringLiteral("输出切向") : QStringLiteral("Output tangent"));
	m_selGroup->setTitle(m_chinese ? QStringLiteral("区域选择") : QStringLiteral("Region selection"));
	m_crossDiscGroup->setTitle(m_chinese ? QStringLiteral("沿交线离散") : QStringLiteral("Along intersection"));
	m_bsplineDiscGroup->setTitle(m_chinese ? QStringLiteral("输出") : QStringLiteral("Output"));
	if (m_meshSession)
	{
		const auto summary = m_meshSession->summary();
		m_selectionLabel->setText(
			m_chinese
				? QStringLiteral("已选 %1 / %2 三角面").arg(summary.selectedTriangleCount).arg(summary.triangleCount)
				: QStringLiteral("Selected %1 / %2 triangles")
					  .arg(summary.selectedTriangleCount)
					  .arg(summary.triangleCount));
	}
}

void MeshTrajectoryPageWidget::applyMethodVisibility()
{
	const auto method = static_cast<geoalgo::MeshTrajectoryMethod>(m_methodCombo->currentData().toInt());
	const bool cross = method == geoalgo::MeshTrajectoryMethod::CrossSection;
	m_methodStack->setCurrentIndex(cross ? 0 : 1);
	m_selGroup->setVisible(!cross);
	m_selectionLabel->setVisible(!cross);
	m_crossDiscGroup->setVisible(cross);
	m_bsplineDiscGroup->setVisible(!cross);
	syncMethodPreview();
}

void MeshTrajectoryPageWidget::bindHost(IRobotMainWindowHost* host)
{
	hideSectionPlanePreview();
	if (m_host)
	{
		if (IRobotOsgViewHost* osg = m_host->osgView())
		{
			osg->clearMeshFittedSurfacePreview();
		}
		m_host->clearMeshTriangleLabelingPickHandlers();
	}
	m_host = host;
	wirePickHandlers();
	refreshBackendCombo();
}

void MeshTrajectoryPageWidget::bindSession(TrajectoryEditSession* session)
{
	m_session = session;
}

void MeshTrajectoryPageWidget::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
}

void MeshTrajectoryPageWidget::refreshWorkpieces()
{
	refreshBackendCombo();
}

void MeshTrajectoryPageWidget::wirePickHandlers()
{
	if (!m_host)
	{
		return;
	}
	IRobotMainWindowHost::MeshTriangleLabelingPickHandlers handlers;
	handlers.onClick = [this](const PickResult& pick)
	{
		if (!m_selGroup->isVisible())
		{
			return;
		}
		if (!pick.hit || pick.meshTriangleIndex < 0)
		{
			return;
		}
		applySelectionIndices({pick.meshTriangleIndex}, MeshTrajectorySelectionMode::Toggle);
		cancelActivePick();
	};
	handlers.onBrushStroke = [this](const std::vector<int>& indices)
	{
		if (!m_selGroup->isVisible())
		{
			return;
		}
		applySelectionIndices(indices, MeshTrajectorySelectionMode::Add);
	};
	handlers.onPolylineClosed =
		[this](const QVector<float>& polyline, const QVector<double>& mvp, const int vw, const int vh)
	{
		if (!m_selGroup->isVisible() || !m_host || m_backendCombo->currentIndex() < 0)
		{
			cancelActivePick();
			return;
		}
		IRobotDocumentHost* doc = m_host->document();
		IRobotOsgViewHost* osg = m_host->osgView();
		const std::string backendId = m_backendCombo->currentData().toString().toStdString();
		std::vector<int> kept;
		std::string err;
		if (mesh_triangle_selection::collectTrianglesByPolyline(doc, osg, backendId, polyline, mvp, vw, vh, kept, &err))
		{
			applySelectionIndices(kept, MeshTrajectorySelectionMode::Add);
		}
		else if (m_host->statusBar())
		{
			m_host->statusBar()->showMessage(QString::fromStdString(err), 4000);
		}
		cancelActivePick();
	};
	m_host->setMeshTriangleLabelingPickHandlers(std::move(handlers));
}

void MeshTrajectoryPageWidget::refreshBackendCombo()
{
	m_backendCombo->clear();
	if (!m_host || !m_host->document())
	{
		return;
	}
	BackendDataManager& mgr = m_host->document()->backend();
	for (const auto& data : m_host->document()->listObjects())
	{
		if (!data || data->className() != std::string("Model"))
		{
			continue;
		}
		if (!isTopLevelWorkpieceBackend(mgr, data->id()))
		{
			continue;
		}
		if (!data->hasGeometry())
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		if (id.startsWith(QStringLiteral("RobotURDF_")))
		{
			continue;
		}
		m_backendCombo->addItem(id, id);
	}
	if (m_backendCombo->count() > 0)
	{
		reloadMeshSession();
	}
}

void MeshTrajectoryPageWidget::reloadMeshSession()
{
	endSectionPlaneEdit();
	if (!m_host || !m_host->document() || m_backendCombo->currentIndex() < 0)
	{
		clearMethodPreview();
		return;
	}
	const std::string backendId = m_backendCombo->currentData().toString().toStdString();
	auto data = m_host->document()->findObject(backendId);
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data);
	if (!mesh)
	{
		return;
	}
	(void)m_meshSession->beginMesh(backendId, mesh->triangleSoup());
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		osg->setMeshPickScopeBackendId(backendId);
	}
	updateUiLabels();
	syncSelectionHighlight();
	syncMethodPreview();
}

void MeshTrajectoryPageWidget::syncMethodPreview()
{
	const auto method = static_cast<geoalgo::MeshTrajectoryMethod>(m_methodCombo->currentData().toInt());
	if (method == geoalgo::MeshTrajectoryMethod::CrossSection)
	{
		if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
		{
			osg->clearMeshFittedSurfacePreview();
		}
		syncSectionPlanePreview();
	}
	else
	{
		hideSectionPlanePreview();
		syncBsplineSurfacePreview();
	}
}

void MeshTrajectoryPageWidget::clearMethodPreview()
{
	if (m_shuttingDown)
	{
		return;
	}
	hideSectionPlanePreview();
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->clearMeshFittedSurfacePreview();
	}
}

void MeshTrajectoryPageWidget::syncSectionPlanePreview()
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
	if (!m_showSectionCheck->isChecked())
	{
		osg->setMeshSectionPlanePreviewVisible(false);
		return;
	}
	const std::string backendId = m_backendCombo->currentData().toString().toStdString();
	const double origin[3] = {m_planeOx->value(), m_planeOy->value(), m_planeOz->value()};
	const double normal[3] = {m_planeNx->value(), m_planeNy->value(), m_planeNz->value()};
	osg->showMeshSectionPlane(backendId, origin, normal);
}

void MeshTrajectoryPageWidget::onShowSectionToggled(const bool)
{
	if (!m_showSectionCheck->isChecked() && m_sectionEditActive)
	{
		endSectionPlaneEdit();
	}
	syncSectionPlanePreview();
}

void MeshTrajectoryPageWidget::fillBsplineParamsFromUi(geoalgo::MeshTrajectoryBsplineParams& out) const
{
	out.uvCountU = m_uvCountU->value();
	out.uvCountV = m_uvCountV->value();
	out.gridAngleDeg = m_gridAngleDeg->value();
	out.fitUvSpacingMm = m_fitUvSpacingMm->value();
	out.traceMode = static_cast<geoalgo::MeshTrajectoryUvTraceMode>(m_traceModeCombo->currentData().toInt());
	if (m_nurbsFitModeCombo)
	{
		out.fitMode = static_cast<geoalgo::MeshSurfaceNurbsFitMode>(m_nurbsFitModeCombo->currentData().toInt());
	}
}

void MeshTrajectoryPageWidget::syncBsplineSurfacePreview()
{
	if (!m_host || !m_meshSession || m_backendCombo->currentIndex() < 0)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const auto& selected = m_meshSession->selectedTriangleIndices();
	if (selected.size() < 3U)
	{
		osg->clearMeshFittedSurfacePreview();
		return;
	}
	geoalgo::MeshTrajectoryBsplineParams bspline;
	fillBsplineParamsFromUi(bspline);
	geoalgo::MeshTrajectoryRegion region;
	region.triangleIndices = selected;
	std::vector<float> previewSoup;
	std::string err;
	if (!geoalgo::buildBsplineRegionSurfacePreview(m_meshSession->triangleSoup(), region, bspline, previewSoup, &err))
	{
		osg->clearMeshFittedSurfacePreview();
		return;
	}
	const std::string backendId = m_meshSession->backendIdUtf8();
	std::vector<cloudsim::core::Vec3> vertsWorld;
	mesh_triangle_selection::triangleSoupModelToWorldVerts(osg, backendId, previewSoup, vertsWorld);
	if (vertsWorld.empty())
	{
		osg->clearMeshFittedSurfacePreview();
	}
	else
	{
		osg->showMeshFittedSurfacePreview(vertsWorld);
	}
}

void MeshTrajectoryPageWidget::syncSelectionHighlight()
{
	if (!m_host || !m_host->document() || !m_meshSession)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const std::string backendId = m_meshSession->backendIdUtf8();
	auto data = m_host->document()->findObject(backendId);
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data);
	if (!mesh)
	{
		osg->clearMeshTriangleHighlight();
		return;
	}
	std::vector<cloudsim::core::Vec3> verts;
	mesh_triangle_selection::selectedTrianglesToWorldVerts(*mesh, osg, backendId,
														   m_meshSession->selectedTriangleIndices(), verts);
	if (verts.empty())
	{
		osg->clearMeshTriangleHighlight();
	}
	else
	{
		osg->showMeshTriangleHighlight(verts);
	}
}

void MeshTrajectoryPageWidget::cancelActivePick()
{
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->cancelMeshTrianglePick();
	}
}

void MeshTrajectoryPageWidget::endSectionPlaneEdit()
{
	if (!m_sectionEditActive || m_shuttingDown)
	{
		return;
	}
	m_sectionEditActive = false;
	if (m_host)
	{
		m_host->endMeshSectionPlaneEditDirect();
	}
	if (!m_shuttingDown)
	{
		updateUiLabels();
	}
}

void MeshTrajectoryPageWidget::hideSectionPlanePreview()
{
	if (m_shuttingDown)
	{
		return;
	}
	endSectionPlaneEdit();
	if (m_host)
	{
		m_host->hideMeshSectionPlaneDirect();
	}
}

void MeshTrajectoryPageWidget::startSectionPlaneEdit()
{
	if (!m_host || m_backendCombo->currentIndex() < 0 || !m_showSectionCheck->isChecked())
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const std::string backendId = m_backendCombo->currentData().toString().toStdString();
	const double origin[3] = {m_planeOx->value(), m_planeOy->value(), m_planeOz->value()};
	const double normal[3] = {m_planeNx->value(), m_planeNy->value(), m_planeNz->value()};
	m_sectionEditActive = true;
	updateUiLabels();
	osg->beginMeshSectionPlaneEdit(backendId, origin, normal,
								   [this](const double o[3], const double n[3]) { syncPlaneSpinboxesFromModel(o, n); });
}

void MeshTrajectoryPageWidget::syncPlaneSpinboxesFromModel(const double origin[3], const double normal[3])
{
	m_syncingPlaneSpinboxes = true;
	const QSignalBlocker blockers[] = {QSignalBlocker(m_planeOx), QSignalBlocker(m_planeOy), QSignalBlocker(m_planeOz),
									   QSignalBlocker(m_planeNx), QSignalBlocker(m_planeNy), QSignalBlocker(m_planeNz)};
	(void)blockers;
	m_planeOx->setValue(origin[0]);
	m_planeOy->setValue(origin[1]);
	m_planeOz->setValue(origin[2]);
	m_planeNx->setValue(normal[0]);
	m_planeNy->setValue(normal[1]);
	m_planeNz->setValue(normal[2]);
	m_syncingPlaneSpinboxes = false;
}

void MeshTrajectoryPageWidget::pushPlaneSpinboxesToOsg()
{
	if (!m_host || m_backendCombo->currentIndex() < 0 || !m_showSectionCheck->isChecked())
	{
		return;
	}
	const auto method = static_cast<geoalgo::MeshTrajectoryMethod>(m_methodCombo->currentData().toInt());
	if (method != geoalgo::MeshTrajectoryMethod::CrossSection)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const double origin[3] = {m_planeOx->value(), m_planeOy->value(), m_planeOz->value()};
	const double normal[3] = {m_planeNx->value(), m_planeNy->value(), m_planeNz->value()};
	if (m_sectionEditActive)
	{
		osg->updateMeshSectionPlanePose(origin, normal);
	}
	else
	{
		const std::string backendId = m_backendCombo->currentData().toString().toStdString();
		osg->showMeshSectionPlane(backendId, origin, normal);
	}
}

void MeshTrajectoryPageWidget::applySelectionIndices(const std::vector<int>& indices,
													 const MeshTrajectorySelectionMode mode)
{
	if (!m_meshSession || indices.empty())
	{
		return;
	}
	(void)m_meshSession->applyTriangleSelection(indices, mode);
	updateUiLabels();
	syncSelectionHighlight();
	syncBsplineSurfacePreview();
}

void MeshTrajectoryPageWidget::onBackendChanged(int)
{
	cancelActivePick();
	reloadMeshSession();
}

void MeshTrajectoryPageWidget::onMethodChanged(const int)
{
	applyMethodVisibility();
	updateUiLabels();
}

void MeshTrajectoryPageWidget::onBsplineParamChanged()
{
	syncBsplineSurfacePreview();
}

void MeshTrajectoryPageWidget::onEditSectionClicked()
{
	if (m_sectionEditActive)
	{
		endSectionPlaneEdit();
	}
	else
	{
		startSectionPlaneEdit();
	}
}

void MeshTrajectoryPageWidget::onPlaneSpinChanged()
{
	if (m_syncingPlaneSpinboxes)
	{
		return;
	}
	pushPlaneSpinboxesToOsg();
}

void MeshTrajectoryPageWidget::onFromCameraNormalClicked()
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
	const std::string backendId = m_backendCombo->currentData().toString().toStdString();
	double dir[3] = {};
	if (!osg->getCameraViewDirectionInBackendModel(backendId, dir))
	{
		return;
	}
	m_planeNx->setValue(dir[0]);
	m_planeNy->setValue(dir[1]);
	m_planeNz->setValue(dir[2]);
	pushPlaneSpinboxesToOsg();
}

void MeshTrajectoryPageWidget::onPickClickClicked()
{
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->setMeshTrianglePickTool(MeshTrianglePickTool::Click);
	}
}

void MeshTrajectoryPageWidget::onPickBrushClicked()
{
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->setMeshTrianglePickTool(MeshTrianglePickTool::Brush, 14.f);
	}
}

void MeshTrajectoryPageWidget::onPickPolylineClicked()
{
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->setMeshTrianglePickTool(MeshTrianglePickTool::Polyline);
	}
}

void MeshTrajectoryPageWidget::onCancelPickClicked()
{
	cancelActivePick();
}

void MeshTrajectoryPageWidget::onClearSelectionClicked()
{
	if (m_meshSession)
	{
		(void)m_meshSession->clearSelection();
		updateUiLabels();
		syncSelectionHighlight();
		syncBsplineSurfacePreview();
	}
}

void MeshTrajectoryPageWidget::resetAfterTrajectoryCommit()
{
	cancelActivePick();
	endSectionPlaneEdit();
	clearMethodPreview();
	if (m_meshSession)
	{
		(void)m_meshSession->clearSelection();
	}
	syncSelectionHighlight();
	if (m_statusLabel)
	{
		m_statusLabel->setText(m_chinese ? QStringLiteral("轨迹已提交，请重新选择 mesh 区域")
										 : QStringLiteral("Trajectory committed; reselect mesh region"));
	}
	updateUiLabels();
}

void MeshTrajectoryPageWidget::onInvertSelectionClicked()
{
	if (m_meshSession)
	{
		(void)m_meshSession->invertSelection();
		updateUiLabels();
		syncSelectionHighlight();
		syncBsplineSurfacePreview();
	}
}

void MeshTrajectoryPageWidget::onGenerateClicked()
{
	endSectionPlaneEdit();
	if (!m_host || !m_session || !m_meshSession || m_backendCombo->currentIndex() < 0)
	{
		return;
	}
	geoalgo::MeshTrajectorySpec spec = m_meshSession->spec();
	spec.workpiece.backendIdUtf8 = m_meshSession->backendIdUtf8();
	spec.method = static_cast<geoalgo::MeshTrajectoryMethod>(m_methodCombo->currentData().toInt());
	spec.crossSection.planeOriginMm[0] = m_planeOx->value();
	spec.crossSection.planeOriginMm[1] = m_planeOy->value();
	spec.crossSection.planeOriginMm[2] = m_planeOz->value();
	spec.crossSection.planeNormal[0] = m_planeNx->value();
	spec.crossSection.planeNormal[1] = m_planeNy->value();
	spec.crossSection.planeNormal[2] = m_planeNz->value();
	fillBsplineParamsFromUi(spec.bspline);
	if (spec.method == geoalgo::MeshTrajectoryMethod::CrossSection)
	{
		spec.discretize.stepMm = m_stepMm->value();
		spec.discretize.outputNormal = m_outputNormalCheck->isChecked();
		spec.discretize.outputTangent = m_outputTangentCheck->isChecked();
	}
	else
	{
		spec.discretize.outputNormal = m_bsplineOutputNormalCheck->isChecked();
		spec.discretize.outputTangent = m_bsplineOutputTangentCheck->isChecked();
		spec.discretize.stepMm = 0.0;
	}
	m_meshSession->setSpec(spec);

	const QString backendId = m_backendCombo->currentData().toString();
	const MeshTrajectorySession sessionCopy = *m_meshSession;
	struct GenResult
	{
		geoalgo::RawPath path;
		std::string err;
		bool ok = false;
	};
	const auto result = std::make_shared<GenResult>();
	m_host->enqueueBackgroundJob(
		m_chinese ? QStringLiteral("Mesh 轨迹生成") : QStringLiteral("Mesh trajectory"),
		[sessionCopy, result]() { result->ok = sessionCopy.generateRawPath(result->path, &result->err); },
		[this, backendId, sessionCopy, result](const bool threw, const QString& msg)
		{
			if (threw)
			{
				m_statusLabel->setText(msg);
				return;
			}
			if (!result->ok)
			{
				m_statusLabel->setText(QString::fromStdString(result->err));
				return;
			}
			RobotInstruction::RawTrajectory traj;
			RobotInstruction::MeshTrajectoryIngressParams ingress;
			const std::string specJson = sessionCopy.specJsonUtf8();
			std::string err;
			if (!RobotInstruction::importMeshRawPathToRawTrajectory(result->path, specJson, ingress, traj, &err))
			{
				m_statusLabel->setText(QString::fromStdString(err));
				return;
			}
			m_session->setRawTrajectory(traj);
			feature_pick_transform::applyMeshLocalRawTrajectoryPreviewToOsg(
				m_host->osgView(), backendId.toStdString(), traj, RobotOsgUi::RawTrajectoryPreviewOptions{}, &err);
			const int segCount =
				traj.segmentEndExclusive.empty() ? 1 : static_cast<int>(traj.segmentEndExclusive.size());
			m_statusLabel->setText(m_chinese
									   ? QStringLiteral("已生成 %1 点（%2 段交线），已写入 PathPlan")
											 .arg(static_cast<int>(traj.points.size()))
											 .arg(segCount)
									   : QStringLiteral("Generated %1 points (%2 polylines), attached to PathPlan")
											 .arg(static_cast<int>(traj.points.size()))
											 .arg(segCount));
			if (m_host->statusBar())
			{
				m_host->statusBar()->showMessage(m_statusLabel->text(), 5000);
			}
		});
}
