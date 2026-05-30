#include "GeometryDockWidget.h"

#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"

#include <QComboBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{
enum class SourceMode
{
	File = 0,
	Backend = 1
};
}

GeometryDockWidget::GeometryDockWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
	, m_useChinese(host ? host->useChinese() : true)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(12, 12, 12, 12);

	m_docGroup = new QGroupBox(this);
	auto* docLayout = new QVBoxLayout(m_docGroup);
	m_docLabel = new QLabel(m_docGroup);
	docLayout->addWidget(m_docLabel);
	layout->addWidget(m_docGroup);

	m_stepGroup = new QGroupBox(this);
	auto* stepLayout = new QVBoxLayout(m_stepGroup);
	m_sourceCombo = new QComboBox(m_stepGroup);
	m_sourceCombo->addItem(QStringLiteral("STEP"), static_cast<int>(SourceMode::File));
	m_sourceCombo->addItem(QStringLiteral("Backend"), static_cast<int>(SourceMode::Backend));
	stepLayout->addWidget(m_sourceCombo);

	auto* backendRow = new QHBoxLayout;
	m_backendCombo = new QComboBox(m_stepGroup);
	m_refreshBackendsBtn = new QPushButton(m_stepGroup);
	backendRow->addWidget(m_backendCombo, 1);
	backendRow->addWidget(m_refreshBackendsBtn);
	stepLayout->addLayout(backendRow);

	auto* pathRow = new QHBoxLayout;
	m_stepPathEdit = new QLineEdit(m_stepGroup);
	m_browseBtn = new QPushButton(m_stepGroup);
	pathRow->addWidget(m_stepPathEdit, 1);
	pathRow->addWidget(m_browseBtn);
	stepLayout->addLayout(pathRow);
	m_qualityCombo = new QComboBox(m_stepGroup);
	m_qualityCombo->addItem(QStringLiteral("Medium"), static_cast<int>(PluginMeshQualityPreset::Medium));
	m_qualityCombo->addItem(QStringLiteral("Fine"), static_cast<int>(PluginMeshQualityPreset::Fine));
	m_qualityCombo->addItem(QStringLiteral("Coarse"), static_cast<int>(PluginMeshQualityPreset::Coarse));
	stepLayout->addWidget(m_qualityCombo);
	m_discretizeBtn = new QPushButton(m_stepGroup);
	stepLayout->addWidget(m_discretizeBtn);
	layout->addWidget(m_stepGroup);

	m_ixEdgeFaceGroup = new QGroupBox(this);
	auto* ixLayout = new QVBoxLayout(m_ixEdgeFaceGroup);
	auto* edgeRow = new QHBoxLayout;
	m_edgeSpin = new QSpinBox(m_ixEdgeFaceGroup);
	m_edgeSpin->setRange(0, 9999);
	m_faceSpin = new QSpinBox(m_ixEdgeFaceGroup);
	m_faceSpin->setRange(0, 9999);
	edgeRow->addWidget(new QLabel(QStringLiteral("E"), m_ixEdgeFaceGroup));
	edgeRow->addWidget(m_edgeSpin);
	edgeRow->addWidget(new QLabel(QStringLiteral("F"), m_ixEdgeFaceGroup));
	edgeRow->addWidget(m_faceSpin);
	ixLayout->addLayout(edgeRow);

	auto* pickRow = new QHBoxLayout;
	m_pickEdgeBtn = new QPushButton(m_ixEdgeFaceGroup);
	m_pickFaceBtn = new QPushButton(m_ixEdgeFaceGroup);
	pickRow->addWidget(m_pickEdgeBtn);
	pickRow->addWidget(m_pickFaceBtn);
	ixLayout->addLayout(pickRow);

	m_ixEdgeFaceBtn = new QPushButton(m_ixEdgeFaceGroup);
	ixLayout->addWidget(m_ixEdgeFaceBtn);
	layout->addWidget(m_ixEdgeFaceGroup);

	m_ixFaceFaceGroup = new QGroupBox(this);
	auto* ffLayout = new QVBoxLayout(m_ixFaceFaceGroup);
	auto* ffIdxRow = new QHBoxLayout;
	m_faceASpin = new QSpinBox(m_ixFaceFaceGroup);
	m_faceASpin->setRange(0, 9999);
	m_faceBSpin = new QSpinBox(m_ixFaceFaceGroup);
	m_faceBSpin->setRange(0, 9999);
	ffIdxRow->addWidget(new QLabel(QStringLiteral("F1"), m_ixFaceFaceGroup));
	ffIdxRow->addWidget(m_faceASpin);
	ffIdxRow->addWidget(new QLabel(QStringLiteral("F2"), m_ixFaceFaceGroup));
	ffIdxRow->addWidget(m_faceBSpin);
	ffLayout->addLayout(ffIdxRow);

	auto* ffPickRow = new QHBoxLayout;
	m_pickFaceABtn = new QPushButton(m_ixFaceFaceGroup);
	m_pickFaceBBtn = new QPushButton(m_ixFaceFaceGroup);
	ffPickRow->addWidget(m_pickFaceABtn);
	ffPickRow->addWidget(m_pickFaceBBtn);
	ffLayout->addLayout(ffPickRow);

	m_ixFaceFaceBtn = new QPushButton(m_ixFaceFaceGroup);
	ffLayout->addWidget(m_ixFaceFaceBtn);
	layout->addWidget(m_ixFaceFaceGroup);

	m_resultGroup = new QGroupBox(this);
	auto* resultLayout = new QHBoxLayout(m_resultGroup);
	m_createTubeBtn = new QPushButton(m_resultGroup);
	m_createRibbonBtn = new QPushButton(m_resultGroup);
	resultLayout->addWidget(m_createTubeBtn);
	resultLayout->addWidget(m_createRibbonBtn);
	layout->addWidget(m_resultGroup);

	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	layout->addWidget(m_statusLabel);
	layout->addStretch();

	wireSignals();
	applyLanguage();
	refreshDocumentLabel();
	refreshComputableBackends();
	syncSourceUiState();
}

void GeometryDockWidget::wireSignals()
{
	connect(m_browseBtn, &QPushButton::clicked, this, &GeometryDockWidget::browseStepFile);
	connect(m_refreshBackendsBtn, &QPushButton::clicked, this, &GeometryDockWidget::refreshComputableBackends);
	connect(m_sourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
		[this](int) { syncSourceUiState(); });
	connect(m_discretizeBtn, &QPushButton::clicked, this, &GeometryDockWidget::discretizeStep);
	connect(m_pickEdgeBtn, &QPushButton::clicked, this, &GeometryDockWidget::pickEdgeForEdgeFace);
	connect(m_pickFaceBtn, &QPushButton::clicked, this, &GeometryDockWidget::pickFaceForEdgeFace);
	connect(m_ixEdgeFaceBtn, &QPushButton::clicked, this, &GeometryDockWidget::intersectEdgeFace);
	connect(m_pickFaceABtn, &QPushButton::clicked, this, &GeometryDockWidget::pickFaceAForFaceFace);
	connect(m_pickFaceBBtn, &QPushButton::clicked, this, &GeometryDockWidget::pickFaceBForFaceFace);
	connect(m_ixFaceFaceBtn, &QPushButton::clicked, this, &GeometryDockWidget::intersectFaceFace);
	connect(m_createTubeBtn, &QPushButton::clicked, this, &GeometryDockWidget::createTubeFromLastIntersection);
	connect(m_createRibbonBtn, &QPushButton::clicked, this, &GeometryDockWidget::createRibbonFromLastIntersection);
}

bool GeometryDockWidget::ensureGeometryHostReady()
{
	if (m_host && m_host->geometryHost())
	{
		return true;
	}
	setStatus(m_useChinese ? QStringLiteral("geometry host 不可用") : QStringLiteral("geometry host unavailable"), true);
	return false;
}

void GeometryDockWidget::setStatus(const QString& text, const bool isError)
{
	m_statusLabel->setText(text);
	m_statusLabel->setStyleSheet(isError ? QStringLiteral("color:#b22222;") : QStringLiteral(""));
}

bool GeometryDockWidget::hasBackendSource() const
{
	return m_sourceCombo && m_sourceCombo->currentData().toInt() == static_cast<int>(SourceMode::Backend);
}

void GeometryDockWidget::syncSourceUiState()
{
	const bool backendMode = hasBackendSource();
	m_backendCombo->setEnabled(backendMode);
	m_refreshBackendsBtn->setEnabled(backendMode);
	m_stepPathEdit->setEnabled(!backendMode);
	m_browseBtn->setEnabled(!backendMode);
}

std::string GeometryDockWidget::activeBackendId() const
{
	if (!hasBackendSource() || !m_backendCombo)
	{
		return std::string();
	}
	const int idx = m_backendCombo->currentData().toInt();
	if (idx < 0 || idx >= static_cast<int>(m_backendEntries.size()))
	{
		return std::string();
	}
	return m_backendEntries[static_cast<std::size_t>(idx)].backendId;
}

QString GeometryDockWidget::activeStepPath() const
{
	if (!hasBackendSource() || !m_backendCombo)
	{
		return m_stepPathEdit->text().trimmed();
	}
	const int idx = m_backendCombo->currentData().toInt();
	if (idx < 0 || idx >= static_cast<int>(m_backendEntries.size()))
	{
		return QString();
	}
	return QString::fromStdString(m_backendEntries[static_cast<std::size_t>(idx)].stepPathUtf8);
}

PluginMeshDiscretizeParams GeometryDockWidget::buildDiscretizeParams() const
{
	PluginMeshDiscretizeParams params;
	params.quality = static_cast<PluginMeshQualityPreset>(m_qualityCombo->currentData().toInt());
	return params;
}

PluginMeshCreateOptions GeometryDockWidget::buildMeshCreateOptions(const QString& displayName) const
{
	PluginMeshCreateOptions options;
	options.displayName = displayName;
	options.sourcePath = activeStepPath();
	options.selectInTree = true;
	return options;
}

void GeometryDockWidget::refreshComputableBackends()
{
	m_backendEntries.clear();
	m_backendCombo->clear();
	if (!ensureGeometryHostReady())
	{
		return;
	}
	std::vector<PluginGeometryBackendEntry> entries;
	QString err;
	const bool ok = m_host->geometryHost()->listComputableBackends(m_host->activeDocument(), entries, &err);
	if (!ok)
	{
		setStatus(err.isEmpty()
				? (m_useChinese ? QStringLiteral("无可计算后端") : QStringLiteral("No computable backend"))
				: err,
			true);
		return;
	}
	m_backendEntries = std::move(entries);
	for (int i = 0; i < static_cast<int>(m_backendEntries.size()); ++i)
	{
		const auto& entry = m_backendEntries[static_cast<std::size_t>(i)];
		const QString name = QStringLiteral("%1 (%2)")
								 .arg(QString::fromStdString(entry.displayName), QString::fromStdString(entry.backendId));
		m_backendCombo->addItem(name, i);
	}
}

void GeometryDockWidget::pickEdgeForEdgeFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (!hasBackendSource())
	{
		setStatus(m_useChinese ? QStringLiteral("3D 点选需切换到后端输入模式") : QStringLiteral("Switch to backend mode for 3D pick"), true);
		return;
	}
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	req.backendIdUtf8 = activeBackendId();
	req.stepPathUtf8 = activeStepPath().toStdString();
	setStatus(m_useChinese ? QStringLiteral("请在 3D 视图点选边…") : QStringLiteral("Pick an edge in 3D view..."));
	m_host->geometryHost()->pickStepElementFromViewport(
		m_host->activeDocument(),
		req,
		[this](const bool ok, const QString& err, const PluginGeometryStepRef& ref) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_edgeSpin->setValue(ref.edgeIndex);
			setStatus(m_useChinese ? QStringLiteral("已选中边 %1").arg(ref.edgeIndex)
								   : QStringLiteral("Picked edge %1").arg(ref.edgeIndex));
		});
}

void GeometryDockWidget::pickFaceForEdgeFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (!hasBackendSource())
	{
		setStatus(m_useChinese ? QStringLiteral("3D 点选需切换到后端输入模式") : QStringLiteral("Switch to backend mode for 3D pick"), true);
		return;
	}
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = activeBackendId();
	req.stepPathUtf8 = activeStepPath().toStdString();
	setStatus(m_useChinese ? QStringLiteral("请在 3D 视图点选面…") : QStringLiteral("Pick a face in 3D view..."));
	m_host->geometryHost()->pickStepElementFromViewport(
		m_host->activeDocument(),
		req,
		[this](const bool ok, const QString& err, const PluginGeometryStepRef& ref) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_faceSpin->setValue(ref.faceIndex);
			setStatus(m_useChinese ? QStringLiteral("已选中面 %1").arg(ref.faceIndex)
								   : QStringLiteral("Picked face %1").arg(ref.faceIndex));
		});
}

void GeometryDockWidget::pickFaceAForFaceFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (!hasBackendSource())
	{
		setStatus(m_useChinese ? QStringLiteral("3D 点选需切换到后端输入模式") : QStringLiteral("Switch to backend mode for 3D pick"), true);
		return;
	}
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = activeBackendId();
	req.stepPathUtf8 = activeStepPath().toStdString();
	setStatus(m_useChinese ? QStringLiteral("请点选第一个面…") : QStringLiteral("Pick first face..."));
	m_host->geometryHost()->pickStepElementFromViewport(
		m_host->activeDocument(),
		req,
		[this](const bool ok, const QString& err, const PluginGeometryStepRef& ref) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_faceASpin->setValue(ref.faceIndex);
			setStatus(m_useChinese ? QStringLiteral("已选中 F1=%1").arg(ref.faceIndex)
								   : QStringLiteral("Picked F1=%1").arg(ref.faceIndex));
		});
}

void GeometryDockWidget::pickFaceBForFaceFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (!hasBackendSource())
	{
		setStatus(m_useChinese ? QStringLiteral("3D 点选需切换到后端输入模式") : QStringLiteral("Switch to backend mode for 3D pick"), true);
		return;
	}
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = activeBackendId();
	req.stepPathUtf8 = activeStepPath().toStdString();
	setStatus(m_useChinese ? QStringLiteral("请点选第二个面…") : QStringLiteral("Pick second face..."));
	m_host->geometryHost()->pickStepElementFromViewport(
		m_host->activeDocument(),
		req,
		[this](const bool ok, const QString& err, const PluginGeometryStepRef& ref) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_faceBSpin->setValue(ref.faceIndex);
			setStatus(m_useChinese ? QStringLiteral("已选中 F2=%1").arg(ref.faceIndex)
								   : QStringLiteral("Picked F2=%1").arg(ref.faceIndex));
		});
}

void GeometryDockWidget::applyLanguage()
{
	const bool zh = m_useChinese;
	m_docGroup->setTitle(zh ? QStringLiteral("文档") : QStringLiteral("Document"));
	m_stepGroup->setTitle(zh ? QStringLiteral("输入与离散") : QStringLiteral("Input & Discretize"));
	m_sourceCombo->setItemText(0, zh ? QStringLiteral("STEP 文件") : QStringLiteral("STEP file"));
	m_sourceCombo->setItemText(1, zh ? QStringLiteral("内部后端对象") : QStringLiteral("Backend object"));
	m_refreshBackendsBtn->setText(zh ? QStringLiteral("刷新") : QStringLiteral("Refresh"));
	m_browseBtn->setText(zh ? QStringLiteral("浏览…") : QStringLiteral("Browse..."));
	m_discretizeBtn->setText(zh ? QStringLiteral("离散生成网格") : QStringLiteral("Discretize to Mesh"));
	m_ixEdgeFaceGroup->setTitle(zh ? QStringLiteral("线面求交") : QStringLiteral("Edge-Face Intersect"));
	m_pickEdgeBtn->setText(zh ? QStringLiteral("点选边") : QStringLiteral("Pick Edge"));
	m_pickFaceBtn->setText(zh ? QStringLiteral("点选面") : QStringLiteral("Pick Face"));
	m_ixEdgeFaceBtn->setText(zh ? QStringLiteral("执行线面求交") : QStringLiteral("Run Edge-Face"));
	m_ixFaceFaceGroup->setTitle(zh ? QStringLiteral("面面求交") : QStringLiteral("Face-Face Intersect"));
	m_pickFaceABtn->setText(zh ? QStringLiteral("点选 F1") : QStringLiteral("Pick F1"));
	m_pickFaceBBtn->setText(zh ? QStringLiteral("点选 F2") : QStringLiteral("Pick F2"));
	m_ixFaceFaceBtn->setText(zh ? QStringLiteral("执行面面求交") : QStringLiteral("Run Face-Face"));
	m_resultGroup->setTitle(zh ? QStringLiteral("求交结果生成后端") : QStringLiteral("Build Backend From Intersection"));
	m_createTubeBtn->setText(zh ? QStringLiteral("生成管状网格") : QStringLiteral("Create Tube Mesh"));
	m_createRibbonBtn->setText(zh ? QStringLiteral("生成带状网格") : QStringLiteral("Create Ribbon Mesh"));
}

void GeometryDockWidget::refreshDocumentLabel()
{
	if (!m_host)
	{
		return;
	}
	IPluginDocument* doc = m_host->activeDocument();
	m_docLabel->setText(doc ? QString::fromStdString(doc->documentLabel())
							: (m_useChinese ? QStringLiteral("无活动文档") : QStringLiteral("No active document")));
	refreshComputableBackends();
}

void GeometryDockWidget::browseStepFile()
{
	const QString path = QFileDialog::getOpenFileName(
		this,
		m_useChinese ? QStringLiteral("选择 STEP") : QStringLiteral("Select STEP"),
		QString(),
		QStringLiteral("STEP (*.step *.stp)"));
	if (!path.isEmpty())
	{
		m_stepPathEdit->setText(path);
	}
}

void GeometryDockWidget::discretizeStep()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	const QString path = activeStepPath();
	if (path.isEmpty())
	{
		setStatus(m_useChinese ? QStringLiteral("请选择 STEP 文件或后端对象") : QStringLiteral("Select STEP file or backend"), true);
		return;
	}
	const PluginMeshDiscretizeParams params = buildDiscretizeParams();
	const PluginMeshCreateOptions options = buildMeshCreateOptions(QStringLiteral("GeometryDiscretizedMesh"));
	setStatus(m_useChinese ? QStringLiteral("离散中…") : QStringLiteral("Discretizing..."));

	const std::string backendId = activeBackendId();
	if (hasBackendSource() && !backendId.empty())
	{
		m_host->geometryHost()->discretizeBackendToMesh(
			m_host->activeDocument(),
			path.toStdString(),
			params,
			options,
			[this](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
				if (!ok)
				{
					setStatus(err, true);
					return;
				}
				setStatus(QStringLiteral("ok: %1 tris=%2")
							  .arg(QString::fromStdString(result.newBackendId))
							  .arg(result.triangleCount));
			});
		return;
	}

	m_host->geometryHost()->discretizeStepToMesh(
		m_host->activeDocument(),
		path.toStdString(),
		params,
		options,
		[this](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			setStatus(QStringLiteral("ok: %1 tris=%2")
						  .arg(QString::fromStdString(result.newBackendId))
						  .arg(result.triangleCount));
		});
}

void GeometryDockWidget::intersectEdgeFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	const QString path = activeStepPath();
	if (path.isEmpty())
	{
		setStatus(m_useChinese ? QStringLiteral("缺少 STEP 路径") : QStringLiteral("STEP path required"), true);
		return;
	}
	PluginGeometryStepRef edgeRef;
	edgeRef.stepPathUtf8 = path.toStdString();
	edgeRef.edgeIndex = m_edgeSpin->value();
	PluginGeometryStepRef faceRef;
	faceRef.stepPathUtf8 = path.toStdString();
	faceRef.faceIndex = m_faceSpin->value();
	PluginGeometryIntersectionParams params;
	setStatus(m_useChinese ? QStringLiteral("线面求交中…") : QStringLiteral("Edge-face intersecting..."));
	m_host->geometryHost()->intersectEdgeFace(
		m_host->activeDocument(),
		edgeRef,
		faceRef,
		params,
		[this, path](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_lastIntersectionPolylines = result.polylines;
			m_lastIntersectionSourcePath = path;
			setStatus(QStringLiteral("hits=%1 curves=%2 maxRes=%3")
						  .arg(result.intersectionPoints.size())
						  .arg(result.polylines.size())
						  .arg(result.maxResidualMm));
		});
}

void GeometryDockWidget::intersectFaceFace()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	const QString path = activeStepPath();
	if (path.isEmpty())
	{
		setStatus(m_useChinese ? QStringLiteral("缺少 STEP 路径") : QStringLiteral("STEP path required"), true);
		return;
	}
	PluginGeometryStepRef f1;
	f1.stepPathUtf8 = path.toStdString();
	f1.faceIndex = m_faceASpin->value();
	PluginGeometryStepRef f2;
	f2.stepPathUtf8 = path.toStdString();
	f2.faceIndex = m_faceBSpin->value();
	PluginGeometryIntersectionParams params;
	setStatus(m_useChinese ? QStringLiteral("面面求交中…") : QStringLiteral("Face-face intersecting..."));
	m_host->geometryHost()->intersectFaces(
		m_host->activeDocument(),
		f1,
		f2,
		params,
		[this, path](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			m_lastIntersectionPolylines = result.polylines;
			m_lastIntersectionSourcePath = path;
			setStatus(QStringLiteral("hits=%1 curves=%2 maxRes=%3")
						  .arg(result.intersectionPoints.size())
						  .arg(result.polylines.size())
						  .arg(result.maxResidualMm));
		});
}

void GeometryDockWidget::createTubeFromLastIntersection()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (m_lastIntersectionPolylines.empty())
	{
		setStatus(m_useChinese ? QStringLiteral("请先执行求交") : QStringLiteral("Run intersection first"), true);
		return;
	}
	PluginMeshDiscretizeParams params = buildDiscretizeParams();
	params.mode = PluginMeshDiscretizeMode::WireTubeMesh;
	const PluginMeshCreateOptions options = buildMeshCreateOptions(QStringLiteral("IntersectionTubeMesh"));
	setStatus(m_useChinese ? QStringLiteral("生成管状网格中…") : QStringLiteral("Creating tube mesh..."));
	m_host->geometryHost()->discretizeWireToTubeMesh(
		m_host->activeDocument(),
		m_lastIntersectionPolylines.front(),
		params,
		options,
		[this](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			setStatus(QStringLiteral("ok: %1 tris=%2")
						  .arg(QString::fromStdString(result.newBackendId))
						  .arg(result.triangleCount));
		});
}

void GeometryDockWidget::createRibbonFromLastIntersection()
{
	if (!ensureGeometryHostReady())
	{
		return;
	}
	if (m_lastIntersectionPolylines.empty())
	{
		setStatus(m_useChinese ? QStringLiteral("请先执行求交") : QStringLiteral("Run intersection first"), true);
		return;
	}
	PluginMeshDiscretizeParams params = buildDiscretizeParams();
	params.mode = PluginMeshDiscretizeMode::WireRibbonMesh;
	const PluginMeshCreateOptions options = buildMeshCreateOptions(QStringLiteral("IntersectionRibbonMesh"));
	setStatus(m_useChinese ? QStringLiteral("生成带状网格中…") : QStringLiteral("Creating ribbon mesh..."));
	m_host->geometryHost()->discretizeWireToRibbonMesh(
		m_host->activeDocument(),
		m_lastIntersectionPolylines.front(),
		params,
		options,
		[this](const bool ok, const QString& err, const PluginGeometryJobResult& result) {
			if (!ok)
			{
				setStatus(err, true);
				return;
			}
			setStatus(QStringLiteral("ok: %1 tris=%2")
						  .arg(QString::fromStdString(result.newBackendId))
						  .arg(result.triangleCount));
		});
}
