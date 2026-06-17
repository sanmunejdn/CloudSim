#include "LabelingAnnotWidget.h"

#include "IPluginDocument.h"
#include "IPluginHostContext.h"
#include "IPluginLabelingHost.h"
#include "PointNetInference.h"

#include <json.hpp>

#include <QAbstractButton>
#include <QButtonGroup>
#include <QColorDialog>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <QAbstractItemView>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace
{

bool isLabelingTargetClass(const std::string& cls)
{
	// MeshBackendData::className() 为 "Model"，与 PointCloudPlugin 等模块一致
	return cls == "PointCloudBackendData" || cls == "Model";
}

bool parseAsciiPlyPoints(const QString& path, std::vector<float>& outPoints, int& outCount)
{
	std::ifstream file(path.toStdString());
	if (!file.is_open())
	{
		return false;
	}
	std::string line;
	int vertexCount = 0;
	bool inHeader = true;
	while (std::getline(file, line))
	{
		if (line.substr(0, 15) == "element vertex ")
		{
			vertexCount = std::stoi(line.substr(15));
		}
		else if (line == "end_header")
		{
			inHeader = false;
			break;
		}
	}
	if (vertexCount <= 0 || inHeader)
	{
		return false;
	}
	outPoints.resize(static_cast<std::size_t>(vertexCount) * 3U);
	outCount = vertexCount;
	for (int i = 0; i < vertexCount; ++i)
	{
		if (!std::getline(file, line))
		{
			break;
		}
		std::istringstream iss(line);
		iss >> outPoints[static_cast<std::size_t>(i) * 3U + 0U]
			>> outPoints[static_cast<std::size_t>(i) * 3U + 1U]
			>> outPoints[static_cast<std::size_t>(i) * 3U + 2U];
	}
	return true;
}

QString findPointNetConfigPath(IPluginHostContext* host)
{
	if (!host)
	{
		return {};
	}
	QStringList searchPaths;
	const QString pluginDir = host->applicationDirPath();
	if (!pluginDir.isEmpty())
	{
		searchPaths.append(pluginDir);
		QFileInfo info(pluginDir);
		if (info.dir().cdUp())
		{
			searchPaths.append(info.dir().absolutePath());
		}
		searchPaths.append(pluginDir + QStringLiteral("/../com.cloudsim.pointnet"));
	}
	for (const QString& dir : searchPaths)
	{
		const QString candidate = QDir::cleanPath(dir + QStringLiteral("/pointnet_config.json"));
		if (QFile::exists(candidate))
		{
			return candidate;
		}
	}
	return {};
}

QString nextExportSampleBaseNameFromDir(const QString& exportDir)
{
	int maxIndex = 0;
	const QRegularExpression plyRe(QStringLiteral("^sample_(\\d+)\\.ply$"), QRegularExpression::CaseInsensitiveOption);
	const QDir dataDir(QDir(exportDir).filePath(QStringLiteral("data")));
	if (dataDir.exists())
	{
		const QStringList plyFiles = dataDir.entryList(QStringList{ QStringLiteral("sample_*.ply") }, QDir::Files);
		for (const QString& name : plyFiles)
		{
			const QRegularExpressionMatch m = plyRe.match(name);
			if (m.hasMatch())
			{
				maxIndex = std::max(maxIndex, m.captured(1).toInt());
			}
		}
	}

	const QString jsonlPath = QDir(exportDir).filePath(QStringLiteral("dataset.jsonl"));
	QFile jsonl(jsonlPath);
	if (jsonl.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		const QRegularExpression inputRe(QStringLiteral("^sample_(\\d+)\\.ply$"), QRegularExpression::CaseInsensitiveOption);
		while (!jsonl.atEnd())
		{
			const QByteArray line = jsonl.readLine().trimmed();
			if (line.isEmpty())
			{
				continue;
			}
			try
			{
				const nlohmann::json entry = nlohmann::json::parse(line.constData(), nullptr, true);
				const std::string input = entry.value("input", std::string());
				const QString inputName = QString::fromStdString(input);
				const QRegularExpressionMatch m = inputRe.match(inputName);
				if (m.hasMatch())
				{
					maxIndex = std::max(maxIndex, m.captured(1).toInt());
				}
			}
			catch (...)
			{
			}
		}
	}

	return QStringLiteral("sample_%1").arg(maxIndex + 1, 3, 10, QChar('0'));
}

} // namespace

LabelingAnnotWidget::LabelingAnnotWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
	, m_labelingHost(host ? host->labelingHost() : nullptr)
	, m_inference(std::make_unique<PointNetInference>())
{
	auto* outer = new QVBoxLayout(this);
	outer->setContentsMargins(0, 0, 0, 0);
	auto* scroll = new QScrollArea(this);
	scroll->setWidgetResizable(true);
	auto* content = new QWidget(scroll);
	auto* layout = new QVBoxLayout(content);
	layout->setContentsMargins(12, 12, 12, 12);
	layout->setSpacing(8);
	scroll->setWidget(content);
	outer->addWidget(scroll);

	m_targetGroup = new QGroupBox(content);
	auto* targetLayout = new QVBoxLayout(m_targetGroup);
	m_backendCombo = new QComboBox(m_targetGroup);
	m_refreshBackendsBtn = new QPushButton(m_targetGroup);
	m_summaryLabel = new QLabel(m_targetGroup);
	m_summaryLabel->setWordWrap(true);
	auto* backendRow = new QHBoxLayout;
	backendRow->addWidget(m_backendCombo, 1);
	backendRow->addWidget(m_refreshBackendsBtn);
	targetLayout->addLayout(backendRow);
	targetLayout->addWidget(m_summaryLabel);
	layout->addWidget(m_targetGroup);

	m_classGroup = new QGroupBox(content);
	auto* classLayout = new QVBoxLayout(m_classGroup);
	m_classTable = new QTableWidget(4, 3, m_classGroup);
	m_classTable->setHorizontalHeaderLabels({ QStringLiteral("ID"), QStringLiteral("Name"), QStringLiteral("Color") });
	m_classTable->horizontalHeader()->setStretchLastSection(true);
	m_classTable->verticalHeader()->setVisible(false);
	m_classTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_classTable->setSelectionMode(QAbstractItemView::SingleSelection);
	m_classTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
	classLayout->addWidget(m_classTable);
	layout->addWidget(m_classGroup);

	m_toolGroup = new QGroupBox(content);
	auto* toolLayout = new QVBoxLayout(m_toolGroup);
	m_toolButtons = new QButtonGroup(this);
	m_clickTool = new QToolButton(m_toolGroup);
	m_brushTool = new QToolButton(m_toolGroup);
	m_lassoTool = new QToolButton(m_toolGroup);
	m_eraseTool = new QToolButton(m_toolGroup);
	m_clickTool->setCheckable(true);
	m_brushTool->setCheckable(true);
	m_lassoTool->setCheckable(true);
	m_eraseTool->setCheckable(true);
	m_toolButtons->addButton(m_clickTool, static_cast<int>(PluginLabelingTool::Click));
	m_toolButtons->addButton(m_brushTool, static_cast<int>(PluginLabelingTool::Brush));
	m_toolButtons->addButton(m_lassoTool, static_cast<int>(PluginLabelingTool::Polyline));
	m_toolButtons->addButton(m_eraseTool, static_cast<int>(PluginLabelingTool::Erase));
	auto* toolRow = new QHBoxLayout;
	toolRow->addWidget(m_clickTool);
	toolRow->addWidget(m_brushTool);
	toolRow->addWidget(m_lassoTool);
	toolRow->addWidget(m_eraseTool);
	toolLayout->addLayout(toolRow);
	m_cancelPickBtn = new QPushButton(m_toolGroup);
	toolLayout->addWidget(m_cancelPickBtn);
	m_brushRadiusSpin = new QDoubleSpinBox(m_toolGroup);
	m_brushRadiusSpin->setRange(4.0, 128.0);
	m_brushRadiusSpin->setValue(16.0);
	toolLayout->addWidget(m_brushRadiusSpin);
	auto* undoRow = new QHBoxLayout;
	m_undoBtn = new QPushButton(m_toolGroup);
	m_redoBtn = new QPushButton(m_toolGroup);
	undoRow->addWidget(m_undoBtn);
	undoRow->addWidget(m_redoBtn);
	toolLayout->addLayout(undoRow);
	layout->addWidget(m_toolGroup);

	m_prelabelBtn = new QPushButton(content);
	m_exportBtn = new QPushButton(content);
	layout->addWidget(m_prelabelBtn);
	layout->addWidget(m_exportBtn);
	layout->addStretch(1);

	rebuildDefaultClasses();
	applyLanguage();

	connect(m_backendCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LabelingAnnotWidget::onBackendChanged);
	connect(m_refreshBackendsBtn, &QPushButton::clicked, this, &LabelingAnnotWidget::onRefreshBackendsClicked);
	connect(m_classTable, &QTableWidget::cellChanged, this, &LabelingAnnotWidget::onClassTableChanged);
	connect(m_classTable, &QTableWidget::currentCellChanged, this, &LabelingAnnotWidget::onClassRowActivated);
	connect(m_toolButtons, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, [this](QAbstractButton* btn) {
		if (!btn)
		{
			return;
		}
		onToolClicked(m_toolButtons->id(btn));
	});
	connect(m_undoBtn, &QPushButton::clicked, this, &LabelingAnnotWidget::onUndoClicked);
	connect(m_redoBtn, &QPushButton::clicked, this, &LabelingAnnotWidget::onRedoClicked);
	connect(m_exportBtn, &QPushButton::clicked, this, &LabelingAnnotWidget::onExportClicked);
	connect(m_prelabelBtn, &QPushButton::clicked, this, &LabelingAnnotWidget::onPrelabelClicked);
	connect(m_cancelPickBtn, &QPushButton::clicked, this, [this]() {
		if (m_labelingHost)
		{
			m_labelingHost->cancelActiveLabelingPick();
		}
	});
	if (m_labelingHost)
	{
		m_labelingHost->setPickCancelledNotifier([this]() {
			onToolPickCancelled();
		});
	}
	connect(m_classTable, &QTableWidget::cellDoubleClicked, this, [this](int row, int column) {
		if (column != 2 || row < 0)
		{
			return;
		}
		const QColor c = QColorDialog::getColor(Qt::gray, this, i18n(QStringLiteral("Pick color"), QStringLiteral("选择颜色")));
		if (!c.isValid())
		{
			return;
		}
		m_classTable->item(row, column)->setBackground(c);
		if (m_sessionId != kInvalidLabelingSessionId)
		{
			onClassTableChanged(row, column);
		}
	});

	refreshBackendList();
	m_clickTool->setChecked(true);
}

LabelingAnnotWidget::~LabelingAnnotWidget()
{
	if (m_labelingHost)
	{
		m_labelingHost->setPickCancelledNotifier(nullptr);
		m_labelingHost->abandonActiveLabelingPick();
	}
}

QString LabelingAnnotWidget::i18n(const QString& en, const QString& zh) const
{
	return m_host && m_host->useChinese() ? zh : en;
}

void LabelingAnnotWidget::applyLanguage()
{
	m_targetGroup->setTitle(i18n(QStringLiteral("Target"), QStringLiteral("目标对象")));
	m_refreshBackendsBtn->setText(i18n(QStringLiteral("Refresh"), QStringLiteral("刷新")));
	m_classGroup->setTitle(i18n(QStringLiteral("Classes"), QStringLiteral("类别表")));
	m_toolGroup->setTitle(i18n(QStringLiteral("Tools"), QStringLiteral("标注工具")));
	m_clickTool->setText(i18n(QStringLiteral("Click"), QStringLiteral("点选")));
	m_brushTool->setText(i18n(QStringLiteral("Brush"), QStringLiteral("刷选")));
	m_lassoTool->setText(i18n(QStringLiteral("Lasso"), QStringLiteral("套索")));
	m_eraseTool->setText(i18n(QStringLiteral("Erase"), QStringLiteral("擦除")));
	m_cancelPickBtn->setText(i18n(QStringLiteral("Cancel Pick"), QStringLiteral("取消选择")));
	m_undoBtn->setText(i18n(QStringLiteral("Undo"), QStringLiteral("撤销")));
	m_redoBtn->setText(i18n(QStringLiteral("Redo"), QStringLiteral("重做")));
	m_prelabelBtn->setText(i18n(QStringLiteral("PointNet Pre-label"), QStringLiteral("PointNet 预标注")));
	m_exportBtn->setText(i18n(QStringLiteral("Export Dataset"), QStringLiteral("导出数据集")));
	m_classTable->setHorizontalHeaderLabels(
		{ i18n(QStringLiteral("ID"), QStringLiteral("ID")),
			i18n(QStringLiteral("Name"), QStringLiteral("名称")),
			i18n(QStringLiteral("Color"), QStringLiteral("颜色")) });
	refreshSummary();
}

void LabelingAnnotWidget::rebuildDefaultClasses()
{
	const bool zh = m_host && m_host->useChinese();
	struct RowDef
	{
		int id;
		const char* en;
		const char* zhName;
		float rgb[3];
	};
	const RowDef defs[] = {
		{ 0, "background", "背景", { 0.35f, 0.35f, 0.35f } },
		{ 1, "main", "主体", { 0.20f, 0.60f, 0.90f } },
		{ 2, "part_a", "部件A", { 0.90f, 0.40f, 0.20f } },
		{ 3, "part_b", "部件B", { 0.30f, 0.80f, 0.30f } },
	};
	QSignalBlocker blocker(m_classTable);
	m_classTable->setRowCount(4);
	for (int row = 0; row < 4; ++row)
	{
		auto* idItem = new QTableWidgetItem(QString::number(defs[row].id));
		idItem->setFlags(idItem->flags() & ~Qt::ItemIsEditable);
		m_classTable->setItem(row, 0, idItem);
		const QString name = zh ? QString::fromUtf8(defs[row].zhName) : QString::fromLatin1(defs[row].en);
		m_classTable->setItem(row, 1, new QTableWidgetItem(name));
		auto* colorItem = new QTableWidgetItem;
		const QColor c = QColor::fromRgbF(defs[row].rgb[0], defs[row].rgb[1], defs[row].rgb[2]);
		colorItem->setBackground(c);
		colorItem->setFlags(colorItem->flags() & ~Qt::ItemIsEditable);
		m_classTable->setItem(row, 2, colorItem);
	}
}

void LabelingAnnotWidget::selectClassRowById(const int classId)
{
	for (int row = 0; row < m_classTable->rowCount(); ++row)
	{
		if (m_classTable->item(row, 0)->text().toInt() == classId)
		{
			QSignalBlocker blocker(m_classTable);
			m_classTable->setCurrentCell(row, 1);
			m_classTable->selectRow(row);
			return;
		}
	}
}

void LabelingAnnotWidget::refreshBackendList()
{
	if (!m_host)
	{
		return;
	}
	QSignalBlocker blocker(m_backendCombo);
	m_backendCombo->clear();
	IPluginDocument* doc = m_host->activeDocument();
	if (!doc)
	{
		clearSession();
		m_summaryLabel->setText(i18n(QStringLiteral("No active document."), QStringLiteral("无活动文档。")));
		return;
	}
	for (const std::string& id : doc->backendIds())
	{
		const std::string cls = doc->backendClassName(id);
		if (isLabelingTargetClass(cls))
		{
			const QString label = QString::fromStdString(doc->backendDisplayName(id)) + QStringLiteral(" [")
				+ QString::fromStdString(id) + QStringLiteral("]");
			m_backendCombo->addItem(label, QString::fromStdString(id));
		}
	}
	if (m_backendCombo->count() > 0)
	{
		onBackendChanged(m_backendCombo->currentIndex());
	}
	else
	{
		clearSession();
		m_summaryLabel->setText(i18n(QStringLiteral("No point cloud or mesh in document."),
									 QStringLiteral("文档中无点云或网格对象。")));
	}
}

void LabelingAnnotWidget::setLastExportDir(const QString& path)
{
	m_lastExportDir = path;
}

PluginLabelingSessionConfig LabelingAnnotWidget::buildSessionConfig() const
{
	PluginLabelingSessionConfig cfg;
	cfg.unlabeledClassId = 0;
	cfg.defaultBrushRadiusPx = static_cast<float>(m_brushRadiusSpin->value());
	for (int row = 0; row < m_classTable->rowCount(); ++row)
	{
		PluginLabelingClassDef c;
		c.classId = m_classTable->item(row, 0)->text().toInt();
		c.nameUtf8 = m_classTable->item(row, 1)->text().toUtf8().constData();
		const QColor color = m_classTable->item(row, 2)->background().color();
		c.colorRgb[0] = static_cast<float>(color.redF());
		c.colorRgb[1] = static_cast<float>(color.greenF());
		c.colorRgb[2] = static_cast<float>(color.blueF());
		cfg.classes.push_back(c);
	}
	return cfg;
}

void LabelingAnnotWidget::clearSession()
{
	if (m_labelingHost)
	{
		m_labelingHost->abandonActiveLabelingPick();
	}
	m_toolPickActive = false;
	if (m_labelingHost && m_sessionId != kInvalidLabelingSessionId)
	{
		m_labelingHost->clearLabelingSession(m_sessionId);
	}
	m_sessionId = kInvalidLabelingSessionId;
	m_backendId.clear();
}

bool LabelingAnnotWidget::ensureSession(QString* err)
{
	if (!m_host || !m_labelingHost)
	{
		if (err)
		{
			*err = i18n(QStringLiteral("Labeling host unavailable."), QStringLiteral("标注宿主不可用。"));
		}
		return false;
	}
	IPluginDocument* doc = m_host->activeDocument();
	if (!doc || m_backendId.empty())
	{
		if (err)
		{
			*err = i18n(QStringLiteral("Select a target object."), QStringLiteral("请选择目标对象。"));
		}
		return false;
	}
	if (m_sessionId != kInvalidLabelingSessionId)
	{
		return true;
	}
	const PluginLabelingSessionConfig cfg = buildSessionConfig();
	m_sessionId = m_labelingHost->beginLabelingSession(doc, m_backendId, cfg, err);
	if (m_sessionId == kInvalidLabelingSessionId)
	{
		return false;
	}
	PluginLabelingSessionSummary summary;
	if (m_labelingHost->getSessionSummary(m_sessionId, summary, err))
	{
		m_geometryKind = summary.geometryKind;
	}
	refreshSummary();
	return true;
}

void LabelingAnnotWidget::refreshSummary()
{
	if (!m_labelingHost || m_sessionId == kInvalidLabelingSessionId)
	{
		m_summaryLabel->setText(i18n(QStringLiteral("Session not started."), QStringLiteral("会话未启动。")));
		return;
	}
	PluginLabelingSessionSummary summary;
	QString err;
	if (!m_labelingHost->getSessionSummary(m_sessionId, summary, &err))
	{
		m_summaryLabel->setText(err);
		return;
	}
	const QString kind = summary.geometryKind == PluginLabelingGeometryKind::PointCloud
		? i18n(QStringLiteral("Point cloud"), QStringLiteral("点云"))
		: i18n(QStringLiteral("Mesh"), QStringLiteral("网格"));
	m_summaryLabel->setText(
		i18n(QStringLiteral("Type: %1 | Total: %2 | Labeled: %3 | Active class: %4"),
			 QStringLiteral("类型: %1 | 总数: %2 | 已标注: %3 | 当前类别: %4"))
			.arg(kind)
			.arg(static_cast<qulonglong>(summary.totalElements))
			.arg(static_cast<qulonglong>(summary.labeledElements))
			.arg(summary.activeClassId));
}

void LabelingAnnotWidget::onBackendChanged(int index)
{
	clearSession();
	if (index < 0)
	{
		return;
	}
	m_backendId = m_backendCombo->itemData(index).toString().toUtf8().constData();
	QString err;
	if (ensureSession(&err))
	{
		m_labelingHost->setActiveClass(m_sessionId, 1, &err);
		selectClassRowById(1);
		refreshSummary();
	}
	else if (!err.isEmpty())
	{
		m_summaryLabel->setText(err);
	}
}

void LabelingAnnotWidget::onClassTableChanged(const int row, const int column)
{
	if (m_sessionId == kInvalidLabelingSessionId || row < 0 || column == 0)
	{
		return;
	}
	QString err;
	if (!m_labelingHost->syncSessionConfig(m_sessionId, buildSessionConfig(), &err) && m_host && !err.isEmpty())
	{
		m_host->logWarn(err);
	}
}

void LabelingAnnotWidget::onClassRowActivated(const int row, const int column)
{
	(void)column;
	if (m_sessionId == kInvalidLabelingSessionId || row < 0)
	{
		return;
	}
	QTableWidgetItem* idItem = m_classTable->item(row, 0);
	if (!idItem)
	{
		return;
	}
	const int classId = idItem->text().toInt();
	QString err;
	if (!m_labelingHost->setActiveClass(m_sessionId, classId, &err))
	{
		if (m_host && !err.isEmpty())
		{
			m_host->logWarn(err);
		}
		return;
	}
	refreshSummary();
}

void LabelingAnnotWidget::onRefreshBackendsClicked()
{
	refreshBackendList();
}

void LabelingAnnotWidget::applySelection(const PluginLabelingSelectionResult& selection, bool erase)
{
	if (!m_labelingHost || m_sessionId == kInvalidLabelingSessionId)
	{
		return;
	}
	int classId = 1;
	if (m_classTable->currentRow() >= 0)
	{
		classId = m_classTable->item(m_classTable->currentRow(), 0)->text().toInt();
	}
	QString err;
	if (!m_labelingHost->applyLabels(m_sessionId, selection, classId, erase, &err))
	{
		if (m_host && !err.isEmpty())
		{
			m_host->logWarn(err);
		}
		return;
	}
	refreshSummary();
}

void LabelingAnnotWidget::onToolPickCancelled()
{
	m_toolPickActive = false;
	if (QAbstractButton* checked = m_toolButtons->checkedButton())
	{
		QSignalBlocker blocker(m_toolButtons);
		checked->setChecked(false);
	}
}

void LabelingAnnotWidget::checkToolButton(const PluginLabelingTool tool)
{
	if (QAbstractButton* btn = m_toolButtons->button(static_cast<int>(tool)))
	{
		QSignalBlocker blocker(m_toolButtons);
		btn->setChecked(true);
	}
}

void LabelingAnnotWidget::armActiveTool()
{
	if (!m_toolPickActive || m_sessionId == kInvalidLabelingSessionId)
	{
		return;
	}
	activateTool(m_activeTool);
}

void LabelingAnnotWidget::activateTool(PluginLabelingTool tool)
{
	if (!ensureSession())
	{
		return;
	}
	if (m_labelingHost)
	{
		m_labelingHost->abandonActiveLabelingPick();
	}
	m_activeTool = tool;
	m_eraseMode = (tool == PluginLabelingTool::Erase);
	m_toolPickActive = true;
	checkToolButton(tool);

	const float radius = static_cast<float>(m_brushRadiusSpin->value());
	const bool erase = m_eraseMode;

	if (tool == PluginLabelingTool::Click || tool == PluginLabelingTool::Erase)
	{
		const auto onFinished = [this, erase](bool ok, const QString& error, const PluginLabelingSelectionResult& result) {
			if (!m_toolPickActive)
			{
				return;
			}
			if (ok)
			{
				applySelection(result, erase);
			}
			else if (m_host && !error.isEmpty()
				&& error != QStringLiteral("No point hit")
				&& error != QStringLiteral("No face hit"))
			{
				m_host->logWarn(error);
			}
		};
		if (m_geometryKind == PluginLabelingGeometryKind::TriangleMesh)
		{
			m_labelingHost->pickMeshFaceOnce(m_sessionId, onFinished);
		}
		else
		{
			m_labelingHost->pickPointsOnce(m_sessionId, onFinished);
		}
		return;
	}

	if (tool == PluginLabelingTool::Brush)
	{
		const auto onStroke = [this](const PluginLabelingSelectionResult& stroke) {
			applySelection(stroke, m_eraseMode);
			refreshSummary();
		};
		const auto onFinished = [](bool, const QString&, const PluginLabelingSelectionResult&) {
		};
		if (m_geometryKind == PluginLabelingGeometryKind::TriangleMesh)
		{
			m_labelingHost->brushMeshFaces(m_sessionId, radius, onStroke, onFinished);
		}
		else
		{
			m_labelingHost->brushStroke(m_sessionId, radius, onStroke, onFinished);
		}
		return;
	}

	if (tool == PluginLabelingTool::Polyline)
	{
		m_labelingHost->pickPolylineRegion(m_sessionId, [this](bool ok, const QString& error, const PluginLabelingSelectionResult& result) {
			if (!m_toolPickActive)
			{
				return;
			}
			if (!ok)
			{
				if (error == QStringLiteral("Polyline pick canceled"))
				{
					onToolPickCancelled();
				}
				else if (m_host && !error.isEmpty())
				{
					m_host->logWarn(error);
				}
				return;
			}
			applySelection(result, m_eraseMode);
			refreshSummary();
			QMetaObject::invokeMethod(this, [this]() { armActiveTool(); }, Qt::QueuedConnection);
		});
	}
}

void LabelingAnnotWidget::onToolClicked(int toolId)
{
	activateTool(static_cast<PluginLabelingTool>(toolId));
}

void LabelingAnnotWidget::onUndoClicked()
{
	if (!m_labelingHost || m_sessionId == kInvalidLabelingSessionId)
	{
		return;
	}
	QString err;
	if (!m_labelingHost->undo(m_sessionId, &err) && m_host && !err.isEmpty())
	{
		m_host->logWarn(err);
	}
	refreshSummary();
}

void LabelingAnnotWidget::onRedoClicked()
{
	if (!m_labelingHost || m_sessionId == kInvalidLabelingSessionId)
	{
		return;
	}
	QString err;
	if (!m_labelingHost->redo(m_sessionId, &err) && m_host && !err.isEmpty())
	{
		m_host->logWarn(err);
	}
	refreshSummary();
}

bool LabelingAnnotWidget::extractBackendPoints(const std::string& backendId, std::vector<float>& outPoints, int& outCount) const
{
	if (!m_host)
	{
		return false;
	}
	IPluginDocument* doc = m_host->activeDocument();
	if (!doc)
	{
		return false;
	}
	const QString tmpPath = QDir::tempPath() + QStringLiteral("/labeling_prelabel_export.ply");
	const std::string tmpUtf8 = tmpPath.toUtf8().constData();
	if (!doc->exportMeshToPly(backendId, tmpUtf8, nullptr))
	{
		return false;
	}
	const bool ok = parseAsciiPlyPoints(tmpPath, outPoints, outCount);
	QFile::remove(tmpPath);
	return ok;
}

bool LabelingAnnotWidget::loadPointNetSegmentModel(QString* err)
{
	const QString configPath = findPointNetConfigPath(m_host);
	if (configPath.isEmpty())
	{
		if (err)
		{
			*err = i18n(QStringLiteral("pointnet_config.json not found."), QStringLiteral("未找到 pointnet_config.json。"));
		}
		return false;
	}
	QFile f(configPath);
	if (!f.open(QIODevice::ReadOnly))
	{
		if (err)
		{
			*err = i18n(QStringLiteral("Cannot open config."), QStringLiteral("无法打开配置文件。"));
		}
		return false;
	}
	try
	{
		const nlohmann::json cfg = nlohmann::json::parse(f.readAll().constData(), nullptr, true);
		if (!cfg.contains("models") || !cfg["models"].contains("segment"))
		{
			if (err)
			{
				*err = i18n(QStringLiteral("No segment model in config."), QStringLiteral("配置中无分割模型。"));
			}
			return false;
		}
		const auto& seg = cfg["models"]["segment"];
		QString modelPath = QString::fromStdString(seg["path"].get<std::string>());
		if (QFileInfo(modelPath).isRelative())
		{
			modelPath = QDir::cleanPath(QFileInfo(configPath).absolutePath() + QStringLiteral("/") + modelPath);
		}
		const int numPoints = seg.value("num_points", 2048);
		const int numClasses = seg.value("num_classes", 4);
		return m_inference->loadSegmentModel(modelPath, numPoints, numClasses, err);
	}
	catch (const std::exception& e)
	{
		if (err)
		{
			*err = QString::fromStdString(e.what());
		}
		return false;
	}
}

void LabelingAnnotWidget::onPrelabelClicked()
{
	if (!ensureSession())
	{
		return;
	}
	QString err;
	if (!loadPointNetSegmentModel(&err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Pre-label"), QStringLiteral("预标注")), err);
		return;
	}
	std::vector<float> points;
	int pointCount = 0;
	if (!extractBackendPoints(m_backendId, points, pointCount) || pointCount <= 0)
	{
		QMessageBox::warning(
			this,
			i18n(QStringLiteral("Pre-label"), QStringLiteral("预标注")),
			i18n(QStringLiteral("Failed to extract points."), QStringLiteral("提取点云失败。")));
		return;
	}
	const PointNetSegmentResult result = m_inference->segment(points, pointCount);
	if (result.labels.empty())
	{
		QMessageBox::warning(
			this,
			i18n(QStringLiteral("Pre-label"), QStringLiteral("预标注")),
			i18n(QStringLiteral("Inference returned no labels."), QStringLiteral("推理未返回标签。")));
		return;
	}
	std::vector<int> fullLabels(static_cast<std::size_t>(pointCount), 0);
	const int sampleCount = static_cast<int>(result.labels.size());
	const float step = static_cast<float>(pointCount) / static_cast<float>(sampleCount);
	for (int i = 0; i < pointCount; ++i)
	{
		const int srcIdx = std::min(static_cast<int>(static_cast<float>(i) / step), sampleCount - 1);
		fullLabels[static_cast<std::size_t>(i)] = result.labels[static_cast<std::size_t>(srcIdx)];
	}
	if (!m_labelingHost->importPerPointLabels(m_sessionId, fullLabels, result.numClasses, &err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Pre-label"), QStringLiteral("预标注")), err);
		return;
	}
	m_labelingHost->syncLabelVisualization(m_sessionId, &err);
	refreshSummary();
	if (m_host)
	{
		m_host->logInfo(i18n(QStringLiteral("PointNet pre-label applied."), QStringLiteral("PointNet 预标注已应用。")));
	}
}

void LabelingAnnotWidget::onExportClicked()
{
	if (!ensureSession())
	{
		return;
	}
	const QString dir = QFileDialog::getExistingDirectory(
		this,
		i18n(QStringLiteral("Export dataset"), QStringLiteral("导出数据集")),
		m_lastExportDir.isEmpty() ? QDir::homePath() : m_lastExportDir);
	if (dir.isEmpty())
	{
		return;
	}
	const QString sampleBase = nextExportSampleBaseNameFromDir(dir);
	PluginLabelingDatasetExportOptions opts;
	opts.sampleNameUtf8 = sampleBase.toUtf8().constData();
	opts.numClasses = m_classTable->rowCount();
	opts.meshSampleCount = 2048;
	PluginLabelingDatasetExportResult result;
	QString err;
	if (!m_labelingHost->exportPointNetDataset(m_sessionId, dir.toUtf8().constData(), opts, result, &err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Export"), QStringLiteral("导出")), err);
		return;
	}
	m_lastExportDir = dir;
	emit datasetExported(dir);
	if (m_host)
	{
		m_host->logInfo(
			i18n(QStringLiteral("Dataset exported: %1 -> %2").arg(sampleBase, dir),
				 QStringLiteral("数据集已导出：%1 -> %2").arg(sampleBase, dir)));
	}
}
