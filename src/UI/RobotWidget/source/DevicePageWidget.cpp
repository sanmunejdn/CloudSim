/// @file DevicePageWidget.cpp
/// @brief DevicePage 控件

#include "DevicePageWidget.h"

#include "UiIconDecorators.h"
#include "UiIcons.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QEvent>
#include <QFileInfo>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSet>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>

namespace
{
constexpr int kTileWidth = 96;
constexpr int kTileHeight = 88;
constexpr int kIconSize = 48;
constexpr int kGridSpacing = 6;

QString applicationDirWithResourceFolder()
{
	const QString appDir = QCoreApplication::applicationDirPath();
	if (QDir(QDir(appDir).filePath(QStringLiteral("resource"))).exists())
	{
		return appDir;
	}
	const QString parent = QDir(appDir).absoluteFilePath(QStringLiteral(".."));
	if (QDir(QDir(parent).filePath(QStringLiteral("resource"))).exists())
	{
		return QDir(parent).absolutePath();
	}
	return appDir;
}

static const QStringList kImageSuffixes{QStringLiteral("png"), QStringLiteral("jpg"),  QStringLiteral("jpeg"),
										QStringLiteral("bmp"), QStringLiteral("webp"), QStringLiteral("gif")};

QStringList listUrdfFilesInPackage(const QString& packageRoot)
{
	const QStringList nameFilters{QStringLiteral("*.urdf"), QStringLiteral("*.urdf.txt")};
	QStringList out;

	const QDir urdfDir(QDir(packageRoot).filePath(QStringLiteral("urdf")));
	if (urdfDir.exists())
	{
		const QStringList names = urdfDir.entryList(nameFilters, QDir::Files, QDir::Name);
		for (const QString& n : names)
		{
			out.push_back(urdfDir.absoluteFilePath(n));
		}
	}

	if (out.isEmpty())
	{
		const QDir rootDir(packageRoot);
		if (rootDir.exists())
		{
			const QStringList names = rootDir.entryList(nameFilters, QDir::Files, QDir::Name);
			for (const QString& n : names)
			{
				out.push_back(rootDir.absoluteFilePath(n));
			}
		}
	}
	return out;
}

QString pickUrdfForImageBase(const QString& imageBase, const QStringList& urdfAbsPaths)
{
	if (urdfAbsPaths.isEmpty())
	{
		return QString();
	}
	if (urdfAbsPaths.size() == 1U)
	{
		return urdfAbsPaths.front();
	}
	for (const QString& p : urdfAbsPaths)
	{
		const QString base = QFileInfo(p).completeBaseName();
		if (base.compare(imageBase, Qt::CaseInsensitive) == 0)
		{
			return p;
		}
	}
	return urdfAbsPaths.front();
}

bool isNonDevicePackageFolderName(const QString& folderName)
{
	const QString n = folderName.toLower();
	static const QStringList kSkip{
		QStringLiteral("urdf"),		 QStringLiteral("meshes"),	  QStringLiteral("mesh"),	  QStringLiteral("config"),
		QStringLiteral("launch"),	 QStringLiteral("materials"), QStringLiteral("textures"), QStringLiteral("cad"),
		QStringLiteral("collision"), QStringLiteral("visual"),	  QStringLiteral("include"),  QStringLiteral("worlds"),
		QStringLiteral("maps"),		 QStringLiteral("rviz"),	  QStringLiteral("test"),	  QStringLiteral("tests"),
	};
	return kSkip.contains(n);
}

QStringList findDevicePackageRoots(const QString& modelsRoot)
{
	QStringList roots;
	if (!QDir(modelsRoot).exists())
	{
		return roots;
	}

	QSet<QString> seenCanon;
	QDirIterator it(modelsRoot, QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
	while (it.hasNext())
	{
		const QString dirPath = QDir::cleanPath(it.next());
		if (dirPath.compare(QDir::cleanPath(modelsRoot), Qt::CaseInsensitive) == 0)
		{
			continue;
		}
		const QString leaf = QFileInfo(dirPath).fileName();
		if (isNonDevicePackageFolderName(leaf))
		{
			continue;
		}
		if (listUrdfFilesInPackage(dirPath).isEmpty())
		{
			continue;
		}
		const QString canon = QFileInfo(dirPath).canonicalFilePath();
		const QString key = !canon.isEmpty() ? canon : dirPath;
		if (seenCanon.contains(key))
		{
			continue;
		}
		seenCanon.insert(key);
		roots.append(key);
	}
	std::sort(roots.begin(), roots.end());
	return roots;
}

void classifyPackagePath(const QString& packageRoot, const QString& modelsRoot, QString& outType, QString& outBrand)
{
	const QDir base(modelsRoot);
	const QString rel = QDir::cleanPath(base.relativeFilePath(packageRoot));
	static const QString kNoBrand = QStringLiteral("未分类");
	if (rel == QLatin1String(".") || rel.isEmpty())
	{
		outType = QStringLiteral("?");
		outBrand = kNoBrand;
		return;
	}
	const QStringList parts = rel.split(QRegularExpression(QStringLiteral("[/\\\\]")), Qt::SkipEmptyParts);
	if (parts.size() >= 3)
	{
		outType = parts[0];
		outBrand = parts[1];
	}
	else if (parts.size() == 2)
	{
		outType = parts[0];
		outBrand = kNoBrand;
	}
	else if (parts.size() == 1)
	{
		outType = parts[0];
		outBrand = kNoBrand;
	}
	else
	{
		outType = QStringLiteral("?");
		outBrand = kNoBrand;
	}
}

QStringList orderedCategoryKeys(const QStringList& categories)
{
	static const QStringList kPreferred{QStringLiteral("Robot"), QStringLiteral("AGV"), QStringLiteral("Worker")};
	QStringList out;
	QStringList rest = categories;
	for (const QString& p : kPreferred)
	{
		if (rest.contains(p))
		{
			out.append(p);
			rest.removeAll(p);
		}
	}
	std::sort(rest.begin(), rest.end());
	out.append(rest);
	return out;
}

QString elidedButtonText(QToolButton* btn, const QString& text)
{
	if (!btn)
	{
		return text;
	}
	const int maxWidth = kTileWidth - 8;
	return btn->fontMetrics().elidedText(text, Qt::ElideRight, maxWidth);
}

} // namespace

DevicePageWidget::DevicePageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	setupUi(root);

	const QString resourceBase = applicationDirWithResourceFolder();
	setModelsRootPath(QDir(resourceBase).filePath(QStringLiteral("resource/models")));
	setUseChinese(true);
}

void DevicePageWidget::setupUi(QVBoxLayout* rootLayout)
{
	auto* filterRow = new QHBoxLayout;
	filterRow->setContentsMargins(0, 0, 0, 0);
	filterRow->setSpacing(6);

	m_typeLabel = new QLabel(this);
	m_typeCombo = new QComboBox(this);
	m_typeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

	m_brandLabel = new QLabel(this);
	m_brandCombo = new QComboBox(this);
	m_brandCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

	m_refreshBtn = new QPushButton(QStringLiteral("↻"), this);
	m_refreshBtn->setFixedSize(28, 28);
	m_refreshBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

	filterRow->addWidget(m_typeLabel);
	filterRow->addWidget(m_typeCombo, 1);
	filterRow->addWidget(m_brandLabel);
	filterRow->addWidget(m_brandCombo, 1);
	filterRow->addWidget(m_refreshBtn);

	m_customDeviceBtn = new QPushButton(this);
	m_customDeviceBtn->setProperty("btnRole", QStringLiteral("secondary"));
	m_editCustomDeviceBtn = new QPushButton(this);
	m_editCustomDeviceBtn->setProperty("btnRole", QStringLiteral("secondary"));
	m_exportCustomDeviceUrdfBtn = new QPushButton(this);
	m_exportCustomDeviceUrdfBtn->setProperty("btnRole", QStringLiteral("secondary"));

	m_modelsScroll = new QScrollArea(this);
	m_modelsScroll->setWidgetResizable(true);
	m_modelsScroll->setFrameShape(QFrame::NoFrame);
	m_modelsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	m_modelsContainer = new QWidget;
	m_modelsGrid = new QGridLayout(m_modelsContainer);
	m_modelsGrid->setContentsMargins(0, 0, 0, 0);
	m_modelsGrid->setHorizontalSpacing(kGridSpacing);
	m_modelsGrid->setVerticalSpacing(kGridSpacing);
	m_modelsGrid->setAlignment(Qt::AlignTop | Qt::AlignLeft);

	m_statusLabel = new QLabel(m_modelsContainer);
	m_statusLabel->setWordWrap(true);
	m_statusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
	m_statusLabel->hide();
	m_modelsGrid->addWidget(m_statusLabel, 0, 0, 1, 1);

	m_modelsScroll->setWidget(m_modelsContainer);
	if (QWidget* viewport = m_modelsScroll->viewport())
	{
		viewport->installEventFilter(this);
	}

	connect(m_typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&DevicePageWidget::onTypeSelectionChanged);
	connect(m_brandCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&DevicePageWidget::onBrandSelectionChanged);
	connect(m_refreshBtn, &QPushButton::clicked, this, &DevicePageWidget::onRefreshClicked);
	connect(m_customDeviceBtn, &QPushButton::clicked, this, &DevicePageWidget::customDeviceCreateRequested);
	connect(m_editCustomDeviceBtn, &QPushButton::clicked, this, &DevicePageWidget::customDeviceEditRequested);
	connect(m_exportCustomDeviceUrdfBtn, &QPushButton::clicked, this, &DevicePageWidget::customDeviceExportUrdfRequested);

	if (rootLayout)
	{
		rootLayout->addLayout(filterRow);
		rootLayout->addWidget(m_customDeviceBtn);
		rootLayout->addWidget(m_editCustomDeviceBtn);
		rootLayout->addWidget(m_exportCustomDeviceUrdfBtn);
		rootLayout->addWidget(m_modelsScroll, 1);
	}
}

void DevicePageWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	updateUiLabels();
}

void DevicePageWidget::updateUiLabels()
{
	const bool zh = m_useChinese;
	if (m_typeLabel)
	{
		m_typeLabel->setText(zh ? QStringLiteral("类型") : QStringLiteral("Type"));
	}
	if (m_brandLabel)
	{
		m_brandLabel->setText(zh ? QStringLiteral("品牌") : QStringLiteral("Brand"));
	}
	if (m_refreshBtn)
	{
		m_refreshBtn->setToolTip(zh ? QStringLiteral("重新扫描设备包") : QStringLiteral("Rescan device packages"));
	}
	if (m_customDeviceBtn)
	{
		m_customDeviceBtn->setText(zh ? QStringLiteral("新建自定义设备…") : QStringLiteral("New Custom Device…"));
		m_customDeviceBtn->setToolTip(zh ? QStringLiteral("导入模型并在画布上定义运动副")
										 : QStringLiteral("Import models and define joints on canvas"));
	}
	if (m_editCustomDeviceBtn)
	{
		m_editCustomDeviceBtn->setText(zh ? QStringLiteral("编辑自定义设备…") : QStringLiteral("Edit Custom Device…"));
		m_editCustomDeviceBtn->setToolTip(zh ? QStringLiteral("打开已有设备的组装画布继续修改")
											: QStringLiteral("Reopen assembly canvas for an existing device"));
	}
	if (m_exportCustomDeviceUrdfBtn)
	{
		m_exportCustomDeviceUrdfBtn->setText(zh ? QStringLiteral("导出 URDF…") : QStringLiteral("Export URDF…"));
		m_exportCustomDeviceUrdfBtn->setToolTip(
			zh ? QStringLiteral("导出为带 package.xml 的 ROS 包，可用导入 URDF 回灌")
			   : QStringLiteral("Export a ROS package with package.xml for URDF re-import"));
	}
}

void DevicePageWidget::setModelsRootPath(const QString& absoluteDirPath)
{
	m_modelsRoot = QDir::cleanPath(absoluteDirPath);
	rescanPackagesAndRefreshUi();
}

void DevicePageWidget::refreshButtons()
{
	rescanPackagesAndRefreshUi();
}

void DevicePageWidget::onRefreshClicked()
{
	rescanPackagesAndRefreshUi();
}

QString DevicePageWidget::selectedType() const
{
	if (!m_typeCombo || m_typeCombo->currentIndex() < 0)
	{
		return {};
	}
	return m_typeCombo->currentText();
}

QString DevicePageWidget::selectedBrand() const
{
	if (!m_brandCombo || m_brandCombo->currentIndex() < 0)
	{
		return {};
	}
	if (!m_brandCombo->isVisible())
	{
		const QString type = selectedType();
		if (!m_packagesByTypeBrand.contains(type))
		{
			return {};
		}
		const QStringList brands = m_packagesByTypeBrand[type].keys();
		return brands.isEmpty() ? QString() : brands.front();
	}
	return m_brandCombo->currentText();
}

void DevicePageWidget::rescanPackagesAndRefreshUi()
{
	m_packagesByTypeBrand.clear();

	const QStringList roots = findDevicePackageRoots(m_modelsRoot);
	for (const QString& root : roots)
	{
		QString t;
		QString b;
		classifyPackagePath(root, m_modelsRoot, t, b);
		m_packagesByTypeBrand[t][b].append(root);
	}
	for (auto it = m_packagesByTypeBrand.begin(); it != m_packagesByTypeBrand.end(); ++it)
	{
		QMap<QString, QStringList>& inner = it.value();
		for (auto jt = inner.begin(); jt != inner.end(); ++jt)
		{
			QStringList& lst = jt.value();
			std::sort(lst.begin(), lst.end());
			const auto last = std::unique(lst.begin(), lst.end());
			lst.erase(last, lst.end());
		}
	}

	fillTypeCombo();

	{
		QSignalBlocker bt(m_typeCombo);
		QSignalBlocker bb(m_brandCombo);
		if (m_typeCombo->count() > 0)
		{
			m_typeCombo->setCurrentIndex(0);
		}
		fillBrandComboForSelectedType();
		if (m_brandCombo->count() > 0)
		{
			m_brandCombo->setCurrentIndex(0);
		}
	}
	updateBrandComboVisibility();
	rebuildModelTiles();

	if (m_typeCombo->count() == 0)
	{
		m_statusLabel->setText(m_useChinese ? QStringLiteral("未找到含 URDF 的设备包。\n%1").arg(m_modelsRoot)
											: QStringLiteral("No URDF device packages found.\n%1").arg(m_modelsRoot));
		m_statusLabel->show();
	}
}

void DevicePageWidget::fillTypeCombo()
{
	m_typeCombo->clear();
	QStringList keys;
	keys.reserve(m_packagesByTypeBrand.size());
	for (auto it = m_packagesByTypeBrand.constBegin(); it != m_packagesByTypeBrand.constEnd(); ++it)
	{
		keys.append(it.key());
	}
	const QStringList ordered = orderedCategoryKeys(keys);
	for (const QString& k : ordered)
	{
		if (!m_packagesByTypeBrand.contains(k))
		{
			continue;
		}
		m_typeCombo->addItem(k);
	}
}

void DevicePageWidget::fillBrandComboForSelectedType()
{
	m_brandCombo->clear();
	const QString type = selectedType();
	if (type.isEmpty() || !m_packagesByTypeBrand.contains(type))
	{
		return;
	}
	QStringList brands = m_packagesByTypeBrand[type].keys();
	std::sort(brands.begin(), brands.end());
	for (const QString& b : brands)
	{
		m_brandCombo->addItem(b);
	}
}

void DevicePageWidget::updateBrandComboVisibility()
{
	const QString type = selectedType();
	const int brandCount =
		type.isEmpty() || !m_packagesByTypeBrand.contains(type) ? 0 : m_packagesByTypeBrand[type].size();
	const bool showBrand = brandCount > 1;
	m_brandLabel->setVisible(showBrand);
	m_brandCombo->setVisible(showBrand);
}

void DevicePageWidget::onTypeSelectionChanged()
{
	QSignalBlocker bb(m_brandCombo);
	fillBrandComboForSelectedType();
	if (m_brandCombo->count() > 0)
	{
		m_brandCombo->setCurrentIndex(0);
	}
	updateBrandComboVisibility();
	rebuildModelTiles();
}

void DevicePageWidget::onBrandSelectionChanged()
{
	rebuildModelTiles();
}

void DevicePageWidget::rebuildModelTiles()
{
	for (QToolButton* btn : m_modelButtons)
	{
		btn->deleteLater();
	}
	m_modelButtons.clear();
	m_statusLabel->hide();

	while (QLayoutItem* item = m_modelsGrid->takeAt(0))
	{
		if (QWidget* w = item->widget())
		{
			if (w != m_statusLabel)
			{
				w->deleteLater();
			}
		}
		delete item;
	}
	m_modelsGrid->addWidget(m_statusLabel, 0, 0, 1, 1);
	m_statusLabel->hide();

	const QString type = selectedType();
	const QString brand = selectedBrand();
	if (type.isEmpty() || brand.isEmpty())
	{
		return;
	}
	if (!m_packagesByTypeBrand.contains(type) || !m_packagesByTypeBrand[type].contains(brand))
	{
		return;
	}

	const QStringList packages = m_packagesByTypeBrand[type][brand];

	auto makeTile = [&](const QString& labelText, const QString& urdfPath, const QIcon& icon)
	{
		auto* btn = new QToolButton(m_modelsContainer);
		btn->setFixedSize(kTileWidth, kTileHeight);
		btn->setIconSize(QSize(kIconSize, kIconSize));
		btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
		btn->setIcon(icon);
		btn->setText(elidedButtonText(btn, labelText));
		btn->setToolTip(QStringLiteral("%1\n%2").arg(labelText, urdfPath));
		connect(btn, &QToolButton::clicked, this, [this, urdfPath]() { emit urdfImportRequested(urdfPath); });
		m_modelButtons.append(btn);
	};

	for (const QString& packageRoot : packages)
	{
		const QStringList urdfs = listUrdfFilesInPackage(packageRoot);
		if (urdfs.isEmpty())
		{
			continue;
		}

		QStringList images;
		const QDir rootDir(packageRoot);
		if (rootDir.exists())
		{
			const QFileInfoList files = rootDir.entryInfoList(QDir::Files);
			for (const QFileInfo& fi : files)
			{
				if (kImageSuffixes.contains(fi.suffix().toLower()))
				{
					images.append(fi.absoluteFilePath());
				}
			}
		}

		if (images.isEmpty())
		{
			const QString label = QFileInfo(packageRoot).fileName();
			const QString urdfPath = urdfs.front();
			makeTile(label, urdfPath, UiIcons::icon(UiIconId::RobotPlaceholder, UiIcons::Size::Medium));
			continue;
		}

		for (const QString& imgPath : images)
		{
			const QString urdfPath = pickUrdfForImageBase(QFileInfo(imgPath).completeBaseName(), urdfs);
			if (urdfPath.isEmpty())
			{
				continue;
			}
			const QString label = QFileInfo(imgPath).completeBaseName();
			QPixmap pm(imgPath);
			QIcon icon;
			if (!pm.isNull())
			{
				icon = QIcon(pm.scaled(kIconSize, kIconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
			}
			makeTile(label, urdfPath, icon);
		}
	}

	if (m_modelButtons.isEmpty())
	{
		m_statusLabel->setText(m_useChinese ? QStringLiteral("此品牌下暂无可用型号。")
											: QStringLiteral("No models under this brand."));
		m_statusLabel->show();
	}

	relayoutModelGrid();
}

void DevicePageWidget::relayoutModelGrid()
{
	while (QLayoutItem* item = m_modelsGrid->takeAt(0))
	{
		delete item;
	}

	if (m_modelButtons.isEmpty())
	{
		if (m_statusLabel->isVisible())
		{
			const int viewportWidth =
				m_modelsScroll && m_modelsScroll->viewport() ? m_modelsScroll->viewport()->width() : width();
			const int columns = qMax(1, viewportWidth / (kTileWidth + kGridSpacing));
			m_modelsGrid->addWidget(m_statusLabel, 0, 0, 1, columns);
		}
		return;
	}

	const int viewportWidth =
		m_modelsScroll && m_modelsScroll->viewport() ? m_modelsScroll->viewport()->width() : width();
	const int columns = qMax(1, viewportWidth / (kTileWidth + kGridSpacing));

	for (int i = 0; i < m_modelButtons.size(); ++i)
	{
		const int row = i / columns;
		const int col = i % columns;
		m_modelsGrid->addWidget(m_modelButtons[i], row, col);
	}
}

void DevicePageWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	relayoutModelGrid();
}

bool DevicePageWidget::eventFilter(QObject* watched, QEvent* event)
{
	if (m_modelsScroll && watched == m_modelsScroll->viewport() && event->type() == QEvent::Resize)
	{
		relayoutModelGrid();
	}
	return QWidget::eventFilter(watched, event);
}
