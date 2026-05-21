#include "DevicePageWidget.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QPixmap>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>
#include <QSet>

#include <algorithm>

namespace
{
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

static const QStringList kImageSuffixes{
	QStringLiteral("png"), QStringLiteral("jpg"), QStringLiteral("jpeg"),
	QStringLiteral("bmp"), QStringLiteral("webp"), QStringLiteral("gif")
};

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
		QStringLiteral("urdf"), QStringLiteral("meshes"), QStringLiteral("mesh"),
		QStringLiteral("config"), QStringLiteral("launch"), QStringLiteral("materials"),
		QStringLiteral("textures"), QStringLiteral("cad"), QStringLiteral("collision"),
		QStringLiteral("visual"), QStringLiteral("include"), QStringLiteral("worlds"),
		QStringLiteral("maps"), QStringLiteral("rviz"), QStringLiteral("test"),
		QStringLiteral("tests"),
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
	// "未分类" — avoid u8 inside QStringLiteral for MSVC
	static const QString kNoBrand = QStringLiteral("未分类");
	if (rel == QLatin1String(".") || rel.isEmpty())
	{
		outType = QStringLiteral("?");
		outBrand = kNoBrand;
		return;
	}
	const QStringList parts =
		rel.split(QRegularExpression(QStringLiteral("[/\\\\]")), Qt::SkipEmptyParts);
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
	static const QStringList kPreferred{
		QStringLiteral("Robot"), QStringLiteral("AGV"), QStringLiteral("Worker")
	};
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

} // namespace

DevicePageWidget::DevicePageWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(8);
	auto* hint = new QLabel(QStringLiteral(
		"从 resource/models 扫描设备包。左侧选择设备类型与品牌，右侧点击型号缩略图导入 URDF。"));
	hint->setWordWrap(true);
	hint->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 11px;"));
	hint->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
	root->addWidget(hint);

	setupDeviceColumns(root);

	const QString resourceBase = applicationDirWithResourceFolder();
	setModelsRootPath(QDir(resourceBase).filePath(QStringLiteral("resource/models")));
}

void DevicePageWidget::setupDeviceColumns(QVBoxLayout* rootLayout)
{
	auto* row = new QWidget(this);
	auto* h = new QHBoxLayout(row);
	h->setContentsMargins(0, 0, 0, 0);
	h->setSpacing(10);

	auto makeColumn = [&](const QString& title, int stretch) -> QWidget* {
		auto* col = new QWidget(row);
		auto* v = new QVBoxLayout(col);
		v->setContentsMargins(0, 0, 0, 0);
		v->setSpacing(4);
		auto* lab = new QLabel(title);
		lab->setStyleSheet(QStringLiteral("font-weight: bold;"));
		v->addWidget(lab);
		return col;
	};

	auto* colType = makeColumn(QStringLiteral("设备类型"), 1);
	auto* colBrand = makeColumn(QStringLiteral("设备品牌"), 1);
	auto* colModel = makeColumn(QStringLiteral("具体型号"), 3);

	m_listType = new QListWidget;
	m_listType->setMinimumWidth(120);
	m_listType->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
	static_cast<QVBoxLayout*>(colType->layout())->addWidget(m_listType, 1);

	m_listBrand = new QListWidget;
	m_listBrand->setMinimumWidth(120);
	m_listBrand->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
	static_cast<QVBoxLayout*>(colBrand->layout())->addWidget(m_listBrand, 1);

	m_modelsScroll = new QScrollArea;
	m_modelsScroll->setWidgetResizable(true);
	m_modelsScroll->setFrameShape(QFrame::StyledPanel);
	m_modelsContainer = new QWidget;
	m_modelsLayout = new QVBoxLayout(m_modelsContainer);
	m_modelsLayout->setContentsMargins(4, 4, 4, 4);
	m_modelsLayout->setSpacing(10);
	m_modelsLayout->setAlignment(Qt::AlignTop);
	m_modelsScroll->setWidget(m_modelsContainer);
	static_cast<QVBoxLayout*>(colModel->layout())->addWidget(m_modelsScroll, 1);

	h->addWidget(colType, 1);
	h->addWidget(colBrand, 1);
	h->addWidget(colModel, 2);

	connect(m_listType, &QListWidget::currentItemChanged, this, &DevicePageWidget::onTypeSelectionChanged);
	connect(m_listBrand, &QListWidget::currentItemChanged, this, &DevicePageWidget::onBrandSelectionChanged);

	if (rootLayout)
	{
		rootLayout->addWidget(row, 1);
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

	fillTypeList();

	{
		QSignalBlocker bt(m_listType);
		QSignalBlocker bb(m_listBrand);
		if (m_listType->count() > 0)
		{
			m_listType->setCurrentRow(0);
		}
		fillBrandListForSelectedType();
		if (m_listBrand->count() > 0)
		{
			m_listBrand->setCurrentRow(0);
		}
	}
	fillModelGridForSelection();

	if (m_listType->count() == 0)
	{
		QLayoutItem* lit;
		while ((lit = m_modelsLayout->takeAt(0)) != nullptr)
		{
			if (QWidget* w = lit->widget())
			{
				w->deleteLater();
			}
			delete lit;
		}
		auto* empty = new QLabel(
			QStringLiteral("未找到含 URDF 的设备包。\n%1").arg(m_modelsRoot));
		empty->setWordWrap(true);
		m_modelsLayout->addWidget(empty);
	}
}

void DevicePageWidget::fillTypeList()
{
	m_listType->clear();
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
		m_listType->addItem(k);
	}
}

void DevicePageWidget::fillBrandListForSelectedType()
{
	m_listBrand->clear();
	QListWidgetItem* cur = m_listType->currentItem();
	if (!cur)
	{
		return;
	}
	const QString type = cur->text();
	if (!m_packagesByTypeBrand.contains(type))
	{
		return;
	}
	QStringList brands = m_packagesByTypeBrand[type].keys();
	std::sort(brands.begin(), brands.end());
	for (const QString& b : brands)
	{
		m_listBrand->addItem(b);
	}
}

void DevicePageWidget::onTypeSelectionChanged()
{
	QSignalBlocker bb(m_listBrand);
	fillBrandListForSelectedType();
	if (m_listBrand->count() > 0)
	{
		m_listBrand->setCurrentRow(0);
	}
	fillModelGridForSelection();
}

void DevicePageWidget::onBrandSelectionChanged()
{
	fillModelGridForSelection();
}

void DevicePageWidget::fillModelGridForSelection()
{
	QLayoutItem* lit;
	while ((lit = m_modelsLayout->takeAt(0)) != nullptr)
	{
		if (QWidget* w = lit->widget())
		{
			w->deleteLater();
		}
		delete lit;
	}

	QListWidgetItem* ti = m_listType->currentItem();
	QListWidgetItem* bi = m_listBrand->currentItem();
	if (!ti || !bi)
	{
		return;
	}
	const QString type = ti->text();
	const QString brand = bi->text();
	if (!m_packagesByTypeBrand.contains(type) || !m_packagesByTypeBrand[type].contains(brand))
	{
		return;
	}

	const QStringList packages = m_packagesByTypeBrand[type][brand];
	int totalButtons = 0;

	auto placeButton = [&](QToolButton* btn) {
		btn->setFixedSize(118, 108);
		btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		m_modelsLayout->addWidget(btn, 0, Qt::AlignHCenter);
		++totalButtons;
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
			auto* btn = new QToolButton(m_modelsContainer);
			btn->setIconSize(QSize(72, 72));
			btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
			btn->setText(QFileInfo(packageRoot).fileName());
			const QString urdfPath = urdfs.front();
			btn->setToolTip(urdfPath);
			connect(btn, &QToolButton::clicked, this, [this, urdfPath]() {
				emit urdfImportRequested(urdfPath);
			});
			placeButton(btn);
			continue;
		}

		for (const QString& imgPath : images)
		{
			const QString urdfPath =
				pickUrdfForImageBase(QFileInfo(imgPath).completeBaseName(), urdfs);
			if (urdfPath.isEmpty())
			{
				continue;
			}
			auto* btn = new QToolButton(m_modelsContainer);
			btn->setIconSize(QSize(72, 72));
			btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
			QPixmap pm(imgPath);
			if (!pm.isNull())
			{
				btn->setIcon(QIcon(pm.scaled(72, 72, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
			}
			btn->setText(QFileInfo(imgPath).completeBaseName());
			btn->setToolTip(QStringLiteral("%1\n%2").arg(QFileInfo(packageRoot).fileName(), urdfPath));
			connect(btn, &QToolButton::clicked, this, [this, urdfPath]() {
				emit urdfImportRequested(urdfPath);
			});
			placeButton(btn);
		}
	}

	if (totalButtons == 0)
	{
		auto* empty = new QLabel(QStringLiteral("此品牌下暂无可用型号。"));
		m_modelsLayout->addWidget(empty);
	}
	else
	{
		m_modelsLayout->addStretch(1);
	}
}
