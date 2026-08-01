/// @file HelpBrowserDialog.cpp
/// @brief 内嵌 HTML 帮助：左侧分级目录，右侧正文

#include "HelpBrowserDialog.h"

#include <QDialogButtonBox>
#include <QFileInfo>
#include <QHeaderView>
#include <QSplitter>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QUrl>

namespace
{
constexpr int kHelpFileRole = Qt::UserRole;

QTreeWidgetItem* addGroup(QTreeWidget* tree, QTreeWidgetItem* parent, const QString& title)
{
	auto* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(tree);
	item->setText(0, title);
	item->setFlags(Qt::ItemIsEnabled);
	item->setExpanded(true);
	return item;
}

QTreeWidgetItem* addPage(QTreeWidgetItem* parent, const QString& title, const QString& fileName)
{
	auto* item = new QTreeWidgetItem(parent);
	item->setText(0, title);
	item->setData(0, kHelpFileRole, fileName);
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	return item;
}

QTreeWidgetItem* addRootPage(QTreeWidget* tree, const QString& title, const QString& fileName)
{
	auto* item = new QTreeWidgetItem(tree);
	item->setText(0, title);
	item->setData(0, kHelpFileRole, fileName);
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	return item;
}
} // namespace

HelpBrowserDialog::HelpBrowserDialog(QWidget* parent, const QString& title, const QString& htmlFilePath,
									 bool useChinese)
	: QDialog(parent)
	, m_useChinese(useChinese)
{
	setWindowTitle(title);
	// 标准最小/最大化，去掉标题栏「?」
	setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::WindowMinMaxButtonsHint |
				   Qt::WindowCloseButtonHint);
	setMinimumSize(860, 560);
	resize(1024, 680);
	setAttribute(Qt::WA_DeleteOnClose);

	const QFileInfo info(htmlFilePath);
	m_helpLangDir = info.absolutePath();

	auto* root = new QVBoxLayout(this);
	auto* splitter = new QSplitter(Qt::Horizontal, this);

	m_tocTree = new QTreeWidget(splitter);
	m_tocTree->setHeaderHidden(true);
	m_tocTree->setMinimumWidth(200);
	m_tocTree->setMaximumWidth(300);
	m_tocTree->setIndentation(16);
	m_tocTree->setAnimated(true);
	m_tocTree->setExpandsOnDoubleClick(true);
	m_tocTree->setRootIsDecorated(true);
	if (QHeaderView* header = m_tocTree->header())
	{
		header->setStretchLastSection(true);
	}
	buildTocTree();

	m_browser = new QTextBrowser(splitter);
	m_browser->setOpenExternalLinks(true);
	m_browser->setSearchPaths({m_helpLangDir, QFileInfo(m_helpLangDir).absolutePath(),
							   QFileInfo(m_helpLangDir).absolutePath() + QStringLiteral("/images")});

	// 左目录、右正文
	splitter->addWidget(m_tocTree);
	splitter->addWidget(m_browser);
	splitter->setStretchFactor(0, 0);
	splitter->setStretchFactor(1, 1);
	splitter->setSizes({240, 760});
	root->addWidget(splitter);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
	connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
	root->addWidget(buttons);

	connect(m_tocTree, &QTreeWidget::itemActivated, this, &HelpBrowserDialog::onTocItemActivated);
	connect(m_tocTree, &QTreeWidget::currentItemChanged, this, &HelpBrowserDialog::onTocCurrentItemChanged);
	connect(m_browser, &QTextBrowser::sourceChanged, this,
			[this](const QUrl& url)
			{
				const QString name = QFileInfo(url.toLocalFile()).fileName();
				if (!name.isEmpty())
				{
					selectTocByFileName(name);
				}
			});

	const QString startName = info.fileName().isEmpty() ? QStringLiteral("index.html") : info.fileName();
	selectTocByFileName(startName);
	loadPage(startName);
}

void HelpBrowserDialog::buildTocTree()
{
	m_tocTree->clear();
	const bool zh = m_useChinese;

	addRootPage(m_tocTree, zh ? QStringLiteral("首页") : QStringLiteral("Home"), QStringLiteral("index.html"));

	QTreeWidgetItem* gettingStarted =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("入门") : QStringLiteral("Getting Started"));
	addPage(gettingStarted, zh ? QStringLiteral("入门与界面") : QStringLiteral("Main Window"),
			QStringLiteral("getting-started.html"));

	QTreeWidgetItem* basics =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("基础操作") : QStringLiteral("Basics"));
	addPage(basics, zh ? QStringLiteral("工程与文件") : QStringLiteral("Projects & Files"),
			QStringLiteral("projects.html"));
	addPage(basics, zh ? QStringLiteral("三维视图") : QStringLiteral("3D View"), QStringLiteral("view-3d.html"));
	addPage(basics, zh ? QStringLiteral("导入资源") : QStringLiteral("Import"), QStringLiteral("import.html"));
	addPage(basics, zh ? QStringLiteral("场景树与属性") : QStringLiteral("Scene & Properties"),
			QStringLiteral("scene.html"));

	QTreeWidgetItem* modeling =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("建模与出图") : QStringLiteral("Modeling & Drawing"));
	addPage(modeling, zh ? QStringLiteral("几何建模") : QStringLiteral("Geometric Modeling"),
			QStringLiteral("geometry.html"));
	addPage(modeling, zh ? QStringLiteral("工程图纸") : QStringLiteral("Engineering Drawing"),
			QStringLiteral("drawing.html"));

	QTreeWidgetItem* robot =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("机器人与仿真") : QStringLiteral("Robot & Simulation"));
	addPage(robot, zh ? QStringLiteral("机器人仿真") : QStringLiteral("Robot Simulation"),
			QStringLiteral("robot.html"));
	addPage(robot, zh ? QStringLiteral("轨迹生成与编辑") : QStringLiteral("Trajectories"),
			QStringLiteral("trajectory.html"));
	addPage(robot, zh ? QStringLiteral("工艺流程") : QStringLiteral("Process Flow"),
			QStringLiteral("process-flow.html"));

	QTreeWidgetItem* plugins =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("插件工具") : QStringLiteral("Plugins"));
	addPage(plugins, zh ? QStringLiteral("点云与网格") : QStringLiteral("Point Cloud & Mesh"),
			QStringLiteral("pointcloud.html"));
	addPage(plugins, zh ? QStringLiteral("几何分析插件") : QStringLiteral("Geometry Plugin"),
			QStringLiteral("geometry-plugin.html"));
	addPage(plugins, zh ? QStringLiteral("标注与深度学习") : QStringLiteral("Labeling & DL"),
			QStringLiteral("labeling.html"));
	addPage(plugins, zh ? QStringLiteral("PLC 与相机") : QStringLiteral("PLC & Camera"),
			QStringLiteral("plc-camera.html"));

	QTreeWidgetItem* assist =
		addGroup(m_tocTree, nullptr, zh ? QStringLiteral("助手与设置") : QStringLiteral("Assistant & Settings"));
	addPage(assist, zh ? QStringLiteral("AI 助手") : QStringLiteral("AI Assistant"), QStringLiteral("ai.html"));
	addPage(assist, zh ? QStringLiteral("设置") : QStringLiteral("Settings"), QStringLiteral("settings.html"));
	addPage(assist, zh ? QStringLiteral("附录") : QStringLiteral("Appendix"), QStringLiteral("appendix.html"));
}

void HelpBrowserDialog::onTocItemActivated(QTreeWidgetItem* item, int /*column*/)
{
	if (!item)
	{
		return;
	}
	const QString fileName = item->data(0, kHelpFileRole).toString();
	if (fileName.isEmpty())
	{
		item->setExpanded(!item->isExpanded());
		return;
	}
	loadPage(fileName);
}

void HelpBrowserDialog::onTocCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* /*previous*/)
{
	if (m_syncingToc || !current)
	{
		return;
	}
	const QString fileName = current->data(0, kHelpFileRole).toString();
	if (!fileName.isEmpty())
	{
		loadPage(fileName);
	}
}

void HelpBrowserDialog::selectTocByFileName(const QString& fileName)
{
	QList<QTreeWidgetItem*> stack;
	for (int i = 0; i < m_tocTree->topLevelItemCount(); ++i)
	{
		stack.append(m_tocTree->topLevelItem(i));
	}
	while (!stack.isEmpty())
	{
		QTreeWidgetItem* item = stack.takeLast();
		if (item->data(0, kHelpFileRole).toString() == fileName)
		{
			m_syncingToc = true;
			m_tocTree->setCurrentItem(item);
			m_tocTree->scrollToItem(item);
			m_syncingToc = false;
			return;
		}
		for (int c = 0; c < item->childCount(); ++c)
		{
			stack.append(item->child(c));
		}
	}
}

void HelpBrowserDialog::loadPage(const QString& fileName)
{
	const QString path = m_helpLangDir + QLatin1Char('/') + fileName;
	if (!QFileInfo::exists(path))
	{
		return;
	}
	m_browser->setSource(QUrl::fromLocalFile(path));
}
