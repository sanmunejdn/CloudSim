#include "MainWindow.h"

#include <QAction>
#include <QList>
#include <QMenu>
#include <QMessageBox>
#include <QPoint>
#include <QHash>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MainWindow_p.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"

#include <osg/AutoTransform>
#include <osg/Camera>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/Vec3f>

using namespace mainwindow_detail;

namespace
{

QString osgNodeLine(const osg::Node* node)
{
	if (!node)
	{
		return QStringLiteral("(null)");
	}
	const QString cls = QString::fromLatin1(node->className());
	const std::string& nm = node->getName();
	if (nm.empty())
	{
		return cls;
	}
	return cls + QStringLiteral(" — ") + QString::fromStdString(nm);
}

QString formatMatrix4(const osg::Matrixd& m)
{
	QString s;
	for (int r = 0; r < 4; ++r)
	{
		QString row;
		for (int c = 0; c < 4; ++c)
		{
			if (c)
			{
				row += QLatin1Char(' ');
			}
			row += QString::number(m(r, c), 'g', 6);
		}
		if (r)
		{
			s += QLatin1Char('\n');
		}
		s += row;
	}
	return s;
}

/// Local transform stored on this node (not accumulated world matrix). Non-transform nodes: em dash.
QString localMatrixSummary(const osg::Node* node)
{
	if (!node)
	{
		return QStringLiteral("—");
	}
	if (const auto* cam = dynamic_cast<const osg::Camera*>(node))
	{
		return QStringLiteral("View:\n%1\nProj:\n%2")
			.arg(formatMatrix4(cam->getViewMatrix()))
			.arg(formatMatrix4(cam->getProjectionMatrix()));
	}
	if (const auto* mt = dynamic_cast<const osg::MatrixTransform*>(node))
	{
		return formatMatrix4(mt->getMatrix());
	}
	if (const auto* pat = dynamic_cast<const osg::PositionAttitudeTransform*>(node))
	{
		const osg::Matrixd m = osg::Matrixd::translate(pat->getPosition()) * osg::Matrixd::rotate(pat->getAttitude())
			* osg::Matrixd::scale(pat->getScale());
		return formatMatrix4(m);
	}
	if (const auto* at = dynamic_cast<const osg::AutoTransform*>(node))
	{
		const osg::Matrixd m = osg::Matrixd::translate(at->getPosition()) * osg::Matrixd::rotate(at->getRotation())
			* osg::Matrixd::scale(at->getScale());
		return formatMatrix4(m);
	}
	return QStringLiteral("—");
}

void appendOsgNodeRecursive(QTreeWidgetItem* parent, osg::Node* node, int depthLeft)
{
	if (!node || depthLeft <= 0)
	{
		return;
	}
	auto* item = new QTreeWidgetItem(QStringList() << osgNodeLine(node) << localMatrixSummary(node));
	parent->addChild(item);

	if (osg::Group* g = node->asGroup())
	{
		for (unsigned i = 0; i < g->getNumChildren(); ++i)
		{
			appendOsgNodeRecursive(item, g->getChild(i), depthLeft - 1);
		}
		return;
	}

	auto* geode = dynamic_cast<osg::Geode*>(node);
	if (!geode)
	{
		return;
	}
	for (unsigned i = 0; i < geode->getNumDrawables(); ++i)
	{
		osg::Drawable* d = geode->getDrawable(i);
		QString line = QStringLiteral("Drawable: ");
		if (d)
		{
			line += QString::fromLatin1(d->className());
			const std::string& dn = d->getName();
			if (!dn.empty())
			{
				line += QStringLiteral(" — ") + QString::fromStdString(dn);
			}
		}
		else
		{
			line += QStringLiteral("(null)");
		}
		item->addChild(new QTreeWidgetItem(QStringList()
			<< line << QStringLiteral("—")));
	}
}

} // namespace

void MainWindow::refreshBackendTree()
{
	if (!m_backendTree || !m_backendRootItem)
	{
		return;
	}

	QString selectedBackendId;
	const QList<QTreeWidgetItem*> prevSel = m_backendTree->selectedItems();
	if (!prevSel.isEmpty())
	{
		QTreeWidgetItem* cur = prevSel.first();
		if (cur && cur->data(0, kRoleItemType).toInt() == kItemTypeBackend)
		{
			selectedBackendId = cur->data(0, kRoleBackendId).toString();
		}
	}

	m_backendRootItem->takeChildren();
	m_annotationRootItem = new QTreeWidgetItem(QStringList()
		<< i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));
	m_backendRootItem->addChild(m_annotationRootItem);
	m_annotationRootItem->setExpanded(true);
	const auto dataList = activeBackend().listData();
	QHash<QString, QTreeWidgetItem*> idToItem;
	QHash<QString, QString> idToParent;
	DocumentPage* page = currentPage();
	for (const auto& data : dataList)
	{
		if (!data)
		{
			continue;
		}

		const QString nodeText = QStringLiteral("%1 [%2]")
			.arg(QString::fromStdString(data->name()))
			.arg(QString::fromStdString(data->id()));
		auto* child = new QTreeWidgetItem(QStringList() << nodeText);
		child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
		child->setData(0, kRoleItemType, kItemTypeBackend);
		child->setData(0, kRoleBackendId, QString::fromStdString(data->id()));
		child->setCheckState(0, Qt::Checked);
		const QString id = QString::fromStdString(data->id());
		idToItem.insert(id, child);
		idToParent.insert(id, page ? page->backendParentId().value(id) : QString());
	}
	for (auto it = idToItem.begin(); it != idToItem.end(); ++it)
	{
		const QString id = it.key();
		QTreeWidgetItem* item = it.value();
		const QString parentId = idToParent.value(id);
		QTreeWidgetItem* parentItem = idToItem.value(parentId, m_backendRootItem);
		if (!parentItem)
		{
			parentItem = m_backendRootItem;
		}
		parentItem->addChild(item);
	}

	if (OsgWidget* osg = currentOsgWidget())
	{
		const QList<OsgWidget::AnnotationSnapshot> snaps = osg->annotationSnapshots();
		for (const OsgWidget::AnnotationSnapshot& s : snaps)
		{
			const QString label = s.displayText.isEmpty() ? s.id : s.displayText;
			auto* item = new QTreeWidgetItem(QStringList() << label);
			item->setData(0, kRoleItemType, kItemTypeAnnotation);
			item->setData(0, kRoleAnnotationId, s.id);
			item->setCheckState(0, s.visible ? Qt::Checked : Qt::Unchecked);
			m_annotationRootItem->addChild(item);
		}
		if (!snaps.isEmpty())
		{
			m_annotationRootItem->setExpanded(true);
		}
	}

	m_backendRootItem->setExpanded(true);

	if (!selectedBackendId.isEmpty())
	{
		for (int i = 0; i < m_backendRootItem->childCount(); ++i)
		{
			QTreeWidgetItem* c = m_backendRootItem->child(i);
			if (!c || c == m_annotationRootItem)
			{
				continue;
			}
			if (c->data(0, kRoleItemType).toInt() == kItemTypeBackend
				&& c->data(0, kRoleBackendId).toString() == selectedBackendId)
			{
				m_backendTree->setCurrentItem(c);
				break;
			}
		}
	}

	refreshOsgSceneTree();
}

void MainWindow::refreshOsgSceneTree()
{
	if (!m_osgSceneTree)
	{
		return;
	}
	m_osgSceneTree->clear();
	OsgWidget* osg = currentOsgWidget();
	const osg::Group* root = osg ? osg->sceneGraphRoot() : nullptr;
	if (!root)
	{
		m_osgSceneTree->addTopLevelItem(new QTreeWidgetItem(QStringList()
			<< i18n(QStringLiteral("No scene"), QStringLiteral("无场景")) << QString()));
		return;
	}

	auto* rootItem = new QTreeWidgetItem(QStringList() << osgNodeLine(root) << localMatrixSummary(root));
	m_osgSceneTree->addTopLevelItem(rootItem);
	osg::Group* rootRw = const_cast<osg::Group*>(root);
	for (unsigned i = 0; i < rootRw->getNumChildren(); ++i)
	{
		appendOsgNodeRecursive(rootItem, rootRw->getChild(i), 256);
	}
	rootItem->setExpanded(true);
}

void MainWindow::onBackendTreeSelectionChanged()
{
	if (!m_backendTree)
	{
		return;
	}
	m_activeInstructionForProperty.reset();
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->clearInstructionSelection();
	}

	OsgWidget* osg = currentOsgWidget();

	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		if (osg)
		{
			osg->setSelectionActive(false);
		}
		updatePropertyPanel(nullptr);
		return;
	}

	const QTreeWidgetItem* current = selected.first();
	if (current->data(0, kRoleItemType).toInt() != kItemTypeBackend)
	{
		if (osg)
		{
			osg->setSelectionActive(false);
		}
		updatePropertyPanel(nullptr);
		return;
	}
	const QString id = current->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	const bool rowVisible = current->checkState(0) != Qt::Unchecked;
	DocumentPage* doc = currentPage();
	const bool urdfLinkMesh = doc && doc->hasRobotSimulationContext() && doc->robotLinkBackendIds().contains(id);

	if (osg)
	{
		const std::string idStd = id.toStdString();
		osg->setBackendObjectVisible(idStd, rowVisible);
		osg->setSelectionActive(data != nullptr);
		if (data)
		{
			auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(data);
			if (pointCloud && !pointCloud->pointPositionsXyz().empty())
			{
				// Full load clears point annotations; only reload when the branch is missing.
				if (osg->hasBackendObjectBranch(idStd))
				{
					osg->syncSelectionFromBackend(*pointCloud);
				}
				else
				{
					QString geomErr;
					osg->loadPointCloudFromBackendData(*pointCloud, &geomErr, false);
				}
			}
			else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
			{
				if (!mesh->triangleSoup().empty())
				{
					if (osg->hasBackendObjectBranch(idStd))
					{
						osg->syncSelectionFromBackend(*mesh);
					}
					else
					{
						QString geomErr;
						if (urdfLinkMesh)
						{
							osg->loadMeshFromBackendData(*mesh, &geomErr, false, true, true);
						}
						else
						{
							osg->loadMeshFromBackendData(*mesh, &geomErr, false);
						}
					}
				}
				else
				{
					// Parent/group node without direct geometry: keep this backend as active
					// and allow picking across all visible children.
					osg->syncSelectionForBackendId(idStd);
				}
			}
			else
			{
				// Generic backend with no direct render branch (e.g. assembly parent).
				osg->syncSelectionForBackendId(idStd);
			}
		}
	}
	updatePropertyPanel(data);
}

void MainWindow::onAnnotationCreated(const QString& annotationId, const QString& displayText)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	auto* item = new QTreeWidgetItem(QStringList() << displayText);
	item->setData(0, kRoleItemType, kItemTypeAnnotation);
	item->setData(0, kRoleAnnotationId, annotationId);
	item->setCheckState(0, Qt::Checked);
	m_annotationRootItem->addChild(item);
	m_annotationRootItem->setExpanded(true);
	refreshOsgSceneTree();
}

void MainWindow::onAnnotationRemoved(const QString& annotationId)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	for (int i = 0; i < m_annotationRootItem->childCount(); ++i)
	{
		QTreeWidgetItem* child = m_annotationRootItem->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			delete m_annotationRootItem->takeChild(i);
			refreshOsgSceneTree();
			return;
		}
	}
}

void MainWindow::onAnnotationVisibilityChanged(const QString& annotationId, bool visible)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	for (int i = 0; i < m_annotationRootItem->childCount(); ++i)
	{
		QTreeWidgetItem* child = m_annotationRootItem->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
			return;
		}
	}
}

void MainWindow::onBackendTreeContextMenu(const QPoint& pos)
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_backendTree || !osg)
	{
		return;
	}
	QTreeWidgetItem* item = m_backendTree->itemAt(pos);
	if (!item)
	{
		return;
	}

	if (item == m_annotationRootItem)
	{
		QMenu menu(this);
		QAction* clearAll = menu.addAction(i18n(QStringLiteral("Clear All Annotations"), QStringLiteral("清空全部注释")));
		QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
		if (action == clearAll)
		{
			osg->clearAllAnnotations();
			refreshOsgSceneTree();
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(QStringLiteral("All annotations cleared."));
			}
		}
		return;
	}

	if (item->data(0, kRoleItemType).toInt() != kItemTypeAnnotation)
	{
		if (item->data(0, kRoleItemType).toInt() == kItemTypeBackend)
		{
			const bool visible = item->checkState(0) == Qt::Checked;
			QMenu menu(this);
			QAction* toggle = menu.addAction(visible
				? i18n(QStringLiteral("Hide Object"), QStringLiteral("隐藏对象"))
				: i18n(QStringLiteral("Show Object"), QStringLiteral("显示对象")));
			QAction* focusView = menu.addAction(i18n(QStringLiteral("Focus View"), QStringLiteral("聚焦显示")));
			QAction* deleteObj = menu.addAction(i18n(QStringLiteral("Delete"), QStringLiteral("删除")));
			QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
			if (action == toggle)
			{
				item->setCheckState(0, visible ? Qt::Unchecked : Qt::Checked);
			}
			else if (action == focusView)
			{
				const QString id = item->data(0, kRoleBackendId).toString();
				osg->focusCameraOnBackend(id.toStdString());
			}
			else if (action == deleteObj)
			{
				const QString id = item->data(0, kRoleBackendId).toString();
				const QMessageBox::StandardButton r = QMessageBox::question(
					this,
					i18n(QStringLiteral("Delete object"), QStringLiteral("删除对象")),
					i18n(QStringLiteral("Delete this object and all child parts? This cannot be undone."),
						QStringLiteral(
							"删除该对象及其子部件？此操作无法撤销。")),
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::No);
				if (r == QMessageBox::Yes)
				{
					removeBackendObjectFromDocument(id);
				}
			}
		}
		return;
	}
	const QString annotationId = item->data(0, kRoleAnnotationId).toString();
	const bool visible = item->checkState(0) == Qt::Checked;

	QMenu menu(this);
	QAction* toggle = menu.addAction(visible
		? i18n(QStringLiteral("Hide Annotation"), QStringLiteral("隐藏注释"))
		: i18n(QStringLiteral("Show Annotation"), QStringLiteral("显示注释")));
	QAction* remove = menu.addAction(i18n(QStringLiteral("Delete Annotation"), QStringLiteral("删除注释")));
	QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
	if (!action)
	{
		return;
	}
	if (action == toggle)
	{
		osg->setAnnotationVisible(annotationId, !visible);
	}
	else if (action == remove)
	{
		osg->removeAnnotation(annotationId);
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(QStringLiteral("Annotation removed: %1").arg(annotationId));
		}
	}
}

void MainWindow::removeBackendObjectFromDocument(const QString& backendId)
{
	DocumentPage* doc = currentPage();
	if (!doc || backendId.isEmpty())
	{
		return;
	}
	const QStringList removed = doc->removeBackendSubtree(backendId);
	if (removed.isEmpty())
	{
		return;
	}
	for (const QString& rid : removed)
	{
		doc->clearRobotSimulationIfContains(rid);
	}
	if (m_robotInstructionPlayback.isRunning() && !doc->hasRobotSimulationContext())
	{
		stopRobotSimulation();
	}
	if (OsgWidget* osg = doc->osgWidget())
	{
		for (const QString& id : removed)
		{
			osg->removeBackendObjectVisual(id.toStdString());
		}
	}
	updatePropertyPanel(nullptr);
	refreshBackendTree();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Removed backend object(s): %1").arg(removed.join(QStringLiteral(", "))),
				QStringLiteral("已删除后端对象: %1").arg(removed.join(QStringLiteral(", ")))));
	}
	refreshSimulationJointListFromCurrentDoc();
}
