#include "MainWindow.h"

#include "../RobotWidget/inc/RobotSimulationController.h"

#include <QAction>
#include <QFileDialog>
#include <QList>
#include <QMenu>
#include <QMessageBox>
#include <QPoint>
#include <QHash>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendHierarchyModel.h"
#include "CoreTypes.h"
#include "DocumentPage.h"
#include "DocumentPointCloudOps.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "MainWindow_p.h"
#include "WidgetRenderAccess.h"
#include "MainWindowSelectionService.h"
#include "RunInfoPage.h"

using namespace mainwindow_detail;

namespace
{

const QHash<QString, QString>& osgClassNameZhMap()
{
	static const QHash<QString, QString> map = {
		{QStringLiteral("Group"), QStringLiteral("组")},
		{QStringLiteral("MatrixTransform"), QStringLiteral("矩阵变换")},
		{QStringLiteral("PositionAttitudeTransform"), QStringLiteral("位姿变换")},
		{QStringLiteral("Geode"), QStringLiteral("几何节点")},
		{QStringLiteral("Geometry"), QStringLiteral("几何体")},
		{QStringLiteral("Camera"), QStringLiteral("相机")},
		{QStringLiteral("AutoTransform"), QStringLiteral("自动变换")},
		{QStringLiteral("Switch"), QStringLiteral("开关节点")},
		{QStringLiteral("LOD"), QStringLiteral("细节层次")},
		{QStringLiteral("LightSource"), QStringLiteral("光源")},
	};
	return map;
}

const QHash<QString, QString>& osgNodeNameZhMap()
{
	static const QHash<QString, QString> map = {
		{QStringLiteral("SceneContent"), QStringLiteral("场景内容")},
		{QStringLiteral("BackendObjects"), QStringLiteral("后端对象")},
		{QStringLiteral("RobotAssembly"), QStringLiteral("机器人装配")},
		{QStringLiteral("TrajectoryOverlay"), QStringLiteral("轨迹叠加层")},
		{QStringLiteral("TcpTeachSceneOverlay"), QStringLiteral("TCP示教场景叠加")},
		{QStringLiteral("GizmoOverlay"), QStringLiteral("变换罗盘叠加")},
		{QStringLiteral("Annotations"), QStringLiteral("注释")},
		{QStringLiteral("InstructionPoseAxes"), QStringLiteral("指令位姿轴")},
		{QStringLiteral("LINE_TargetAxis"), QStringLiteral("直线目标轴")},
		{QStringLiteral("PTP_TargetAxis"), QStringLiteral("点到点目标轴")},
		{QStringLiteral("TcpTeachCompass"), QStringLiteral("TCP示教罗盘")},
		{QStringLiteral("TcpTeachMount"), QStringLiteral("TCP示教挂载")},
		{QStringLiteral("TcpTeachWorld"), QStringLiteral("TCP示教世界")},
		{QStringLiteral("TcpTeachGizmoOverlay"), QStringLiteral("TCP示教罗盘叠加")},
		{QStringLiteral("RobotHierarchy"), QStringLiteral("机器人层级")},
		{QStringLiteral("meshWireOverlay"), QStringLiteral("网格线框叠加")},
		{QStringLiteral("RosZUp_to_OsgYUp"), QStringLiteral("ROS Z上→OSG Y上")},
		{QStringLiteral("LinkFrameAxes"), QStringLiteral("连杆坐标轴")},
		{QStringLiteral("JointRotationAxis"), QStringLiteral("关节旋转轴")},
		{QStringLiteral("PointCloud"), QStringLiteral("点云")},
		{QStringLiteral("Model"), QStringLiteral("模型")},
	};
	return map;
}

QString translateOsgClassName(const QString& cls, bool useChinese)
{
	if (!useChinese)
	{
		return cls;
	}
	return osgClassNameZhMap().value(cls, cls);
}

QString translateOsgNodeName(const QString& name, bool useChinese)
{
	if (!useChinese || name.isEmpty())
	{
		return name;
	}
	const auto exact = osgNodeNameZhMap().constFind(name);
	if (exact != osgNodeNameZhMap().constEnd())
	{
		return exact.value();
	}
	if (name.startsWith(QStringLiteral("RobotToolFrame_")))
	{
		return QStringLiteral("机器人工具坐标系_") + name.mid(15);
	}
	if (name.startsWith(QStringLiteral("RobotUserFrame_")))
	{
		return QStringLiteral("机器人用户坐标系_") + name.mid(15);
	}
	if (name.startsWith(QStringLiteral("Joint")) && name.length() > 5)
	{
		bool ok = false;
		name.mid(5).toInt(&ok);
		if (ok)
		{
			return QStringLiteral("关节") + name.mid(5);
		}
	}
	if (name.startsWith(QStringLiteral("Link")) && name.length() > 4)
	{
		bool ok = false;
		name.mid(4).toInt(&ok);
		if (ok)
		{
			return QStringLiteral("连杆") + name.mid(4);
		}
	}
	if (name.startsWith(QStringLiteral("Axis_Visual_")))
	{
		return QStringLiteral("轴可视化_") + name.mid(12);
	}
	auto replaceSuffix = [&name](const QString& enSuffix, const QString& zhSuffix) -> QString {
		if (name.endsWith(enSuffix))
		{
			return name.left(name.size() - enSuffix.size()) + zhSuffix;
		}
		return QString();
	};
	if (QString r = replaceSuffix(QStringLiteral("_JointContent"), QStringLiteral("_关节内容")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_OriginMarker"), QStringLiteral("_原点标记")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_BackendVisual"), QStringLiteral("_后端可视化")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Geometry"), QStringLiteral("_几何")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Container"), QStringLiteral("_容器")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Visual"), QStringLiteral("_可视化")); !r.isEmpty())
	{
		return r;
	}
	return name;
}

QStringList collectSubtreeBackendIds(const DocumentPage& doc, const QString& rootBackendId)
{
	QStringList ids;
	if (rootBackendId.isEmpty())
	{
		return ids;
	}
	const std::string rootStd = rootBackendId.toStdString();
	const std::vector<std::string> subtree = doc.hierarchyModel().subtreeIds(rootStd);
	if (subtree.empty() && const_cast<DocumentPage&>(doc).data().isValid(rootBackendId))
	{
		ids.append(rootBackendId);
	}
	else
	{
		for (const std::string& id : subtree)
		{
			ids.append(QString::fromStdString(id));
		}
	}
	return ids;
}

} // namespace

void MainWindow::beginBackendTreeEventRefreshSuppress()
{
	++m_backendTreeEventRefreshSuppress;
}

void MainWindow::endBackendTreeEventRefreshSuppress()
{
	if (m_backendTreeEventRefreshSuppress > 0)
	{
		--m_backendTreeEventRefreshSuppress;
	}
}

MainWindow::ScopedBackendTreeRefreshSuppress::ScopedBackendTreeRefreshSuppress(MainWindow& mw) : m_mw(mw)
{
	m_mw.beginBackendTreeEventRefreshSuppress();
}

MainWindow::ScopedBackendTreeRefreshSuppress::~ScopedBackendTreeRefreshSuppress()
{
	m_mw.endBackendTreeEventRefreshSuppress();
}

void MainWindow::focusBackendInTreeAfterImport(const QString& backendId)
{
	focusBackendInTreeLocal(backendId);
}

void MainWindow::refreshBackendTree()
{
	if (!m_backendTree || !m_backendRootItem)
	{
		return;
	}

	const QString selectedBackendId = m_selectionState.selectedBackendId();

	m_backendTreeItemsById.clear();
	m_backendRootItem->takeChildren();
	m_annotationRootItem = new QTreeWidgetItem(QStringList()
		<< i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));
	m_backendRootItem->addChild(m_annotationRootItem);
	m_annotationRootItem->setExpanded(true);
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return;
	}
	const QVector<cloudsim::core::BackendObjectDto> objects = doc->data().listObjectSnapshots();
	QHash<QString, QTreeWidgetItem*> idToItem;
	idToItem.reserve(objects.size());
	for (const cloudsim::core::BackendObjectDto& dto : objects)
	{
		const QString id = dto.id;
		if (id.isEmpty())
		{
			continue;
		}

		const QString nodeText = QStringLiteral("%1 [%2]").arg(dto.name).arg(id);
		auto* child = new QTreeWidgetItem(QStringList() << nodeText);
		child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
		child->setData(0, kRoleItemType, kItemTypeBackend);
		child->setData(0, kRoleBackendId, id);
		child->setCheckState(0, Qt::Checked);
		idToItem.insert(id, child);
		m_backendTreeItemsById.insert(id, child);
	}
	for (const cloudsim::core::BackendObjectDto& dto : objects)
	{
		const QString id = dto.id;
		QTreeWidgetItem* const item = idToItem.value(id, nullptr);
		if (!item)
		{
			continue;
		}
		const QVector<QString> parentIds = dto.parentIds;
		const QString parentId = parentIds.isEmpty() ? QString() : parentIds.front();
		QTreeWidgetItem* parentItem = idToItem.value(parentId, m_backendRootItem);
		if (!parentItem)
		{
			parentItem = m_backendRootItem;
		}
		parentItem->addChild(item);

		for (int index = 1; index < parentIds.size(); ++index)
		{
			const QString extraParentId = parentIds[index];
			if (extraParentId.isEmpty() || extraParentId == parentId)
			{
				continue;
			}
			QTreeWidgetItem* extraParentItem = idToItem.value(extraParentId, m_backendRootItem);
			if (!extraParentItem)
			{
				extraParentItem = m_backendRootItem;
			}
			const QString refText = item->text(0) + QStringLiteral(" (ref)");
			auto* refItem = new QTreeWidgetItem(QStringList() << refText);
			refItem->setFlags(refItem->flags() | Qt::ItemIsUserCheckable);
			refItem->setData(0, kRoleItemType, kItemTypeBackend);
			refItem->setData(0, kRoleBackendId, id);
			refItem->setCheckState(0, item->checkState(0));
			extraParentItem->addChild(refItem);
		}
	}

	if (cloudsim::core::IRenderView* rv = renderViewFromPage(doc))
	{
		const QVector<cloudsim::core::AnnotationSnapshotDto> snaps = rv->annotationSnapshots();
		for (const cloudsim::core::AnnotationSnapshotDto& s : snaps)
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
		QTreeWidgetItem* const item = m_backendTreeItemsById.value(selectedBackendId, nullptr);
		if (item)
		{
			m_backendTree->setCurrentItem(item);
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
	DocumentPage* page = currentPage();
	if (!page)
	{
		m_osgSceneTree->addTopLevelItem(new QTreeWidgetItem(QStringList()
			<< i18n(QStringLiteral("No scene"), QStringLiteral("无场景")) << QString()));
		return;
	}
	const auto snapshot = page->render().sceneGraphSnapshot(256);
	if (snapshot.className.isEmpty())
	{
		m_osgSceneTree->addTopLevelItem(new QTreeWidgetItem(QStringList()
			<< i18n(QStringLiteral("No scene"), QStringLiteral("无场景")) << QString()));
		return;
	}
	auto* rootItem = new QTreeWidgetItem(QStringList() << snapshot.name << snapshot.localMatrixSummary);
	m_osgSceneTree->addTopLevelItem(rootItem);
	std::function<void(QTreeWidgetItem*, const cloudsim::core::IRenderView::SceneNodeInfo&)> buildTree;
	buildTree = [&](QTreeWidgetItem* parent, const cloudsim::core::IRenderView::SceneNodeInfo& node)
	{
		auto* item = new QTreeWidgetItem(QStringList() << node.name << node.localMatrixSummary);
		parent->addChild(item);
		for (const auto& child : node.children)
		{
			buildTree(item, child);
		}
	};
	for (const auto& child : snapshot.children)
	{
		buildTree(rootItem, child);
	}
	rootItem->setExpanded(true);
}

void MainWindow::onBackendTreeSelectionChanged()
{
	MainWindowSelectionService::handleBackendTreeSelectionChanged(*this);
}

void MainWindow::onOsgBackendObjectPicked(const QString& backendId)
{
	MainWindowSelectionService::handleOsgBackendObjectPicked(*this, backendId);
}

void MainWindow::onAnnotationCreated(const QString& annotationId, const QString& displayText)
{
	if (sender() != renderWidgetFromPage(currentPage()) || !m_annotationRootItem)
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
	if (sender() != renderWidgetFromPage(currentPage()) || !m_annotationRootItem)
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
	if (sender() != renderWidgetFromPage(currentPage()) || !m_annotationRootItem)
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
	cloudsim::core::IRenderView* rv = renderViewFromPage(currentPage());
	if (!m_backendTree || !rv)
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
			rv->clearAllAnnotations();
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
			const QString backendId = item->data(0, kRoleBackendId).toString();
			DocumentPage* doc = currentPage();
			QMenu menu(this);
			QAction* toggle = menu.addAction(visible
				? i18n(QStringLiteral("Hide Object"), QStringLiteral("隐藏对象"))
				: i18n(QStringLiteral("Show Object"), QStringLiteral("显示对象")));
			QAction* focusView = menu.addAction(i18n(QStringLiteral("Focus View"), QStringLiteral("聚焦显示")));
			QAction* exportObj = nullptr;
			if (doc && doc->data().isValid(backendId))
			{
				const cloudsim::core::BackendObjectDto dto = doc->data().objectSnapshot(backendId);
				if (dto.hasGeometry)
				{
					if (dto.className == QStringLiteral("PointCloudBackendData"))
					{
						exportObj = menu.addAction(
							i18n(QStringLiteral("Export Point Cloud…"), QStringLiteral("导出点云…")));
					}
					else if (dto.className == QStringLiteral("BrepModel"))
					{
						exportObj = menu.addAction(
							i18n(QStringLiteral("Export STEP…"), QStringLiteral("导出 STEP…")));
					}
				}
			}
			QAction* deleteObj = menu.addAction(i18n(QStringLiteral("Delete"), QStringLiteral("删除")));
			QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
			if (action == toggle)
			{
				item->setCheckState(0, visible ? Qt::Unchecked : Qt::Checked);
			}
			else if (action == focusView)
			{
				rv->focusCameraOnBackend(backendId);
			}
			else if (action == exportObj)
			{
				exportBackendObjectFromTree(backendId);
			}
			else if (action == deleteObj)
			{
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
					removeBackendObjectFromDocument(backendId);
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
		rv->setAnnotationVisible(annotationId, !visible);
	}
	else if (action == remove)
	{
		rv->removeAnnotation(annotationId);
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
	const QStringList removed = collectSubtreeBackendIds(*doc, backendId);
	if (removed.isEmpty())
	{
		return;
	}
	QString unregErr;
	if (!doc->data().unregisterSubtree(backendId, &unregErr))
	{
		if (m_runInfoPage && !unregErr.isEmpty())
		{
			m_runInfoPage->appendWarning(unregErr);
		}
		return;
	}
	for (const QString& rid : removed)
	{
		doc->clearRobotSimulationIfContains(rid);
	}
	if (m_robotSimulation && m_robotSimulation->programExecutor().isRunning()
		&& !doc->hasRobotSimulationContext())
	{
		stopRobotSimulation();
	}
	MainWindowSelectionService::clearSelection(*this, true);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Removed backend object(s): %1").arg(removed.join(QStringLiteral(", "))),
				QStringLiteral("已删除后端对象: %1").arg(removed.join(QStringLiteral(", ")))));
	}
	refreshSimulationJointListFromCurrentDoc();
}

void MainWindow::exportBackendObjectFromTree(const QString& backendId)
{
	DocumentPage* doc = currentPage();
	if (!doc || backendId.isEmpty() || !doc->data().isValid(backendId))
	{
		return;
	}
	const cloudsim::core::BackendObjectDto dto = doc->data().objectSnapshot(backendId);
	if (!dto.hasGeometry)
	{
		return;
	}
	const bool isPointCloud = dto.className == QStringLiteral("PointCloudBackendData");
	const bool isBrep = dto.className == QStringLiteral("BrepModel");
	if (!isPointCloud && !isBrep)
	{
		return;
	}

	const QString baseName = dto.name.isEmpty() ? backendId : dto.name;
	QString savePath;
	if (isPointCloud)
	{
		savePath = QFileDialog::getSaveFileName(
			this,
			i18n(QStringLiteral("Export Point Cloud"), QStringLiteral("导出点云")),
			baseName + QStringLiteral(".ply"),
			QStringLiteral("PLY Files (*.ply);;All Files (*.*)"));
	}
	else
	{
		savePath = QFileDialog::getSaveFileName(
			this,
			i18n(QStringLiteral("Export STEP"), QStringLiteral("导出 STEP")),
			baseName + QStringLiteral(".step"),
			QStringLiteral("STEP Files (*.step *.stp);;All Files (*.*)"));
	}
	if (savePath.isEmpty())
	{
		return;
	}

	const std::string idUtf8 = backendId.toUtf8().constData();
	const std::string pathUtf8 = savePath.toUtf8().constData();
	std::string err;
	const bool ok = isPointCloud
		? document_point_cloud_ops::exportPointCloudToPly(doc, idUtf8, pathUtf8, &err)
		: document_point_cloud_ops::exportBrepToStep(doc, idUtf8, pathUtf8, &err);
	if (!m_runInfoPage)
	{
		return;
	}
	if (ok)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Exported to %1").arg(savePath),
				QStringLiteral("已导出到 %1").arg(savePath)));
	}
	else
	{
		const QString errQs = err.empty() ? QStringLiteral("unknown error") : QString::fromStdString(err);
		m_runInfoPage->appendWarning(
			i18n(QStringLiteral("Export failed: %1").arg(errQs),
				QStringLiteral("导出失败: %1").arg(errQs)));
	}
}
