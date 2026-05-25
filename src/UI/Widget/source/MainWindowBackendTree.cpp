#include "MainWindow.h"

#include "../RobotWidget/inc/RobotSimulationController.h"

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
#include "BackendHierarchyModel.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
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

QString osgNodeLine(const osg::Node* node, bool useChinese)
{
	if (!node)
	{
		return useChinese ? QStringLiteral("（空）") : QStringLiteral("(null)");
	}
	const QString cls = translateOsgClassName(QString::fromLatin1(node->className()), useChinese);
	const std::string& nm = node->getName();
	if (nm.empty())
	{
		return cls;
	}
	return cls + QStringLiteral(" — ") + translateOsgNodeName(QString::fromStdString(nm), useChinese);
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

/// 本节点局部变换（非累积世界矩阵）；非变换节点显示 —
QString localMatrixSummary(const osg::Node* node, bool useChinese)
{
	if (!node)
	{
		return QStringLiteral("—");
	}
	if (const auto* cam = dynamic_cast<const osg::Camera*>(node))
	{
		if (useChinese)
		{
			return QStringLiteral("视图:\n%1\n投影:\n%2")
				.arg(formatMatrix4(cam->getViewMatrix()))
				.arg(formatMatrix4(cam->getProjectionMatrix()));
		}
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

void appendOsgNodeRecursive(QTreeWidgetItem* parent, osg::Node* node, int depthLeft, bool useChinese)
{
	if (!node || depthLeft <= 0)
	{
		return;
	}
	auto* item = new QTreeWidgetItem(QStringList() << osgNodeLine(node, useChinese) << localMatrixSummary(node, useChinese));
	parent->addChild(item);

	if (osg::Group* g = node->asGroup())
	{
		for (unsigned i = 0; i < g->getNumChildren(); ++i)
		{
			appendOsgNodeRecursive(item, g->getChild(i), depthLeft - 1, useChinese);
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
		QString line = useChinese ? QStringLiteral("可绘制体：") : QStringLiteral("Drawable: ");
		if (d)
		{
			line += translateOsgClassName(QString::fromLatin1(d->className()), useChinese);
			const std::string& dn = d->getName();
			if (!dn.empty())
			{
				line += QStringLiteral(" — ") + translateOsgNodeName(QString::fromStdString(dn), useChinese);
			}
		}
		else
		{
			line += useChinese ? QStringLiteral("（空）") : QStringLiteral("(null)");
		}
		item->addChild(new QTreeWidgetItem(QStringList()
			<< line << QStringLiteral("—")));
	}
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
	if (subtree.empty() && doc.backend().contains(rootStd))
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

void MainWindow::focusBackendInTreeAfterImport(const std::shared_ptr<BackendDataBase>& backendObject)
{
	focusBackendInTree(backendObject);
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
	std::vector<std::shared_ptr<BackendDataBase>> objects;
	const std::vector<std::string> topoIds = activeBackend().topoOrder();
	objects.reserve(topoIds.size());
	for (const std::string& id : topoIds)
	{
		std::shared_ptr<BackendDataBase> data = activeBackend().getData(id);
		if (data)
		{
			objects.push_back(std::move(data));
		}
	}
	QHash<QString, QTreeWidgetItem*> idToItem;
	idToItem.reserve(static_cast<int>(objects.size()));
	for (const std::shared_ptr<BackendDataBase>& data : objects)
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		if (id.isEmpty())
		{
			continue;
		}

		const QString nodeText = QStringLiteral("%1 [%2]")
			.arg(QString::fromStdString(data->name()))
			.arg(QString::fromStdString(data->id()));
		auto* child = new QTreeWidgetItem(QStringList() << nodeText);
		child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
		child->setData(0, kRoleItemType, kItemTypeBackend);
		child->setData(0, kRoleBackendId, id);
		child->setCheckState(0, Qt::Checked);
		idToItem.insert(id, child);
		m_backendTreeItemsById.insert(id, child);
	}
	for (const std::shared_ptr<BackendDataBase>& data : objects)
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		QTreeWidgetItem* const item = idToItem.value(id, nullptr);
		if (!item)
		{
			continue;
		}
		const std::vector<std::string> parentIdsStd = activeBackend().parentsOf(data->id());
		const QString parentId = parentIdsStd.empty() ? QString() : QString::fromStdString(parentIdsStd.front());
		QTreeWidgetItem* parentItem = idToItem.value(parentId, m_backendRootItem);
		if (!parentItem)
		{
			parentItem = m_backendRootItem;
		}
		parentItem->addChild(item);

		for (std::size_t index = 1; index < parentIdsStd.size(); ++index)
		{
			const QString extraParentId = QString::fromStdString(parentIdsStd[index]);
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
	const bool useChinese = m_useChinese;
	m_osgSceneTree->clear();
	OsgWidget* osg = currentOsgWidget();
	const osg::Group* root = osg ? osg->sceneGraphRoot() : nullptr;
	if (!root)
	{
		m_osgSceneTree->addTopLevelItem(new QTreeWidgetItem(QStringList()
			<< i18n(QStringLiteral("No scene"), QStringLiteral("无场景")) << QString()));
		return;
	}

	auto* rootItem = new QTreeWidgetItem(QStringList() << osgNodeLine(root, useChinese) << localMatrixSummary(root, useChinese));
	m_osgSceneTree->addTopLevelItem(rootItem);
	osg::Group* rootRw = const_cast<osg::Group*>(root);
	for (unsigned i = 0; i < rootRw->getNumChildren(); ++i)
	{
		appendOsgNodeRecursive(rootItem, rootRw->getChild(i), 256, useChinese);
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
