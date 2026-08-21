/// @file WidgetSceneSignalWiring.cpp
/// @brief OsgWidget 信号接线

#include "WidgetSceneSignalWiring.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "OsgWidget.h"
#include "RobotSimulationController.h"
#include "SimulationCommandWidget.h"
#include "ViewportInteraction/ViewportHit.h"
#include "ViewportInteraction/ViewportInteractionController.h"

void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost)
{
	OsgWidget* o = page ? page->osgWidget() : nullptr;
	if (!o)
	{
		return;
	}
	QObject::connect(o, &OsgWidget::selectedObjectPoseChanged, &mw, &MainWindow::onSelectedObjectPoseChanged);
	QObject::connect(o, &OsgWidget::selectedObjectRotationChanged, &mw, &MainWindow::onSelectedObjectRotationChanged);
	QObject::connect(o, &OsgWidget::selectedObjectColorChanged, &mw, &MainWindow::onSelectedObjectColorChanged);
	QObject::connect(o, &OsgWidget::transformGizmoCommitted, &mw, &MainWindow::onTransformGizmoCommitted);
	QObject::connect(o, &OsgWidget::tcpDragTeachPoseChanged, &mw, &MainWindow::onTcpDragTeachPoseChanged);
	QObject::connect(o, &OsgWidget::tcpDragTeachEnded, &mw, &MainWindow::onTcpDragTeachEnded);
	QObject::connect(o, &OsgWidget::activeAxisChanged, &mw, &MainWindow::onActiveAxisChanged);
	QObject::connect(o, &OsgWidget::selectionCanceledByEsc, &mw, &MainWindow::onSelectionCanceledByEsc);
	QObject::connect(o, &OsgWidget::annotationCreated, &mw, &MainWindow::onAnnotationCreated);
	QObject::connect(o, &OsgWidget::annotationRemoved, &mw, &MainWindow::onAnnotationRemoved);
	QObject::connect(o, &OsgWidget::annotationVisibilityChanged, &mw, &MainWindow::onAnnotationVisibilityChanged);
	QObject::connect(o, &OsgWidget::pointPickFeedback, &mw, &MainWindow::onPointPickFeedback);
	QObject::connect(o, &OsgWidget::meshPickFeedback, &mw, &MainWindow::onMeshPickFeedback);
	QObject::connect(o, &OsgWidget::meshPickCommitted, &mw,
					 [robotHost, o](const PickResult pick, const int pickKindInt)
					 {
						 if (o && o->hasInteractionSession() && o->interactionController())
						 {
							 ViewportHit hit;
							 hit.phase = HitPhase::Commit;
							 hit.kind = static_cast<PickKind>(pickKindInt);
							 hit.raw = pick;
							 o->interactionController()->dispatchCommit(hit, HitResolveContext{});
							 return;
						 }
						 if (robotHost)
						 {
							 robotHost->notifyMeshPickCommitted(pick, static_cast<PickKind>(pickKindInt));
						 }
					 });
	QObject::connect(o, &OsgWidget::labelingClickCommitted, &mw,
					 [robotHost](const PickResult pick)
					 {
						 if (robotHost)
						 {
							 robotHost->notifyMeshTriangleLabelingClick(pick);
						 }
					 });
	QObject::connect(o, &OsgWidget::labelingBrushStroke, &mw,
					 [robotHost](const QVector<int> triIndices)
					 {
						 if (robotHost)
						 {
							 std::vector<int> indices;
							 indices.reserve(static_cast<std::size_t>(triIndices.size()));
							 for (int ti : triIndices)
							 {
								 indices.push_back(ti);
							 }
							 robotHost->notifyMeshTriangleLabelingBrush(indices);
						 }
					 });
	QObject::connect(o, &OsgWidget::polylinePickCommitted, &mw,
					 [robotHost](const QVector<float> polylineScreenXy, const QVector<double> mvpMatrix,
								 const int viewportWidth, const int viewportHeight)
					 {
						 if (robotHost)
						 {
							 robotHost->notifyMeshTriangleLabelingPolyline(polylineScreenXy, mvpMatrix, viewportWidth,
																		   viewportHeight);
						 }
					 });
	QObject::connect(o, &OsgWidget::backendObjectPicked, &mw, &MainWindow::onOsgBackendObjectPicked);
	// 路点拾取：文档页创建时再接 Qt 信号，避免仅依赖启动期 std::function（当时常无 OsgWidget）
	if (RobotSimulationController* sim = mw.robotSimulation())
	{
		QObject::connect(o, &OsgWidget::instructionWaypointPicked, sim,
						 [sim](const QString& instructionId, bool isArcVia)
						 { sim->onInstructionWaypointPicked(instructionId.toStdString(), isArcVia); });
		QObject::connect(o, &OsgWidget::instructionWaypointPickCanceled, sim,
						 [sim]()
						 {
							 if (SimulationCommandWidget* page =
									 sim->host() ? sim->host()->simulationCommandPage() : nullptr)
							 {
								 page->setInstructionWaypointPickMode(false);
							 }
						 });
	}
}
