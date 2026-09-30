/// @file WidgetSceneSignalWiring.cpp
/// @brief 视口场景信号经 IViewportSceneSignals / IViewportPickObserve 接线

#include "WidgetSceneSignalWiring.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "DocumentPage.h"
#include "IViewportPickObserve.h"
#include "IViewportSceneSignals.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "RobotSimulationController.h"
#include "SimulationCommandWidget.h"

void wireMainWindowDocumentSceneSignals(MainWindow& mw, DocumentPage* page, MainWindowRobotHost* robotHost)
{
	IViewportPickObserve* pick = page ? page->pickObserve() : nullptr;
	IViewportSceneSignals* scene = page ? page->viewportSceneSignals() : nullptr;
	if (!pick || !scene)
	{
		return;
	}

	scene->observeSelectedObjectPoseChanged(
		&mw, [&mw](const float x, const float y, const float z) { mw.onSelectedObjectPoseChanged(x, y, z); });
	scene->observeSelectedObjectRotationChanged(
		&mw, [&mw](const float rx, const float ry, const float rz) { mw.onSelectedObjectRotationChanged(rx, ry, rz); });
	scene->observeSelectedObjectColorChanged(
		&mw, [&mw](const float r, const float g, const float b, const float a)
		{ mw.onSelectedObjectColorChanged(r, g, b, a); });
	scene->observeTransformGizmoCommitted(&mw, [&mw]() { mw.onTransformGizmoCommitted(); });
	scene->observeTcpDragTeachPoseChanged(
		&mw, [&mw](const double pxMm, const double pyMm, const double pzMm, const double exDeg, const double eyDeg,
				   const double ezDeg)
		{ mw.onTcpDragTeachPoseChanged(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg); });
	scene->observeTcpDragTeachEnded(&mw, [&mw]() { mw.onTcpDragTeachEnded(); });
	scene->observeActiveAxisChanged(&mw, [&mw](const QString& axis) { mw.onActiveAxisChanged(axis); });
	scene->observeSelectionCanceledByEsc(&mw, [&mw]() { mw.onSelectionCanceledByEsc(); });
	scene->observeAnnotationCreated(
		&mw, [&mw](const QString& id, const QString& text) { mw.onAnnotationCreated(id, text); });
	scene->observeAnnotationRemoved(&mw, [&mw](const QString& id) { mw.onAnnotationRemoved(id); });
	scene->observeAnnotationVisibilityChanged(
		&mw, [&mw](const QString& id, const bool visible) { mw.onAnnotationVisibilityChanged(id, visible); });
	scene->observePointPickFeedback(&mw, [&mw](const QString& text) { mw.onPointPickFeedback(text); });
	scene->observeMeshPickFeedback(&mw, [&mw](const QString& text) { mw.onMeshPickFeedback(text); });
	pick->observeMeshPickCommitted(
		&mw, [scene, robotHost](const PickResult pickResult, const int pickKindInt)
		{
			if (scene->dispatchMeshPickCommitToInteractionSession(pickResult, pickKindInt))
			{
				return;
			}
			if (robotHost)
			{
				robotHost->notifyMeshPickCommitted(pickResult, static_cast<PickKind>(pickKindInt));
			}
		});
	pick->observeLabelingClickCommitted(
		&mw, [robotHost](const PickResult pickResult)
		{
			if (robotHost)
			{
				robotHost->notifyMeshTriangleLabelingClick(pickResult);
			}
		});
	pick->observeLabelingBrushStroke(
		&mw, [robotHost](const QVector<int> triIndices)
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
	pick->observePolylinePickCommitted(
		&mw, [robotHost](const QVector<float> polylineScreenXy, const QVector<double> mvpMatrix, const int viewportWidth,
						 const int viewportHeight)
		{
			if (robotHost)
			{
				robotHost->notifyMeshTriangleLabelingPolyline(polylineScreenXy, mvpMatrix, viewportWidth,
															  viewportHeight);
			}
		});
	scene->observeBackendObjectPicked(&mw, [&mw](const QString& backendId) { mw.onOsgBackendObjectPicked(backendId); });
	if (RobotSimulationController* sim = mw.robotSimulation())
	{
		scene->observeInstructionWaypointPicked(
			sim, [sim](const QString& instructionId, const bool isArcVia)
			{ sim->onInstructionWaypointPicked(instructionId.toStdString(), isArcVia); });
		scene->observeInstructionWaypointPickCanceled(
			sim,
			[sim]()
			{
				if (SimulationCommandWidget* cmdPage = sim->host() ? sim->host()->simulationCommandPage() : nullptr)
				{
					cmdPage->setInstructionWaypointPickMode(false);
				}
			});
	}
}
