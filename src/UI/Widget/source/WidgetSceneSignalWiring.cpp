#include "WidgetSceneSignalWiring.h"

#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "OsgWidget.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"

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
	QObject::connect(o, &OsgWidget::meshPickCommitted, &mw, [robotHost](const PickResult pick, const int pickKindInt) {
		if (robotHost)
		{
			robotHost->notifyMeshPickCommitted(pick, static_cast<PickKind>(pickKindInt));
		}
	});
	QObject::connect(o, &OsgWidget::labelingClickCommitted, &mw, [robotHost](const PickResult pick) {
		if (robotHost)
		{
			robotHost->notifyMeshTriangleLabelingClick(pick);
		}
	});
	QObject::connect(o, &OsgWidget::labelingBrushStroke, &mw, [robotHost](const QVector<int> triIndices) {
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
	QObject::connect(
		o,
		&OsgWidget::polylinePickCommitted,
		&mw,
		[robotHost](
			const QVector<float> polylineScreenXy,
			const QVector<double> mvpMatrix,
			const int viewportWidth,
			const int viewportHeight) {
			if (robotHost)
			{
				robotHost->notifyMeshTriangleLabelingPolyline(
					polylineScreenXy, mvpMatrix, viewportWidth, viewportHeight);
			}
		});
	QObject::connect(o, &OsgWidget::backendObjectPicked, &mw, &MainWindow::onOsgBackendObjectPicked);
}
