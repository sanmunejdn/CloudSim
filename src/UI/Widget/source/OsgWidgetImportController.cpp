#include "OsgWidgetImportController.h"

#include "OsgWidget.h"
#include "MeshBackendData.h"
#include "PlyIo.h"
#include "PointCloudBackendData.h"

#include <QFileInfo>
#include <QFile>

#include <osg/Camera>
#include <osg/Viewport>
#include <osgDB/Options>
#include <osgDB/ReadFile>

bool OsgWidgetImportController::importModelFile(OsgWidget& self, const QString& filePath, QString* errorMessage)
{
	if (filePath.isEmpty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("Empty file path.");
		return false;
	}

	const QString extension = QFileInfo(filePath).suffix().toLower();
	const QStringList supportedExtensions = {
		QStringLiteral("obj"), QStringLiteral("stl"), QStringLiteral("ply"), QStringLiteral("dae"),
		QStringLiteral("3ds"), QStringLiteral("fbx"), QStringLiteral("step"), QStringLiteral("stp"),
		QStringLiteral("igs"), QStringLiteral("iges")
	};

	if (!supportedExtensions.contains(extension))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Unsupported file extension: %1").arg(extension);
		return false;
	}

	// OSG-only formats (dae/fbx/…): staging preview; MainWindow copies into MeshBackendData then loadMeshFromBackendData.
	// obj/stl/ply/off are loaded in Data via CGAL first.
	osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
	options->setOptionString("noRotation noTessellation");
	osg::ref_ptr<osg::Node> modelNode = osgDB::readNodeFile(filePath.toStdString(), options.get());
	if (!modelNode.valid())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Failed to load model. Ensure OSG plugin for '.%1' is available.")
				.arg(extension);
		}
		return false;
	}

	self.clearStagingGeometry();
	self.m_stagingGroup->addChild(modelNode.get());
	self.m_modelCenter = modelNode->getBound().center();
	self.detachGizmoOverlay();
	self.syncCompassGizmoOrientation();
	self.cachePickablePointsFromNode(modelNode.get());
	self.clearPointAnnotations();
	self.clearPointPickMarker();
	self.applyColorToStagingGeometry(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
	self.setSelectionActive(true);
	emit self.selectedObjectPoseChanged(0.0f, 0.0f, 0.0f);

	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
		if (self.m_backendObjectRoots.empty())
		{
			self.m_viewer->home();
			if (osg::Camera* cam = self.m_viewer->getCamera())
			{
				double aspect = 1.0;
				const osg::Viewport* vp = cam->getViewport();
				if (vp && vp->width() > 0 && vp->height() > 0)
				{
					aspect = static_cast<double>(vp->width()) / static_cast<double>(vp->height());
				}
				cam->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
			}
		}
	}
	return true;
}

bool OsgWidgetImportController::importPointCloudFile(OsgWidget& self, const QString& filePath, QString* errorMessage)
{
	if (filePath.isEmpty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("Empty file path.");
		return false;
	}

	const QString extension = QFileInfo(filePath).suffix().toLower();
	const QStringList supportedExtensions = {
		QStringLiteral("ply"), QStringLiteral("laz"), QStringLiteral("las"), QStringLiteral("xyz")
	};

	if (!supportedExtensions.contains(extension))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Unsupported point cloud extension: %1").arg(extension);
		return false;
	}

	osg::ref_ptr<osg::Node> cloudNode;
	if (extension == QStringLiteral("xyz"))
	{
		cloudNode = self.loadXyzPointCloud(filePath, errorMessage);
	}
	else if (extension == QStringLiteral("ply"))
	{
		const QByteArray nativePathBytes = QFile::encodeName(filePath);
		const std::string nativePath(nativePathBytes.constData(),
			static_cast<size_t>(nativePathBytes.size()));
		if (plyFileHasTriangleFaces(nativePath))
		{
			MeshBackendData meshBackend;
			std::string meshErr;
			if (meshBackend.loadFromFile(nativePath, &meshErr) && meshBackend.hasGeometry())
			{
				return self.loadMeshFromBackendData(meshBackend, errorMessage, true, true, true);
			}
			if (errorMessage)
			{
				*errorMessage = meshErr.empty()
					? QStringLiteral("PLY has faces but mesh load failed.")
					: QString::fromStdString(meshErr);
			}
			return false;
		}

		// 纯顶点 PLY：CGAL 读点云缓冲，避免 OSG 网格 PLY 插件告警
		PointCloudBackendData plyBackend;
		std::string plyErr;
		if (plyBackend.readPointCloudFromPlyFile(nativePath, &plyErr) && !plyBackend.pointPositionsXyz().empty())
		{
			QString loadErr;
			osg::ref_ptr<osg::Geode> g = self.buildPointCloudGeode(plyBackend, &loadErr);
			if (!g)
			{
				if (errorMessage)
				{
					*errorMessage = loadErr.isEmpty()
						? QStringLiteral("PLY read OK but failed to build viewer geometry.")
						: loadErr;
				}
				return false;
			}
			self.clearStagingGeometry();
			self.m_stagingGroup->addChild(g.get());
			self.m_modelCenter = self.computePointCloudCenterFromXyz(plyBackend.pointPositionsXyz());
			self.detachGizmoOverlay();
			self.syncCompassGizmoOrientation();
			self.cachePickablePointsFromNode(g.get());
			self.clearPointAnnotations();
			self.clearPointPickMarker();
			const BackendColor c = plyBackend.color();
			self.applyColorToStagingGeometry(osg::Vec4(c.r, c.g, c.b, c.a));
			self.setSelectionActive(true);
			emit self.selectedObjectPoseChanged(0.0f, 0.0f, 0.0f);
			if (self.m_viewer.valid())
			{
				self.m_viewer->setSceneData(self.m_root.get());
				if (self.m_backendObjectRoots.empty())
				{
					self.m_viewer->home();
					if (osg::Camera* cam = self.m_viewer->getCamera())
					{
						double aspect = 1.0;
						const osg::Viewport* vp = cam->getViewport();
						if (vp && vp->width() > 0 && vp->height() > 0)
						{
							aspect = static_cast<double>(vp->width()) / static_cast<double>(vp->height());
						}
						cam->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
					}
				}
			}
			return true;
		}

		// Legacy fallbacks if CGAL/ASCII fallback could not parse this PLY.
		cloudNode = self.loadAsciiPlyPointCloud(filePath, nullptr);
		if (!cloudNode.valid())
		{
			osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
			const QByteArray nativePath = QFile::encodeName(filePath);
			cloudNode = osgDB::readRefNodeFile(std::string(nativePath.constData()), options.get());
			if (!cloudNode.valid())
			{
				cloudNode = osgDB::readRefNodeFile(filePath.toStdString(), options.get());
			}
		}
		if (!cloudNode.valid() && errorMessage && !plyErr.empty())
		{
			*errorMessage = QString::fromStdString(plyErr);
		}
	}
	else
	{
		// PLY/LAZ/LAS depend on available OSG reader plugins.
		osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
		// Keep options empty for point clouds; plugin-specific option strings can break readers.
		auto tryReadNode = [&](const std::string& path) -> osg::ref_ptr<osg::Node> {
			return osgDB::readRefNodeFile(path, options.get());
		};

		// 1) Native local 8-bit path first (Windows compatibility for non-ASCII paths).
		const QByteArray nativePath = QFile::encodeName(filePath);
		cloudNode = tryReadNode(std::string(nativePath.constData()));
		// 2) UTF-8/std path fallback.
		if (!cloudNode.valid())
		{
			cloudNode = tryReadNode(filePath.toStdString());
		}

		if (!cloudNode.valid() && errorMessage)
		{
			if (extension == QStringLiteral("las") || extension == QStringLiteral("laz"))
			{
				*errorMessage = QStringLiteral(
					"Failed to load '.%1'. OSG usually needs an external LAS/LAZ plugin. "
					"Try converting to .ply/.xyz first.")
					.arg(extension);
			}
			else
			{
				*errorMessage = QStringLiteral(
					"Failed to load point cloud. Check plugin support and path encoding for '.%1'.")
					.arg(extension);
			}
		}
	}

	if (!cloudNode.valid())
	{
		return false;
	}

	self.clearStagingGeometry();
	self.m_stagingGroup->addChild(cloudNode.get());
	self.m_modelCenter = cloudNode->getBound().center();
	self.detachGizmoOverlay();
	self.syncCompassGizmoOrientation();
	self.cachePickablePointsFromNode(cloudNode.get());
	self.clearPointAnnotations();
	self.clearPointPickMarker();
	self.applyColorToStagingGeometry(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
	self.setSelectionActive(true);
	emit self.selectedObjectPoseChanged(0.0f, 0.0f, 0.0f);

	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
		if (self.m_backendObjectRoots.empty())
		{
			self.m_viewer->home();
			if (osg::Camera* cam = self.m_viewer->getCamera())
			{
				double aspect = 1.0;
				const osg::Viewport* vp = cam->getViewport();
				if (vp && vp->width() > 0 && vp->height() > 0)
				{
					aspect = static_cast<double>(vp->width()) / static_cast<double>(vp->height());
				}
				cam->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
			}
		}
	}
	return true;
}

