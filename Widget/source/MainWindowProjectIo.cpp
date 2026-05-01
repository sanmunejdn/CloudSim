#include "MainWindow.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <QByteArray>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QMessageBox>
#include <QTemporaryDir>

#include <osg/Vec3f>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "ProjectPackageZip.h"
#include "RunInfoPage.h"

namespace {

std::string qStringToUtf8StdString(const QString& q)
{
	const QByteArray b = q.toUtf8();
	return std::string(b.constData(), static_cast<std::size_t>(b.size()));
}

constexpr quint32 kPointCloudBinMagic = 0x31435050u;

bool readPointCloudBinaryFile(const QString& absolutePath, PointCloudBackendData& pc, QString* errMsg)
{
	QFile f(absolutePath);
	if (!f.open(QIODevice::ReadOnly))
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("Cannot open point blob: %1").arg(absolutePath);
		}
		return false;
	}
	QDataStream ds(&f);
	ds.setByteOrder(QDataStream::LittleEndian);
	quint32 magic = 0;
	quint32 version = 0;
	quint64 n = 0;
	quint32 hasRgba = 0;
	ds >> magic >> version >> n >> hasRgba;
	if (magic != kPointCloudBinMagic || version != 1u || n == 0u)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("Invalid point blob header.");
		}
		return false;
	}
	const std::size_t xyzCount = static_cast<std::size_t>(n) * 3U;
	std::vector<float> xyz(xyzCount);
	const int xyzBytes = static_cast<int>(xyz.size() * sizeof(float));
	if (ds.readRawData(reinterpret_cast<char*>(xyz.data()), xyzBytes) != xyzBytes)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("Truncated XYZ payload.");
		}
		return false;
	}
	std::vector<float> rgba;
	if (hasRgba)
	{
		rgba.resize(static_cast<std::size_t>(n) * 4U);
		const int rgbaBytes = static_cast<int>(rgba.size() * sizeof(float));
		if (ds.readRawData(reinterpret_cast<char*>(rgba.data()), rgbaBytes) != rgbaBytes)
		{
			if (errMsg)
			{
				*errMsg = QStringLiteral("Truncated RGBA payload.");
			}
			return false;
		}
	}
	pc.setPointBuffers(std::move(xyz), std::move(rgba));
	return true;
}

QString mergeBase64Shards(const QJsonObject& emb, const QString& singleKey, const QString& shardsKey)
{
	QString merged = emb.value(singleKey).toString();
	if (!merged.isEmpty())
	{
		return merged;
	}
	const QJsonArray shards = emb.value(shardsKey).toArray();
	for (const QJsonValue& v : shards)
	{
		merged += v.toString();
	}
	return merged;
}

QString sanitizedArchiveBase(const QString& base)
{
	QString s = base.trimmed();
	for (int i = 0; i < s.length(); ++i)
	{
		const QChar c = s.at(i);
		if (c == QLatin1Char('/') || c == QLatin1Char('\\') || c == QLatin1Char(':') || c == QLatin1Char('*')
			|| c == QLatin1Char('?') || c == QLatin1Char('"') || c == QLatin1Char('<') || c == QLatin1Char('>')
			|| c == QLatin1Char('|'))
		{
			s[i] = QLatin1Char('_');
		}
	}
	if (s.isEmpty())
	{
		s = QStringLiteral("project");
	}
	return s;
}

void rebuildLegacyParentMirror(DocumentPage* page)
{
	if (!page)
	{
		return;
	}
	QMap<QString, QString>& parentMap = page->backendParentId();
	parentMap.clear();
	const auto all = page->backend().listData();
	for (const auto& data : all)
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		const std::vector<std::string> parents = page->backend().parentsOf(data->id());
		if (parents.empty())
		{
			parentMap[id] = QString();
			continue;
		}
		parentMap[id] = QString::fromStdString(parents.front());
	}
}

void applyPointCloudPoseFromJson(PointCloudBackendData& pc, OsgWidget* osgWidget, const QJsonObject& obj)
{
	const QJsonObject pose = obj.value(QStringLiteral("pose")).toObject();
	const QJsonObject rot = obj.value(QStringLiteral("rotation")).toObject();
	const QJsonObject col = obj.value(QStringLiteral("color")).toObject();
	const BackendVec3 p{
		pose.value(QStringLiteral("x")).toDouble(),
		pose.value(QStringLiteral("y")).toDouble(),
		pose.value(QStringLiteral("z")).toDouble()
	};
	const BackendVec3 r{
		rot.value(QStringLiteral("x")).toDouble(),
		rot.value(QStringLiteral("y")).toDouble(),
		rot.value(QStringLiteral("z")).toDouble()
	};
	const BackendColor c{
		static_cast<float>(col.value(QStringLiteral("r")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("g")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("b")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("a")).toDouble(1.0))
	};
	pc.setPose(p);
	pc.setRotation(r);
	pc.setColor(c);
	if (osgWidget)
	{
		osgWidget->setSelectedPosition(osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)));
		osgWidget->setSelectedRotationEulerDeg(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
		osgWidget->setSelectedColor(c.r, c.g, c.b, c.a);
	}
}

} // namespace

void MainWindow::onSaveProject()
{
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return;
	}
	const QString savePath = QFileDialog::getSaveFileName(
		this,
		i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
		QString(),
		QStringLiteral("Point Cloud Package (*.pcp);;PointCloud Project (*.pcproj.json);;JSON Files (*.json);;All Files (*.*)"));
	if (savePath.isEmpty())
	{
		return;
	}

	const QFileInfo saveFileInfo(savePath);
	const bool packageMode = saveFileInfo.suffix().compare(QStringLiteral("pcp"), Qt::CaseInsensitive) == 0;
	QTemporaryDir packageTemp;
	const QString workRoot = packageMode ? packageTemp.path() : saveFileInfo.absolutePath();
	if (packageMode && !packageTemp.isValid())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
			i18n(QStringLiteral("Cannot create a temporary folder to build the package."),
				QStringLiteral("无法创建临时目录以打包工程。")));
		return;
	}
	const QString jsonWritePath = packageMode ? QDir(workRoot).filePath(QStringLiteral("project.json")) : savePath;

	QJsonObject root;
	root.insert(QStringLiteral("version"), 3);
	if (packageMode)
	{
		root.insert(QStringLiteral("bundle"), QStringLiteral("zip"));
	}
	root.insert(QStringLiteral("language"), m_useChinese ? QStringLiteral("zh") : QStringLiteral("en"));

	QJsonArray objects;
	const auto dataList = doc->backend().listData();
	const int pointCloudObjectCount = static_cast<int>(std::count_if(dataList.begin(), dataList.end(),
		[](const std::shared_ptr<BackendDataBase>& d) {
			return d && std::dynamic_pointer_cast<PointCloudBackendData>(d);
		}));
	int pointCloudBlobIndex = 0;

	for (const auto& data : dataList)
	{
		if (!data) continue;
		QJsonObject obj;
		const std::string id = data->id();
		const QString idQs = QString::fromStdString(id);
		obj.insert(QStringLiteral("id"), QString::fromStdString(id));
		obj.insert(QStringLiteral("name"), QString::fromStdString(data->name()));
		obj.insert(QStringLiteral("className"), QString::fromStdString(data->className()));
		const QString srcPath = doc->backendSourcePath().count(idQs) ? doc->backendSourcePath()[idQs] : QString();
		obj.insert(QStringLiteral("sourcePath"), srcPath);
		obj.insert(QStringLiteral("sourceType"), doc->backendSourceType().count(idQs) ? doc->backendSourceType()[idQs] : QString());
		const std::vector<std::string> parentIds = doc->backend().parentsOf(id);
		obj.insert(QStringLiteral("parentId"), parentIds.empty() ? QString() : QString::fromStdString(parentIds.front()));

		if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
		{
			if (pc->pointPositionsXyz().empty() && pointCloudObjectCount == 1 && doc->osgWidget())
			{
				QString resyncErr;
				if (!doc->osgWidget()->captureImportedPointCloudBackend(*pc, &resyncErr) && m_runInfoPage)
				{
					m_runInfoPage->appendWarning(QStringLiteral("Save: could not embed point cloud from viewer: %1").arg(resyncErr));
				}
			}
			const BackendVec3 p = pc->pose();
			const BackendVec3 r = pc->rotation();
			const BackendColor c = pc->color();
			QJsonObject pose; pose.insert(QStringLiteral("x"), p.x); pose.insert(QStringLiteral("y"), p.y); pose.insert(QStringLiteral("z"), p.z);
			QJsonObject rot; rot.insert(QStringLiteral("x"), r.x); rot.insert(QStringLiteral("y"), r.y); rot.insert(QStringLiteral("z"), r.z);
			QJsonObject col; col.insert(QStringLiteral("r"), c.r); col.insert(QStringLiteral("g"), c.g); col.insert(QStringLiteral("b"), c.b); col.insert(QStringLiteral("a"), c.a);
			obj.insert(QStringLiteral("pose"), pose);
			obj.insert(QStringLiteral("rotation"), rot);
			obj.insert(QStringLiteral("color"), col);
			if (pc->pointPositionsXyz().empty())
			{
				QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
					i18n(QStringLiteral("Point cloud has no coordinates in the backend; cannot save. Re-import the file while it still exists on disk."),
						QStringLiteral("后端没有点云坐标，无法保存。请在源文件仍在磁盘上时重新导入。")));
				return;
			}
			QString sidecarRelative;
			QString sidecarAbs;
			if (packageMode)
			{
				const QString ab = sanitizedArchiveBase(saveFileInfo.completeBaseName());
				sidecarRelative = QStringLiteral("assets/%1_pcloud%2.ply").arg(ab).arg(pointCloudBlobIndex++);
				sidecarAbs = QDir(workRoot).filePath(sidecarRelative);
				QDir().mkpath(QFileInfo(sidecarAbs).absolutePath());
			}
			else
			{
				sidecarRelative = QStringLiteral("%1.pcloud%2.ply")
					.arg(saveFileInfo.completeBaseName())
					.arg(pointCloudBlobIndex++);
				sidecarAbs = saveFileInfo.absolutePath() + QLatin1Char('/') + sidecarRelative;
			}
			std::string plyErr;
			if (!pc->writePointCloudPlySidecar(qStringToUtf8StdString(sidecarAbs), &plyErr))
			{
				const QString blobErr = QString::fromStdString(plyErr);
				QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
					i18n(QStringLiteral("Failed to write point cloud data file: %1").arg(blobErr),
						QStringLiteral("写点云数据文件失败：%1").arg(blobErr)));
				return;
			}
			const std::size_t npts = pc->pointPositionsXyz().size() / 3U;
			QJsonObject emb;
			emb.insert(QStringLiteral("kind"), QStringLiteral("points"));
			emb.insert(QStringLiteral("encoding"), QStringLiteral("cgal_ply_sidecar"));
			emb.insert(QStringLiteral("pointCount"), QJsonValue(static_cast<double>(npts)));
			emb.insert(QStringLiteral("sidecar"), sidecarRelative);
			obj.insert(QStringLiteral("embeddedGeometry"), emb);
		}
		else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
		{
			std::string soupB64;
			if (mesh->writeProjectEmbeddedGeometry(soupB64))
			{
				QJsonObject emb;
				emb.insert(QStringLiteral("kind"), QStringLiteral("triangles"));
				emb.insert(QStringLiteral("encoding"), QStringLiteral("float32_le"));
				emb.insert(QStringLiteral("xyzBase64"), QString::fromStdString(soupB64));
				obj.insert(QStringLiteral("embeddedGeometry"), emb);
			}
		}
		objects.push_back(obj);
	}
	root.insert(QStringLiteral("objects"), objects);
	QJsonArray edgeArray;
	for (const auto& edge : doc->backend().listEdges())
	{
		QJsonObject edgeObj;
		edgeObj.insert(QStringLiteral("parentId"), QString::fromStdString(edge.first));
		edgeObj.insert(QStringLiteral("childId"), QString::fromStdString(edge.second));
		edgeArray.push_back(edgeObj);
	}
	root.insert(QStringLiteral("edges"), edgeArray);

	QJsonArray annArray;
	if (OsgWidget* w = doc->osgWidget())
	{
		const auto snapshots = w->annotationSnapshots();
		for (const auto& s : snapshots)
		{
			QJsonObject a;
			a.insert(QStringLiteral("id"), s.id);
			a.insert(QStringLiteral("displayText"), s.displayText);
			a.insert(QStringLiteral("backendId"), s.backendId);
			QJsonObject local;
			local.insert(QStringLiteral("x"), s.localCentered.x());
			local.insert(QStringLiteral("y"), s.localCentered.y());
			local.insert(QStringLiteral("z"), s.localCentered.z());
			a.insert(QStringLiteral("localCentered"), local);
			if (s.hasWorldAnchor)
			{
				QJsonObject w;
				w.insert(QStringLiteral("x"), s.worldAnchor.x());
				w.insert(QStringLiteral("y"), s.worldAnchor.y());
				w.insert(QStringLiteral("z"), s.worldAnchor.z());
				a.insert(QStringLiteral("worldAnchor"), w);
			}
			a.insert(QStringLiteral("visible"), s.visible);
			annArray.push_back(a);
		}
	}
	root.insert(QStringLiteral("annotations"), annArray);

	QFile file(jsonWritePath);
	if (!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
			i18n(QStringLiteral("Failed to write project file."), QStringLiteral("写入工程文件失败。")));
		return;
	}
	file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	file.close();

	if (packageMode)
	{
		QString zipErr;
		if (!project_package_zip::zipDirectoryTree(savePath, workRoot, &zipErr))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
				i18n(QStringLiteral("Failed to write package (.pcp): %1").arg(zipErr),
					QStringLiteral("写入打包文件 (.pcp) 失败：%1").arg(zipErr)));
			return;
		}
	}

	if (m_runInfoPage) m_runInfoPage->appendInfo(QStringLiteral("Project saved: %1").arg(savePath));
	doc->setProjectFilePath(savePath);
	if (m_documentTabs)
	{
		const int idx = m_documentTabs->indexOf(doc);
		if (idx >= 0)
		{
			m_documentTabs->setTabText(idx, saveFileInfo.fileName());
		}
	}
}

void MainWindow::onOpenProjectFile()
{
	DocumentPage* page = currentPage();
	if (!page)
	{
		return;
	}
	const QString openPath = QFileDialog::getOpenFileName(
		this,
		i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
		QString(),
		QStringLiteral("Point Cloud Package (*.pcp);;PointCloud Project (*.pcproj.json);;JSON Files (*.json);;All Files (*.*)"));
	if (openPath.isEmpty())
	{
		return;
	}

	QTemporaryDir zipExtractDir;
	QString projectJsonPath = openPath;
	QString projectDir = QFileInfo(openPath).absolutePath();
	if (project_package_zip::isZipArchiveFile(openPath))
	{
		if (!zipExtractDir.isValid())
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
				i18n(QStringLiteral("Cannot create a temporary folder to unpack the project."),
					QStringLiteral("无法创建临时目录解压工程。")));
			return;
		}
		QString unpackErr;
		if (!project_package_zip::extractZipArchive(openPath, zipExtractDir.path(), &unpackErr))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")), unpackErr);
			return;
		}
		projectJsonPath = QDir(zipExtractDir.path()).filePath(QStringLiteral("project.json"));
		projectDir = zipExtractDir.path();
		if (!QFileInfo::exists(projectJsonPath))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
				i18n(QStringLiteral("The archive does not contain project.json."),
					QStringLiteral("压缩包中没有 project.json。")));
			return;
		}
	}

	QFile file(projectJsonPath);
	if (!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
			i18n(QStringLiteral("Failed to open project file."), QStringLiteral("打开工程文件失败。")));
		return;
	}
	const QJsonDocument jsonDoc = QJsonDocument::fromJson(file.readAll());
	file.close();
	if (!jsonDoc.isObject())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
			i18n(QStringLiteral("Invalid project format."), QStringLiteral("工程文件格式无效。")));
		return;
	}

	page->backend().clear();
	page->backendSourcePath().clear();
	page->backendSourceType().clear();
	page->backendParentId().clear();
	if (OsgWidget* wClear = page->osgWidget())
	{
		wClear->clearAllAnnotations();
	}

	OsgWidget* osg = page->osgWidget();

	const QJsonObject root = jsonDoc.object();
	const QJsonArray edgesJson = root.value(QStringLiteral("edges")).toArray();
	std::vector<std::pair<QString, QString>> pendingEdges;
	pendingEdges.reserve(static_cast<std::size_t>(edgesJson.size()));
	for (const QJsonValue& v : edgesJson)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject edgeObj = v.toObject();
		const QString parentId = edgeObj.value(QStringLiteral("parentId")).toString();
		const QString childId = edgeObj.value(QStringLiteral("childId")).toString();
		if (parentId.isEmpty() || childId.isEmpty() || parentId == childId)
		{
			continue;
		}
		pendingEdges.emplace_back(parentId, childId);
	}
	const bool useEdgesRelation = !pendingEdges.empty();
	const QJsonArray objects = root.value(QStringLiteral("objects")).toArray();
	for (const auto& v : objects)
	{
		if (!v.isObject()) continue;
		const QJsonObject obj = v.toObject();
		const QString sourcePath = obj.value(QStringLiteral("sourcePath")).toString();
		const QString assetRelativePath = obj.value(QStringLiteral("assetRelativePath")).toString();
		const QString sourceType = obj.value(QStringLiteral("sourceType")).toString();
		const QString legacyParentId = obj.value(QStringLiteral("parentId")).toString();
		const QString persistedId = obj.value(QStringLiteral("id")).toString();
		const QString classNameVal = obj.value(QStringLiteral("className")).toString();
		const QJsonObject emb = obj.value(QStringLiteral("embeddedGeometry")).toObject();
		const QString embKind = emb.value(QStringLiteral("kind")).toString();
		const bool hasEmb = !emb.isEmpty() && !embKind.isEmpty();

		if (classNameVal == QStringLiteral("Compass")
			|| sourceType.compare(QStringLiteral("Compass"), Qt::CaseInsensitive) == 0)
		{
			continue;
		}

		if (!hasEmb && sourcePath.isEmpty() && assetRelativePath.isEmpty())
		{
			continue;
		}

		bool loaded = false;
		if (hasEmb && embKind == QStringLiteral("points")
			&& sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0)
		{
			auto pc = std::make_shared<PointCloudBackendData>();
			const QString objName = obj.value(QStringLiteral("name")).toString();
			if (!objName.isEmpty())
			{
				pc->setName(objName.toStdString());
			}
			applyPointCloudPoseFromJson(*pc, osg, obj);
			const QString enc = emb.value(QStringLiteral("encoding")).toString();
			bool geoOk = false;
			QString geoErr;
			if (enc == QStringLiteral("float32_le_sidecar") || enc == QStringLiteral("cgal_ply_sidecar"))
			{
				const QString rel = emb.value(QStringLiteral("sidecar")).toString();
				if (!rel.isEmpty())
				{
					const QString absBlob = QDir(projectDir).filePath(rel);
					if (absBlob.endsWith(QStringLiteral(".bin"), Qt::CaseInsensitive))
					{
						geoOk = readPointCloudBinaryFile(absBlob, *pc, &geoErr);
					}
					else
					{
						std::string plyErr;
						geoOk = pc->readPointCloudPlySidecar(qStringToUtf8StdString(absBlob), &plyErr);
						geoErr = QString::fromStdString(plyErr);
					}
				}
				else
				{
					geoErr = QStringLiteral("embeddedGeometry.sidecar is missing.");
				}
			}
			else
			{
				const QString xyzMerged = mergeBase64Shards(emb, QStringLiteral("xyzBase64"), QStringLiteral("xyzBase64Shards"));
				const QString rgbaMerged = mergeBase64Shards(emb, QStringLiteral("rgbaPerVertexBase64"),
					QStringLiteral("rgbaPerVertexBase64Shards"));
				geoOk = pc->readProjectEmbeddedGeometry(xyzMerged.toStdString(), rgbaMerged.toStdString());
				if (!geoOk)
				{
					geoErr = QStringLiteral("Invalid or missing inline base64 point data (xyzBase64).");
				}
			}
			if (geoOk)
			{
				QString err;
				if (osg && osg->loadPointCloudFromBackendData(*pc, &err)
					&& registerExistingBackendObject(
						pc,
						sourcePath,
						QStringLiteral("PointCloud"),
						persistedId,
						false,
						useEdgesRelation ? QString() : legacyParentId))
				{
					loaded = true;
				}
				else if (m_runInfoPage)
				{
					m_runInfoPage->appendWarning(QStringLiteral("Embedded point cloud failed: %1").arg(err));
				}
			}
			else if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Point cloud geometry: %1").arg(geoErr));
			}
		}
		else if (hasEmb && embKind == QStringLiteral("triangles"))
		{
			const std::string soupB64 = emb.value(QStringLiteral("xyzBase64")).toString().toStdString();
			auto mesh = std::make_shared<MeshBackendData>();
			const QString objName = obj.value(QStringLiteral("name")).toString();
			if (!objName.isEmpty())
			{
				mesh->setName(objName.toStdString());
			}
			if (mesh->readProjectEmbeddedGeometry(soupB64))
			{
				QString err;
				if (osg && osg->loadMeshFromBackendData(*mesh, &err)
					&& registerExistingBackendObject(
						mesh,
						sourcePath,
						QStringLiteral("Model"),
						persistedId,
						false,
						useEdgesRelation ? QString() : legacyParentId))
				{
					loaded = true;
				}
				else if (m_runInfoPage)
				{
					m_runInfoPage->appendWarning(QStringLiteral("Embedded mesh failed: %1").arg(err));
				}
			}
			else if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Invalid or missing embedded mesh (xyzBase64)."));
			}
		}

		if (loaded)
		{
			continue;
		}

		QString loadPath;
		if (!sourcePath.isEmpty() && QFileInfo::exists(sourcePath))
		{
			loadPath = sourcePath;
		}
		else if (!assetRelativePath.isEmpty())
		{
			const QString bundled = QDir(projectDir).filePath(assetRelativePath);
			if (QFileInfo::exists(bundled))
			{
				loadPath = QDir::cleanPath(bundled);
			}
		}
		if (loadPath.isEmpty())
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Missing data (no usable embedded geometry and file missing): %1")
					.arg(sourcePath.isEmpty() ? assetRelativePath : sourcePath));
			}
			continue;
		}
		const bool ok = registerBackendObject(loadPath,
			sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0
				? QStringLiteral("PointCloud")
				: QStringLiteral("Model"),
			sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0,
			true);
		if (!ok)
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Failed to load object from file: %1").arg(loadPath));
			}
			continue;
		}

		const auto all = page->backend().listData();
		if (!all.empty())
		{
			auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(all.back());
			if (pc)
			{
				applyPointCloudPoseFromJson(*pc, osg, obj);
			}
		}
	}

	if (useEdgesRelation)
	{
		for (const auto& edge : pendingEdges)
		{
			const std::string parentId = edge.first.toStdString();
			const std::string childId = edge.second.toStdString();
			if (!page->backend().contains(parentId) || !page->backend().contains(childId))
			{
				if (m_runInfoPage)
				{
					m_runInfoPage->appendWarning(
						QStringLiteral("Skip dangling edge: %1 -> %2").arg(edge.first, edge.second));
				}
				continue;
			}
			if (!page->backend().attachChild(parentId, childId) && m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("Skip invalid edge (cycle or duplicate): %1 -> %2").arg(edge.first, edge.second));
			}
		}
	}
	rebuildLegacyParentMirror(page);
	if (osg)
	{
		for (const auto& data : page->backend().listData())
		{
			if (!data)
			{
				continue;
			}
			const std::vector<std::string> parents = page->backend().parentsOf(data->id());
			const std::string parent = parents.empty() ? std::string() : parents.front();
			osg->setBackendParent(data->id(), parent);
		}
	}

	QList<OsgWidget::AnnotationSnapshot> snapshots;
	const QJsonArray annArray = root.value(QStringLiteral("annotations")).toArray();
	for (const QJsonValue& v : annArray)
	{
		if (!v.isObject()) continue;
		const QJsonObject a = v.toObject();
		OsgWidget::AnnotationSnapshot s;
		s.id = a.value(QStringLiteral("id")).toString();
		s.displayText = a.value(QStringLiteral("displayText")).toString();
		s.backendId = a.value(QStringLiteral("backendId")).toString();
		const QJsonObject local = a.value(QStringLiteral("localCentered")).toObject();
		s.localCentered = osg::Vec3f(
			static_cast<float>(local.value(QStringLiteral("x")).toDouble()),
			static_cast<float>(local.value(QStringLiteral("y")).toDouble()),
			static_cast<float>(local.value(QStringLiteral("z")).toDouble()));
		const QJsonObject world = a.value(QStringLiteral("worldAnchor")).toObject();
		if (!world.isEmpty())
		{
			s.worldAnchor = osg::Vec3f(
				static_cast<float>(world.value(QStringLiteral("x")).toDouble()),
				static_cast<float>(world.value(QStringLiteral("y")).toDouble()),
				static_cast<float>(world.value(QStringLiteral("z")).toDouble()));
			s.hasWorldAnchor = true;
		}
		s.visible = a.value(QStringLiteral("visible")).toBool(true);
		snapshots.push_back(s);
	}
	if (osg)
	{
		osg->restoreAnnotations(snapshots);
	}

	refreshBackendTree();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("Project opened: %1").arg(openPath));
	}
	page->setProjectFilePath(openPath);
	if (m_documentTabs)
	{
		const int idx = m_documentTabs->indexOf(page);
		if (idx >= 0)
		{
			m_documentTabs->setTabText(idx, QFileInfo(openPath).fileName());
		}
	}
}
