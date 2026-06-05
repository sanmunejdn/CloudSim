#include "pch.h"

#include "OsgScene.h"

#include "BackendIdUserData.h"
#include "BackendPickDomain.h"
#include "BrepPickIndex.h"

#include <ShapeHandle.h>
#include <ShapeQuery.h>
#include <Discretize.h>
#include <Types.h>

#include <algorithm>
#include <limits>

#include <osgUtil/LineSegmentIntersector>

namespace
{

osg::NodePath nodePathToSceneRootFromLeaf(const osg::Node* leaf)
{
	osg::NodePath path;
	for (const osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), const_cast<osg::Node*>(n));
	}
	return path;
}

const BackendIdUserData* backendMetaFromRoot(const osg::Node* rootNode)
{
	if (!rootNode)
	{
		return nullptr;
	}
	return dynamic_cast<const BackendIdUserData*>(rootNode->getUserData());
}

bool brepShapeForBackend(
	const std::unordered_map<std::string, osg::ref_ptr<osg::MatrixTransform>>& roots,
	const std::string& backendId,
	geoalgo::ShapeHandle& outShape)
{
	outShape = geoalgo::ShapeHandle{};
	const auto it = roots.find(backendId);
	if (it == roots.end() || !it->second.valid())
	{
		return false;
	}
	const BackendIdUserData* meta = backendMetaFromRoot(it->second.get());
	if (!meta || meta->pickDomain() != BackendPickDomain::Brep || !meta->hasBrepShape())
	{
		return false;
	}
	outShape = meta->brepShape();
	return !outShape.isNull();
}

const BrepPickIndex* brepPickIndexForBackend(
	const BackendPickIndexRegistry& registry,
	const std::string& backendId)
{
	const BackendPickBundle* bundle = registry.find(backendId);
	if (!bundle || bundle->brepIndex.empty())
	{
		return nullptr;
	}
	return &bundle->brepIndex;
}

unsigned triangleIndexFromHit(const osgUtil::LineSegmentIntersector::Intersection& hit)
{
	if (hit.primitiveIndex >= 0)
	{
		return static_cast<unsigned int>(hit.primitiveIndex);
	}
	if (!hit.indexList.empty())
	{
		const unsigned int i0 = hit.indexList[0];
		return i0 / 3U;
	}
	return 0U;
}

bool isTriangleMeshIntersection(const osgUtil::LineSegmentIntersector::Intersection& hit)
{
	const osg::Drawable* drawable = hit.drawable.get();
	const osg::Geometry* geom = drawable ? drawable->asGeometry() : nullptr;
	if (!geom || geom->getNumPrimitiveSets() == 0U)
	{
		return false;
	}
	const osg::PrimitiveSet* ps = geom->getPrimitiveSet(0);
	if (!ps)
	{
		return false;
	}
	const GLenum mode = ps->getMode();
	return mode == GL_TRIANGLES || mode == GL_TRIANGLE_STRIP || mode == GL_TRIANGLE_FAN;
}

const osgUtil::LineSegmentIntersector::Intersection* chooseNearestBrepHit(
	const OsgScene& scene,
	osgUtil::LineSegmentIntersector& intersector,
	const std::string& scopeBackendId,
	const std::unordered_map<std::string, osg::ref_ptr<osg::MatrixTransform>>& roots,
	std::string& outBackendId,
	geoalgo::ShapeHandle& outShape)
{
	const osgUtil::LineSegmentIntersector::Intersection* chosenHit = nullptr;
	double bestDistance = (std::numeric_limits<double>::max)();

	const auto consider = [&](const osgUtil::LineSegmentIntersector::Intersection& candidate) {
		if (!isTriangleMeshIntersection(candidate))
		{
			return;
		}
		std::string id;
		if (!scene.resolveBackendIdFromPickedPath(candidate.nodePath, id))
		{
			return;
		}
		if (!scopeBackendId.empty() && id != scopeBackendId)
		{
			return;
		}
		geoalgo::ShapeHandle candidateShape;
		if (!brepShapeForBackend(roots, id, candidateShape))
		{
			return;
		}
		if (!chosenHit || candidate.ratio < bestDistance)
		{
			chosenHit = &candidate;
			bestDistance = candidate.ratio;
			outBackendId = std::move(id);
			outShape = std::move(candidateShape);
		}
	};

	for (const auto& candidate : intersector.getIntersections())
	{
		consider(candidate);
	}
	return chosenHit;
}

} // namespace

bool OsgScene::isBrepPickBackend(const std::string& backendId) const
{
	geoalgo::ShapeHandle shape;
	return brepShapeForBackend(m_backendObjectRoots, backendId, shape);
}

bool OsgScene::getWorldPickRay(double mouseX, double mouseY, osg::Vec3d& outStart, osg::Vec3d& outEnd) const
{
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return false;
	}
	double windowX = 0.0;
	double windowY = 0.0;
	logicalMouseToPickWindowCoords(mouseX, mouseY, windowX, windowY);
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, windowX, windowY);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.setTraversalMask(kMaskPickContent);
	m_viewer->getCamera()->accept(iv);
	if (!intersector->containsIntersections())
	{
		return false;
	}
	outStart = intersector->getStart();
	outEnd = intersector->getEnd();
	return true;
}

bool OsgScene::backendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const
{
	const auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return false;
	}
	outWorld = osg::computeLocalToWorld(nodePathToSceneRootFromLeaf(it->second.get()));
	return true;
}

bool OsgScene::worldPointToStepModelMm(const std::string& backendId, const osg::Vec3d& worldMm, geoalgo::Point3d& outModel) const
{
	osg::Matrixd worldMat;
	if (!backendRootWorldMatrix(backendId, worldMat))
	{
		return false;
	}
	osg::Matrixd invMat;
	if (!invMat.invert(worldMat))
	{
		return false;
	}
	const osg::Vec3d pOuter = worldMm * invMat;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	const auto skipIt = m_backendSkipCenterRebase.find(backendId);
	const bool skipCenter = skipIt != m_backendSkipCenterRebase.end() && skipIt->second;
	if (!skipCenter)
	{
		const auto cit = m_backendModelCenters.find(backendId);
		if (cit != m_backendModelCenters.end())
		{
			cx = static_cast<double>(cit->second.x());
			cy = static_cast<double>(cit->second.y());
			cz = static_cast<double>(cit->second.z());
		}
	}
	outModel.x = pOuter.x() + cx;
	outModel.y = pOuter.y() + cy;
	outModel.z = pOuter.z() + cz;
	return true;
}

bool OsgScene::stepModelPointToWorldMm(const std::string& backendId, const geoalgo::Point3d& modelMm, osg::Vec3f& outWorld) const
{
	osg::Matrixd worldMat;
	if (!backendRootWorldMatrix(backendId, worldMat))
	{
		return false;
	}
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	const auto skipIt = m_backendSkipCenterRebase.find(backendId);
	const bool skipCenter = skipIt != m_backendSkipCenterRebase.end() && skipIt->second;
	if (!skipCenter)
	{
		const auto cit = m_backendModelCenters.find(backendId);
		if (cit != m_backendModelCenters.end())
		{
			cx = static_cast<double>(cit->second.x());
			cy = static_cast<double>(cit->second.y());
			cz = static_cast<double>(cit->second.z());
		}
	}
	const osg::Vec3d pFile(modelMm.x - cx, modelMm.y - cy, modelMm.z - cz);
	const osg::Vec3d pw = pFile * worldMat;
	outWorld.set(static_cast<float>(pw.x()), static_cast<float>(pw.y()), static_cast<float>(pw.z()));
	return true;
}

bool OsgScene::tryQueryBrepPick(const PickQuery& query, bool pickFace, PickResult& out) const
{
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())
	{
		return false;
	}

	double windowX = 0.0;
	double windowY = 0.0;
	logicalMouseToPickWindowCoords(query.screenX, query.screenY, windowX, windowY);
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, windowX, windowY);
	intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.setTraversalMask(kMaskPickContent);
	m_viewer->getCamera()->accept(iv);
	if (!intersector->containsIntersections())
	{
		return false;
	}

	std::string backendId;
	geoalgo::ShapeHandle shape;
	const osgUtil::LineSegmentIntersector::Intersection* chosenHit = chooseNearestBrepHit(
		*this,
		*intersector,
		query.scopeBackendId,
		m_backendObjectRoots,
		backendId,
		shape);
	if (!chosenHit || backendId.empty() || shape.isNull())
	{
		return false;
	}
	const std::string xformBackendId = resolvePickScopeBackendId(backendId);

	const BrepPickIndex* brepIndex = brepPickIndexForBackend(m_backendPickIndexes, xformBackendId);
	const osg::Vec3d worldHit = chosenHit->getWorldIntersectPoint();
	geoalgo::Point3d hitModel;
	if (!worldPointToStepModelMm(xformBackendId, worldHit, hitModel))
	{
		return false;
	}

	const unsigned triIndex = triangleIndexFromHit(*chosenHit);
	int faceIndex = -1;
	if (brepIndex)
	{
		faceIndex = brepIndex->faceIndexForTriangle(triIndex);
	}

	geoalgo::ShapeRayPickResult pick;
	std::string err;
	if (pickFace)
	{
		if (faceIndex < 0)
		{
			osg::Vec3d worldStart;
			osg::Vec3d worldEnd;
			if (!getWorldPickRay(query.screenX, query.screenY, worldStart, worldEnd))
			{
				return false;
			}
			osg::Vec3d worldDir = worldEnd - worldStart;
			const double dirLen = worldDir.length();
			if (dirLen < 1e-12)
			{
				return false;
			}
			worldDir /= dirLen;

			geoalgo::Point3d rayOriginModel;
			if (!worldPointToStepModelMm(xformBackendId, worldStart, rayOriginModel))
			{
				return false;
			}
			osg::Matrixd worldMat;
			osg::Matrixd invMat;
			if (!backendRootWorldMatrix(xformBackendId, worldMat) || !invMat.invert(worldMat))
			{
				return false;
			}
			worldMat.setTrans(0.0, 0.0, 0.0);
			invMat.setTrans(0.0, 0.0, 0.0);
			const osg::Vec3d modelDirV = worldDir * invMat;
			const double modelDirLen = modelDirV.length();
			if (modelDirLen < 1e-12)
			{
				return false;
			}
			const geoalgo::Point3d rayDirModel{
				modelDirV.x() / modelDirLen, modelDirV.y() / modelDirLen, modelDirV.z() / modelDirLen};
			if (!geoalgo::pickShapeFaceByModelRay(shape, rayOriginModel, rayDirModel, pick, &err))
			{
				return false;
			}
			faceIndex = pick.faceIndex;
		}
		else
		{
			pick.hitPointModelMm = hitModel;
		}
		if (faceIndex < 0)
		{
			return false;
		}
		if (!query.hoverPick && !geoalgo::validateShapeFaceIndex(shape, faceIndex, &err))
		{
			return false;
		}
	}
	else
	{
		const osg::Matrixd mvp = m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
		int edgeIndex = -1;
		double edgeDistPx = 0.0;
		std::vector<osg::Vec3f> polyWorld;
		const auto modelToWorld = [this, &xformBackendId](const geoalgo::Point3d& mp, osg::Vec3f& wp) {
			return stepModelPointToWorldMm(xformBackendId, mp, wp);
		};
		if (brepIndex)
		{
			double devicePickX = 0.0;
			double devicePickY = 0.0;
			logicalMouseToDeviceCoords(query.screenX, query.screenY, devicePickX, devicePickY);
			const double dpr = (m_devicePixelRatio > 0.0) ? m_devicePixelRatio : 1.0;
			(void)brepIndex->pickEdgeByScreen(
				faceIndex,
				devicePickX,
				devicePickY,
				mvp,
				viewportWidth(),
				viewportHeight(),
				kMeshEdgeHitRadiusPx * dpr,
				modelToWorld,
				edgeIndex,
				edgeDistPx,
				polyWorld);
		}
		if (edgeIndex < 0)
		{
			if (query.hoverPick)
			{
				return false;
			}
			double edgeTolMm = 5.0;
			const auto rootIt = m_backendObjectRoots.find(xformBackendId);
			if (rootIt != m_backendObjectRoots.end() && rootIt->second.valid())
			{
				const osg::BoundingSphere bs = rootIt->second->getBound();
				if (bs.radius() > 1e-6f)
				{
					edgeTolMm = static_cast<double>(bs.radius()) * 0.02;
					edgeTolMm = (std::max)(5.0, (std::min)(edgeTolMm, 50.0));
				}
			}
			if (!geoalgo::pickShapeEdgeByModelPoint(shape, hitModel, edgeTolMm, pick, &err) || pick.edgeIndex < 0)
			{
				return false;
			}
			edgeIndex = pick.edgeIndex;
			if (brepIndex)
			{
				const std::vector<float>& pl = brepIndex->edgePolylineModel(edgeIndex);
				polyWorld.clear();
				polyWorld.reserve(pl.size() / 3U);
				for (std::size_t i = 0; i + 2U < pl.size(); i += 3U)
				{
					const geoalgo::Point3d mp{ pl[i], pl[i + 1U], pl[i + 2U] };
					osg::Vec3f wp;
					if (stepModelPointToWorldMm(xformBackendId, mp, wp))
					{
						polyWorld.push_back(wp);
					}
				}
			}
			edgeDistPx = 0.0;
		}
		else
		{
			pick.edgeIndex = edgeIndex;
			pick.hitPointModelMm = hitModel;
		}
		if (edgeIndex < 0 && pick.edgeIndex >= 0)
		{
			edgeIndex = pick.edgeIndex;
		}
		if (edgeIndex < 0)
		{
			return false;
		}
		if (!query.hoverPick && !geoalgo::validateShapeEdgeIndex(shape, edgeIndex, &err))
		{
			return false;
		}
		out.brepEdgeIndex = edgeIndex;
		out.screenDistancePx = edgeDistPx;
		out.meshEdgePolylineWorld = std::move(polyWorld);
		if (!out.meshEdgePolylineWorld.empty())
		{
			out.meshEdgeA = out.meshEdgePolylineWorld.front();
			out.meshEdgeB = out.meshEdgePolylineWorld.back();
		}
	}

	out.hit = true;
	out.backendId = backendId;
	out.brepNativePick = true;
	out.brepFaceIndex = pickFace ? faceIndex : -1;
	out.pickedTriangleIndex = static_cast<int>(triIndex);
	if (const BackendPickBundle* bundle = m_backendPickIndexes.find(xformBackendId))
	{
		out.indexGeneration = bundle->generation;
	}

	if (!stepModelPointToWorldMm(xformBackendId, pick.hitPointModelMm, out.worldPoint))
	{
		return false;
	}

	if (pickFace)
	{
		const std::vector<float>* soup = brepIndex ? &brepIndex->faceSoupModel(faceIndex) : nullptr;
		std::vector<float> fallbackSoup;
		if (!soup || soup->empty())
		{
			geoalgo::TessellateParams disc;
			disc.linearDeflectionMm = 0.01;
			disc.angularDeflectionDeg = 0.5;
			disc.linearDeflectionRelative = false;
			if (!geoalgo::discretizeShapeFaceByIndex(shape, faceIndex, disc, fallbackSoup, &err))
			{
				return false;
			}
			soup = &fallbackSoup;
		}
		out.meshFaceVertsWorld.clear();
		out.meshFaceVertsWorld.reserve(soup->size() / 3U);
		for (std::size_t i = 0; i + 2U < soup->size(); i += 3U)
		{
			const geoalgo::Point3d mp{ (*soup)[i], (*soup)[i + 1U], (*soup)[i + 2U] };
			osg::Vec3f wp;
			if (stepModelPointToWorldMm(xformBackendId, mp, wp))
			{
				out.meshFaceVertsWorld.push_back(wp);
			}
		}
		if (out.meshFaceVertsWorld.size() < 3U)
		{
			return false;
		}
		const osg::Vec3f& a = out.meshFaceVertsWorld[0];
		const osg::Vec3f& b = out.meshFaceVertsWorld[1];
		const osg::Vec3f& c = out.meshFaceVertsWorld[2];
		osg::Vec3f n = (b - a) ^ (c - a);
		if (n.length2() > 1e-12f)
		{
			n.normalize();
		}
		else
		{
			n.set(0.0f, 0.0f, 1.0f);
		}
		out.meshNormalWorld = n;
	}

	return true;
}
