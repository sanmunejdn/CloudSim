#include "detail/OccIncludes.h"

#include "BrepBoolean.h"
#include "Discretize.h"
#include "Intersection.h"
#include "GeoMeshBoolean.h"
#include "MeshDiscretize.h"
#include "SelfTest.h"
#include "ShapeQuery.h"
#include "TemplateBrepUpdate.h"
#include "TemplateBrepRegistration.h"
#include "BrepImportArtifacts.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ViewTessellate.h"
#include "WireOps.h"
#include "MeshSurfaceReconstruction.h"
#include "MeshSurfaceReconstruction/NurbsSurfaceFitting.h"
#include "MeshSurfaceReconstruction/MeshSurfaceReconstructionAmrtoLoader.h"
#include "MeshSurfaceReconstruction/MeshSurfaceReconstructionInstantMeshes.h"
#include "MeshTrajectory.h"
#include "TubularGrinding.h"
#include "FeatureDiscretizerBridge.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <sstream>

#include <Eigen/Geometry>

namespace geoalgo
{
namespace
{

void applyIsometryInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& transform)
{
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0U; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(xyz[b], xyz[b + 1U], xyz[b + 2U]);
		p = transform * p;
		xyz[b] = static_cast<float>(p.x());
		xyz[b + 1U] = static_cast<float>(p.y());
		xyz[b + 2U] = static_cast<float>(p.z());
	}
}

double maxSampledPairDistanceMm(
	const std::vector<float>& aXyz,
	const std::vector<float>& bXyz,
	const std::size_t maxSamples = 512U)
{
	if (aXyz.size() < 9U || bXyz.size() < 9U)
	{
		return std::numeric_limits<double>::max();
	}
	const std::size_t nA = aXyz.size() / 3U;
	const std::size_t nB = bXyz.size() / 3U;
	const std::size_t strideA = std::max<std::size_t>(1U, nA / maxSamples);
	const std::size_t strideB = std::max<std::size_t>(1U, nB / 512U);
	double maxDist = 0.0;
	for (std::size_t i = 0U; i < nA; i += strideA)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d query(aXyz[b], aXyz[b + 1U], aXyz[b + 2U]);
		double bestSq = std::numeric_limits<double>::max();
		for (std::size_t j = 0U; j < nB; j += strideB)
		{
			const std::size_t tb = j * 3U;
			const double d2 =
				(query - Eigen::Vector3d(bXyz[tb], bXyz[tb + 1U], bXyz[tb + 2U])).squaredNorm();
			bestSq = std::min(bestSq, d2);
		}
		maxDist = std::max(maxDist, std::sqrt(bestSq));
	}
	return maxDist;
}

bool soupBoundingBoxDiagonal(const std::vector<float>& soup, double& outDiagonal)
{
	if (soup.size() < 9U)
	{
		return false;
	}
	double minX = soup[0];
	double minY = soup[1];
	double minZ = soup[2];
	double maxX = minX;
	double maxY = minY;
	double maxZ = minZ;
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		minX = std::min(minX, static_cast<double>(soup[i]));
		minY = std::min(minY, static_cast<double>(soup[i + 1U]));
		minZ = std::min(minZ, static_cast<double>(soup[i + 2U]));
		maxX = std::max(maxX, static_cast<double>(soup[i]));
		maxY = std::max(maxY, static_cast<double>(soup[i + 1U]));
		maxZ = std::max(maxZ, static_cast<double>(soup[i + 2U]));
	}
	const double dx = maxX - minX;
	const double dy = maxY - minY;
	const double dz = maxZ - minZ;
	outDiagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
	return true;
}

bool loadTriObjIndexedMesh(const std::string& objPath, meshrecon::IndexedMeshLite& outMesh, std::string* errMsg)
{
	outMesh = {};
	std::ifstream in(objPath);
	if (!in)
	{
		if (errMsg)
		{
			*errMsg = "cannot open tri obj: " + objPath;
		}
		return false;
	}
	std::string line;
	while (std::getline(in, line))
	{
		if (line.size() < 2U || line[0] == '#')
		{
			continue;
		}
		if (line[0] == 'v' && line[1] == ' ')
		{
			std::istringstream iss(line.substr(2));
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			iss >> x >> y >> z;
			outMesh.vertices.push_back(static_cast<float>(x));
			outMesh.vertices.push_back(static_cast<float>(y));
			outMesh.vertices.push_back(static_cast<float>(z));
		}
		else if (line[0] == 'f')
		{
			std::istringstream iss(line.substr(2));
			std::vector<int> verts;
			std::string tok;
			while (iss >> tok)
			{
				const std::size_t slash = tok.find('/');
				const int vi = std::stoi(slash == std::string::npos ? tok : tok.substr(0, slash)) - 1;
				verts.push_back(vi);
			}
			if (verts.size() == 3U)
			{
				outMesh.faces.push_back(verts[0]);
				outMesh.faces.push_back(verts[1]);
				outMesh.faces.push_back(verts[2]);
			}
			else if (verts.size() >= 4U)
			{
				for (std::size_t i = 1; i + 1U < verts.size(); ++i)
				{
					outMesh.faces.push_back(verts[0]);
					outMesh.faces.push_back(verts[static_cast<std::size_t>(i)]);
					outMesh.faces.push_back(verts[static_cast<std::size_t>(i) + 1U]);
				}
			}
		}
	}
	if (outMesh.vertices.empty() || outMesh.faces.empty())
	{
		if (errMsg)
		{
			*errMsg = "tri obj empty: " + objPath;
		}
		return false;
	}
	return true;
}

} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();
	auto fail = [&](const char* name, const std::string& err) {
		std::ostringstream oss;
		oss << name << ": " << err;
		failures.push_back(oss.str());
	};

	{
		std::string err;
		if (!meshBooleanRunSelfTest(&err))
		{
			fail("meshBoolean", err);
		}
	}

	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		MeshDiscretizeParams params;
		params.quality = MeshQualityPreset::Medium;
		std::vector<float> soup;
		MeshDiscretizeReport report;
		std::string err;
		if (!discretizeShapeToMesh(box, params, soup, report, &err))
		{
			fail("discretizeBox", err);
		}
		else if (report.triangleCount < 10U)
		{
			fail("discretizeBox", "too few triangles");
		}
	}

	{
		MeshDiscretizeParams fine = {};
		fine.quality = MeshQualityPreset::Fine;
		MeshDiscretizeParams medium = {};
		medium.quality = MeshQualityPreset::Medium;
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
		std::vector<float> soupFine;
		std::vector<float> soupMedium;
		MeshDiscretizeReport r1;
		MeshDiscretizeReport r2;
		std::string err;
		if (!discretizeShapeToMesh(box, fine, soupFine, r1, &err))
		{
			fail("qualityFine", err);
		}
		else if (!discretizeShapeToMesh(box, medium, soupMedium, r2, &err))
		{
			fail("qualityMedium", err);
		}
		else if (r1.triangleCount < r2.triangleCount)
		{
			fail("quality", "Fine should have >= triangles than Medium");
		}
	}

	{
		Polyline3d line;
		line.xyz = { 0.f, 0.f, 0.f, 100.f, 0.f, 0.f };
		MeshDiscretizeParams tube;
		tube.mode = MeshDiscretizeMode::WireTubeMesh;
		tube.tubeRadiusMm = 5.0;
		tube.tubeSides = 8;
		std::vector<float> soup;
		std::string err;
		if (!discretizePolylineToMesh(line, tube, soup, &err))
		{
			fail("wireTube", err);
		}
	}

	{
		const TopoDS_Shape plane = BRepPrimAPI_MakeBox(100.0, 100.0, 1.0).Shape();
		const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(25.0, 80.0).Shape();
		IntersectionParams ix;
		IntersectionResult result;
		std::string err;
		if (!intersectShapes(plane, cyl, ix, result, &err))
		{
			fail("intersectShapes", err);
		}
	}

	{
		const TopoDS_Shape a = BRepPrimAPI_MakeBox(40.0, 40.0, 40.0).Shape();
		const TopoDS_Shape b = BRepPrimAPI_MakeBox(gp_Pnt(20.0, 20.0, 20.0), 40.0, 40.0, 40.0).Shape();
		std::vector<float> soup;
		std::string err;
		if (!brepBooleanToMesh(a, b, BrepBooleanOp::Fuse, MeshDiscretizeParams{}, soup, &err))
		{
			fail("brepFuse", err);
		}
	}

	{
		const TopoDS_Edge e1 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(50, 0, 0));
		const TopoDS_Edge e2 = BRepBuilderAPI_MakeEdge(gp_Pnt(50, 0, 0), gp_Pnt(100, 0, 0));
		std::vector<TopoDS_Wire> wires;
		wires.push_back(BRepBuilderAPI_MakeWire(e1));
		wires.push_back(BRepBuilderAPI_MakeWire(e2));
		Polyline3d poly;
		std::string err;
		if (!fuseWiresToPolyline(wires, TessellateParams{}, poly, &err))
		{
			fail("fuseWires", err);
		}
	}

	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		const ShapeHandle handle = ShapeHandleAccess::fromNativeShape(&box);
		std::vector<float> perFaceSoup;
		std::vector<int> perFaceIndex;
		std::vector<std::vector<float>> faceSoups;
		std::string err;
		if (!tessellateShapePerFaceMedium(handle, perFaceSoup, perFaceIndex, &faceSoups, &err))
		{
			fail("tessellatePerFaceBox", err);
		}
		else
		{
			const std::size_t triCount = perFaceSoup.size() / 9U;
			if (triCount != perFaceIndex.size())
			{
				fail("tessellatePerFaceBox", "triangleFaceIndex size mismatch");
			}
			else if (triCount < 10U)
			{
				fail("tessellatePerFaceBox", "too few triangles");
			}
			else if (faceSoups.size() != 6U)
			{
				fail("tessellatePerFaceBox", "box should have 6 faces");
			}
			else
			{
				double diagPerFace = 0.0;
				if (!soupBoundingBoxDiagonal(perFaceSoup, diagPerFace))
				{
					fail("tessellatePerFaceBox", "bbox failed");
				}
				else if (std::abs(diagPerFace - 100.0 * std::sqrt(3.0)) > 1.0)
				{
					fail("tessellatePerFaceBox", "bbox diagonal out of tolerance");
				}
			}
		}
		TessellateParams disc;
		disc.linearDeflectionMm = 0.01;
		disc.angularDeflectionDeg = 0.5;
		disc.linearDeflectionRelative = false;
		std::vector<float> wholeSoup;
		std::vector<int> wholeIndex;
		if (!discretizeShapeToSoupPerFace(box, disc, wholeSoup, wholeIndex, nullptr, &err))
		{
			fail("discretizeShapeToSoupPerFace", err);
		}
		else
		{
			const std::size_t triWhole = wholeSoup.size() / 9U;
			if (triWhole != wholeIndex.size())
			{
				fail("discretizeShapeToSoupPerFace", "triangleFaceIndex size mismatch");
			}
			else if (triWhole < 10U)
			{
				fail("discretizeShapeToSoupPerFace", "too few triangles");
			}
		}
	}

	// TemplateBrepUpdate: 盒体 + 扫描点云逐面更新
	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		const ShapeHandle templateShape = ShapeHandleAccess::fromNativeShape(&box);
		TopoDS_Shape templateNative;
		if (!ShapeHandleAccess::nativeShape(templateShape, &templateNative))
		{
			fail("templateBrepUpdate", "template access failed");
		}
		else
		{
			const int templateFaceCount = shapeFaceCount(templateNative);

			auto addPlanarFace = [](
				std::vector<float>& xyz,
				std::vector<float>& normals,
				const float fixedCoord,
				const int fixedAxis,
				const float nx,
				const float ny,
				const float nz) {
				for (int i = 0; i < 10; ++i)
				{
					for (int j = 0; j < 10; ++j)
					{
						const float u = 10.0f + static_cast<float>(i * 8);
						const float v = 10.0f + static_cast<float>(j * 8);
						float px = 0.0f;
						float py = 0.0f;
						float pz = 0.0f;
						if (fixedAxis == 0)
						{
							px = fixedCoord;
							py = u;
							pz = v;
						}
						else if (fixedAxis == 1)
						{
							px = u;
							py = fixedCoord;
							pz = v;
						}
						else
						{
							px = u;
							py = v;
							pz = fixedCoord;
						}
						const float noise = 0.02f * static_cast<float>(i + j);
						if (fixedAxis == 0)
						{
							px += noise * ((fixedCoord > 50.0f) ? 1.0f : -1.0f);
						}
						else if (fixedAxis == 1)
						{
							py += noise * ((fixedCoord > 50.0f) ? 1.0f : -1.0f);
						}
						else
						{
							pz += noise * ((fixedCoord > 50.0f) ? 1.0f : -1.0f);
						}
						xyz.push_back(px);
						xyz.push_back(py);
						xyz.push_back(pz);
						normals.push_back(nx);
						normals.push_back(ny);
						normals.push_back(nz);
					}
				}
			};

			std::vector<float> xyz;
			std::vector<float> normals;
			addPlanarFace(xyz, normals, 100.0f, 2, 0.0f, 0.0f, 1.0f);
			addPlanarFace(xyz, normals, 0.0f, 2, 0.0f, 0.0f, -1.0f);
			addPlanarFace(xyz, normals, 100.0f, 0, 1.0f, 0.0f, 0.0f);
			addPlanarFace(xyz, normals, 0.0f, 0, -1.0f, 0.0f, 0.0f);
			addPlanarFace(xyz, normals, 100.0f, 1, 0.0f, 1.0f, 0.0f);
			addPlanarFace(xyz, normals, 0.0f, 1, 0.0f, -1.0f, 0.0f);

			TemplateBrepUpdateParams params;
			params.faceBandMm = 3.0;
			params.normalThresholdDeg = 35.0;
			params.minPointsPerFace = 30U;
			params.maxAllowedDeviationMm = 1.0;

			TemplateBrepUpdateResult result;
			std::string err;
			if (!updateShapeFromPointCloud(templateShape, xyz, normals, params, result, &err))
			{
				fail("templateBrepUpdate", err);
			}
			else if (result.updatedShape.isNull())
			{
				fail("templateBrepUpdate", "null updated shape");
			}
			else
			{
				TopoDS_Shape updatedNative;
				if (!ShapeHandleAccess::nativeShape(result.updatedShape, &updatedNative))
				{
					fail("templateBrepUpdate", "updated shape access failed");
				}
				else if (shapeFaceCount(updatedNative) != templateFaceCount)
				{
					fail("templateBrepUpdate", "face count changed after update");
				}
				else
				{
					for (int fi = 0; fi < templateFaceCount; ++fi)
					{
						if (!validateShapeFaceIndex(result.updatedShape, fi, &err))
						{
							fail("templateBrepUpdate", err);
							break;
						}
					}
				}
				if (!result.qualityPassed)
				{
					fail("templateBrepUpdate", "quality gate failed");
				}
				else if (result.globalMaxDeviationMm > params.maxAllowedDeviationMm)
				{
					fail("templateBrepUpdate", "max deviation above threshold");
				}
				else if (result.updatedFaceCount < 6U)
				{
					fail("templateBrepUpdate", "expected 6 updated faces");
				}
			}
		}
	}

	// TemplateBrepUpdate: BSpline 超阈值点约束调整控制点
	{
		TColgp_Array2OfPnt grid(1, 8, 1, 8);
		for (int i = 1; i <= 8; ++i)
		{
			for (int j = 1; j <= 8; ++j)
			{
				grid.SetValue(
					i,
					j,
					gp_Pnt(
						10.0 + static_cast<double>(i - 1) * 10.0,
						10.0 + static_cast<double>(j - 1) * 10.0,
						50.0));
			}
		}
		GeomAPI_PointsToBSplineSurface approx(
			grid,
			Approx_ChordLength,
			3,
			8,
			GeomAbs_C2,
			1.0);
		if (!approx.IsDone() || approx.Surface().IsNull())
		{
			fail("templateBrepBsplineAdjust", "failed to build template BSpline surface");
		}
		else
		{
			BRepBuilderAPI_MakeFace faceMaker(approx.Surface(), 1e-6);
			if (!faceMaker.IsDone())
			{
				fail("templateBrepBsplineAdjust", "failed to build BSpline face");
			}
			else
			{
				const TopoDS_Shape bsplineShape = faceMaker.Face();
				const ShapeHandle templateShape = ShapeHandleAccess::fromNativeShape(&bsplineShape);

				std::vector<float> xyz;
				std::vector<float> normals;
				for (int i = 0; i < 8; ++i)
				{
					for (int j = 0; j < 8; ++j)
					{
						const float x = 10.0f + static_cast<float>(i * 10);
						const float y = 10.0f + static_cast<float>(j * 10);
						const float zBump = (i + j > 8) ? 1.5f : 0.05f;
						xyz.push_back(x);
						xyz.push_back(y);
						xyz.push_back(50.0f + zBump);
						normals.push_back(0.0f);
						normals.push_back(0.0f);
						normals.push_back(1.0f);
					}
				}

				TemplateBrepUpdateParams params;
				params.faceBandMm = 5.0;
				params.normalThresholdDeg = 45.0;
				params.minPointsPerFace = 10U;
				params.maxAllowedDeviationMm = 0.5;
				params.selectedFaceIndices = {0};

				TemplateBrepUpdateResult result;
				std::string err;
				if (!updateShapeFromPointCloud(templateShape, xyz, normals, params, result, &err))
				{
					fail("templateBrepBsplineAdjust", err);
				}
				else if (result.perFace.empty())
				{
					fail("templateBrepBsplineAdjust", "empty perFace report");
				}
				else if (result.perFace[0].action != FaceUpdateAction::BSplineAdjusted)
				{
					fail("templateBrepBsplineAdjust", "expected BSplineAdjusted");
				}
				else if (result.perFace[0].maxDeviationMm > 0.8)
				{
					fail("templateBrepBsplineAdjust", "max deviation not reduced enough after pole adjust");
				}
				else if (!result.qualityPassed)
				{
					fail("templateBrepBsplineAdjust", "selective quality gate should pass");
				}
			}
		}
	}

	// TemplateBrepUpdate: 无点面保留原几何
	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		const ShapeHandle templateShape = ShapeHandleAccess::fromNativeShape(&box);

		auto addPlanarFace = [](
			std::vector<float>& xyz,
			std::vector<float>& normals,
			const float fixedCoord,
			const int fixedAxis,
			const float nx,
			const float ny,
			const float nz) {
			for (int i = 0; i < 10; ++i)
			{
				for (int j = 0; j < 10; ++j)
				{
					const float u = 10.0f + static_cast<float>(i * 8);
					const float v = 10.0f + static_cast<float>(j * 8);
					float px = 0.0f;
					float py = 0.0f;
					float pz = 0.0f;
					if (fixedAxis == 0)
					{
						px = fixedCoord;
						py = u;
						pz = v;
					}
					else if (fixedAxis == 1)
					{
						px = u;
						py = fixedCoord;
						pz = v;
					}
					else
					{
						px = u;
						py = v;
						pz = fixedCoord;
					}
					xyz.push_back(px);
					xyz.push_back(py);
					xyz.push_back(pz);
					normals.push_back(nx);
					normals.push_back(ny);
					normals.push_back(nz);
				}
			}
		};

		std::vector<float> xyz;
		std::vector<float> normals;
		addPlanarFace(xyz, normals, 100.0f, 2, 0.0f, 0.0f, 1.0f);
		addPlanarFace(xyz, normals, 100.0f, 0, 1.0f, 0.0f, 0.0f);
		addPlanarFace(xyz, normals, 0.0f, 0, -1.0f, 0.0f, 0.0f);
		addPlanarFace(xyz, normals, 100.0f, 1, 0.0f, 1.0f, 0.0f);
		addPlanarFace(xyz, normals, 0.0f, 1, 0.0f, -1.0f, 0.0f);

		TemplateBrepUpdateParams params;
		params.faceBandMm = 3.0;
		params.normalThresholdDeg = 35.0;
		params.minPointsPerFace = 30U;
		params.maxAllowedDeviationMm = 0.0;

		TemplateBrepUpdateResult result;
		std::string err;
		if (!updateShapeFromPointCloud(templateShape, xyz, normals, params, result, &err))
		{
			fail("templateBrepSkip", err);
		}
		else
		{
			bool foundSkipped = false;
			for (const FaceUpdateReport& report : result.perFace)
			{
				if (report.action == FaceUpdateAction::SkippedNoPoints)
				{
					foundSkipped = true;
					break;
				}
			}
			if (!foundSkipped)
			{
				fail("templateBrepSkip", "expected at least one skipped face");
			}
			else if (result.updatedShape.isNull())
			{
				fail("templateBrepSkip", "null updated shape");
			}
		}
	}

	// displaySoup 点云 + shape 变换一致性（配准输入路径）
	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		const ShapeHandle templateShape = ShapeHandleAccess::fromNativeShape(&box);

		std::vector<float> soupXyz;
		std::vector<float> soupNormals;
		std::size_t triCount = 0U;
		std::string extractErr;
		if (!extractDisplaySoupPointCloud(templateShape, soupXyz, soupNormals, 5000U, &triCount, &extractErr))
		{
			fail("reverseTemplateSoupIcp", extractErr.empty() ? "soup extract failed" : extractErr);
		}
		else if (soupXyz.size() < 900U || triCount < 10U)
		{
			fail("reverseTemplateSoupIcp", "display soup too sparse");
		}
		else
		{
			Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
			offset.translation() = Eigen::Vector3d(2.0, 1.0, 0.5);

			std::vector<float> shiftedSoup = soupXyz;
			applyIsometryInPlace(shiftedSoup, offset);

			ShapeHandle misaligned;
			std::string transformErr;
			if (!applyIsometryToShapeHandle(templateShape, offset, misaligned, &transformErr))
			{
				fail("reverseTemplateSoupIcp", transformErr.empty() ? "offset template failed" : transformErr);
			}
			else
			{
				std::vector<float> soupFromShape;
				std::vector<float> normsFromShape;
				if (!extractDisplaySoupPointCloud(misaligned, soupFromShape, normsFromShape, 5000U, nullptr, &extractErr))
				{
					fail("reverseTemplateSoupIcp", extractErr.empty() ? "re-extract failed" : extractErr);
				}
				else
				{
					const double maxDev = maxSampledPairDistanceMm(shiftedSoup, soupFromShape);
					if (maxDev > 0.5)
					{
						std::ostringstream oss;
						oss << "soup/shape transform mismatch maxDev=" << maxDev << "mm";
						fail("reverseTemplateSoupIcp", oss.str());
					}
				}
			}
		}
	}

	{
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(80.0, 60.0, 40.0).Shape();
		MeshDiscretizeParams discParams;
		discParams.quality = MeshQualityPreset::Medium;
		std::vector<float> boxSoup;
		MeshDiscretizeReport discReport;
		std::string discErr;
		if (!discretizeShapeToMesh(box, discParams, boxSoup, discReport, &discErr))
		{
			fail("meshSurfaceReconstruct", discErr.empty() ? "box tessellation failed" : discErr);
		}
		else
		{
			MeshSurfaceReconstructParams reconParams;
			reconParams.patchCountHint = 2;
			reconParams.samplesPerPatchEdge = 8;
			ShapeHandle brepShape;
			MeshSurfaceReconstructReport reconReport;
			std::string reconErr;
			if (!reconstructBrepFromMeshSoup(boxSoup, reconParams, brepShape, reconReport, &reconErr))
			{
				fail("meshSurfaceReconstruct", reconErr.empty() ? "reconstruct failed" : reconErr);
			}
			else if (brepShape.isNull())
			{
				fail("meshSurfaceReconstruct", "null output shape");
			}
			else if (reconReport.patchCount < 1)
			{
				fail("meshSurfaceReconstruct", "patchCount < 1");
			}
			else if (reconReport.nurbsPatchCount < 1 && reconReport.bsplinePatchCount < 1)
			{
				fail("meshSurfaceReconstruct", "no NURBS patches fitted");
			}

			MeshSurfaceReconstructParams hybridParams;
			hybridParams.partitionMode = MeshSurfacePartitionMode::HybridNormalCvt;
			hybridParams.hybridEnableRegionAdjust = true;
			auto hybridSession = createMeshSurfaceReconstructSession(boxSoup);
			std::string hybridErr;
			if (!runMeshSurfaceReconstructStage(
					*hybridSession,
					MeshSurfaceReconstructStage::Partition,
					hybridParams,
					nullptr,
					&hybridErr))
			{
				fail("meshSurfaceHybridPartition", hybridErr.empty() ? "hybrid partition failed" : hybridErr);
			}
			else if (hybridSession->report().patchCount < 4)
			{
				fail("meshSurfaceHybridPartition", "box hybrid patchCount < 4");
			}
			else if (hybridSession->report().initialRegionCount < 4)
			{
				fail("meshSurfaceHybridPartition", "box hybrid initialRegionCount < 4");
			}
			else if (hybridSession->report().quadPatchCount < 1)
			{
				fail("meshSurfaceHybridPartition", "box hybrid quadPatchCount < 1 after adjust");
			}

			MeshSurfaceReconstructParams cgalParams;
			cgalParams.partitionMode = MeshSurfacePartitionMode::CgalChartHybrid;
			cgalParams.harmonicBoundaryMode = MeshSurfaceHarmonicBoundaryMode::GeodesicSquare;
			cgalParams.enableMultiResolutionFit = true;
			auto cgalSession = createMeshSurfaceReconstructSession(boxSoup);
			std::string cgalErr;
			if (!runMeshSurfaceReconstructStage(
					*cgalSession,
					MeshSurfaceReconstructStage::Partition,
					cgalParams,
					nullptr,
					&cgalErr))
			{
				fail("meshSurfaceCgalChartPartition", cgalErr.empty() ? "cgal chart partition failed" : cgalErr);
			}
			else if (cgalSession->report().patchCount < 1)
			{
				fail("meshSurfaceCgalChartPartition", "cgal chart patchCount < 1");
			}

			const std::string goldenRoot = geoalgo::meshrecon::defaultAmrtoGoldenDataDirectory();
			const std::filesystem::path goldenObj = std::filesystem::path(goldenRoot) / "smooth_060.obj";
			if (std::filesystem::exists(goldenObj))
			{
				geoalgo::meshrecon::QuadMeshLite goldenQuad;
				std::string goldenLoadErr;
				if (!geoalgo::meshrecon::loadObjQuadMeshWithVt(goldenObj.string(), goldenQuad, &goldenLoadErr))
				{
					fail("meshSurfaceAmrtoGoldenPartition", goldenLoadErr.empty() ? "load smooth_060 failed" : goldenLoadErr);
				}
				else
				{
					geoalgo::meshrecon::IndexedMeshLite goldenTri;
					if (!geoalgo::meshrecon::triangulateQuadMeshToIndexed(goldenQuad, goldenTri, &goldenLoadErr))
					{
						fail("meshSurfaceAmrtoGoldenPartition", goldenLoadErr.empty() ? "triangulate failed" : goldenLoadErr);
					}
					else
					{
						std::vector<float> goldenSoup;
						goldenSoup.reserve(goldenTri.faces.size() * 3U);
						const int faceCount = static_cast<int>(goldenTri.faces.size() / 3U);
						for (int fi = 0; fi < faceCount; ++fi)
						{
							const std::size_t b = static_cast<std::size_t>(fi) * 3U;
							for (int k = 0; k < 3; ++k)
							{
								const int vi = goldenTri.faces[b + static_cast<std::size_t>(k)];
								const std::size_t vb = static_cast<std::size_t>(vi) * 3U;
								goldenSoup.push_back(goldenTri.vertices[vb]);
								goldenSoup.push_back(goldenTri.vertices[vb + 1U]);
								goldenSoup.push_back(goldenTri.vertices[vb + 2U]);
							}
						}
						MeshSurfaceReconstructParams amrtoGoldenParams;
						amrtoGoldenParams.partitionMode = MeshSurfacePartitionMode::AmrtoImGmcg;
						amrtoGoldenParams.gmcgBackend = MeshSurfaceGmcgBackend::GoldenLoader;
						amrtoGoldenParams.amrtoGoldenDataPath = goldenRoot;
						amrtoGoldenParams.samplesPerPatchEdge = 8;
						auto amrtoSession = createMeshSurfaceReconstructSession(goldenSoup);
						std::string amrtoErr;
						if (!runMeshSurfaceReconstructStage(
								*amrtoSession,
								MeshSurfaceReconstructStage::Partition,
								amrtoGoldenParams,
								nullptr,
								&amrtoErr))
						{
							fail("meshSurfaceAmrtoGoldenPartition", amrtoErr.empty() ? "golden partition failed" : amrtoErr);
						}
						else if (amrtoSession->report().patchCount < 140)
						{
							fail("meshSurfaceAmrtoGoldenPartition", "golden patchCount < 140");
						}
						else if (!runMeshSurfaceReconstructStage(
								*amrtoSession,
								MeshSurfaceReconstructStage::Sample,
								amrtoGoldenParams,
								nullptr,
								&amrtoErr))
						{
							fail("meshSurfaceAmrtoGoldenPartition", amrtoErr.empty() ? "golden sample failed" : amrtoErr);
						}
					}
				}
			}
		}

		{
			const std::filesystem::path data2Tri =
				std::filesystem::path(geoalgo::meshrecon::resolveCloudSimSdkRoot()) / "CODE_AMRTO" / "data_2"
				/ "meshlab_suitable_catmull.obj";
			const std::filesystem::path goldenQuad =
				std::filesystem::path(geoalgo::meshrecon::resolveCloudSimSdkRoot()) / "CODE_AMRTO" / "data_2"
				/ "Hole_quad_InstantMeshes_7.87k.obj";
			if (std::filesystem::exists(data2Tri) && std::filesystem::exists(goldenQuad))
			{
				geoalgo::meshrecon::IndexedMeshLite triMesh;
				std::string imErr;
				if (!loadTriObjIndexedMesh(data2Tri.string(), triMesh, &imErr))
				{
					fail("meshSurfaceImRemesh", imErr.empty() ? "load data_2 tri failed" : imErr);
				}
				else
				{
					geoalgo::meshrecon::QuadMeshLite quadOut;
					geoalgo::meshrecon::InstantMeshesParams imParams;
					imParams.deterministic = true;
					imParams.pureQuad = true;
					if (!geoalgo::meshrecon::remeshToQuadMesh(triMesh, quadOut, imParams, &imErr))
					{
						fail("meshSurfaceImRemesh", imErr.empty() ? "instant meshes remesh failed" : imErr);
					}
					else
					{
						geoalgo::meshrecon::QuadMeshLite goldenRef;
						if (!geoalgo::meshrecon::loadObjQuadMeshWithVt(goldenQuad.string(), goldenRef, &imErr))
						{
							fail("meshSurfaceImRemesh", imErr.empty() ? "load golden quad failed" : imErr);
						}
						else
						{
							const int outQuads = static_cast<int>(quadOut.quadFaces.size() / 4U);
							const int refQuads = static_cast<int>(goldenRef.quadFaces.size() / 4U);
							const int lo = static_cast<int>(static_cast<double>(refQuads) * 0.85);
							const int hi = static_cast<int>(static_cast<double>(refQuads) * 1.15);
							if (outQuads < lo || outQuads > hi)
							{
								fail(
									"meshSurfaceImRemesh",
									"quad count " + std::to_string(outQuads) + " outside ["
										+ std::to_string(lo) + "," + std::to_string(hi) + "] vs golden "
										+ std::to_string(refQuads));
							}
						}
					}
				}
			}
		}

		{
			const TopoDS_Shape onlineBox = BRepPrimAPI_MakeBox(80.0, 60.0, 40.0).Shape();
			MeshDiscretizeParams onlineDisc;
			onlineDisc.quality = MeshQualityPreset::Medium;
			std::vector<float> onlineSoup;
			MeshDiscretizeReport onlineDiscReport;
			std::string onlineDiscErr;
			if (!discretizeShapeToMesh(onlineBox, onlineDisc, onlineSoup, onlineDiscReport, &onlineDiscErr))
			{
				fail("meshSurfaceAmrtoOnlinePartition", onlineDiscErr.empty() ? "box tessellation failed" : onlineDiscErr);
			}
			else
			{
				MeshSurfaceReconstructParams onlineParams;
				onlineParams.partitionMode = MeshSurfacePartitionMode::AmrtoImGmcg;
				onlineParams.gmcgBackend = MeshSurfaceGmcgBackend::Native;
				onlineParams.amrtoFallbackGoldenOnImFailure = false;
				onlineParams.samplesPerPatchEdge = 8;
				auto onlineSession = createMeshSurfaceReconstructSession(onlineSoup);
				std::string onlineErr;
				if (!runMeshSurfaceReconstructStage(
						*onlineSession,
						MeshSurfaceReconstructStage::Partition,
						onlineParams,
						nullptr,
						&onlineErr))
				{
					fail(
						"meshSurfaceAmrtoOnlinePartition",
						onlineErr.empty() ? "online amrto partition failed" : onlineErr);
				}
				else if (onlineSession->report().patchCount < 2)
				{
					fail("meshSurfaceAmrtoOnlinePartition", "online patchCount < 2");
				}
				else
				{
					const int maxPatch = onlineSession->report().maxFacesPerPatch;
					const int totalFaces = static_cast<int>(onlineSoup.size() / 9U);
					if (totalFaces > 0 && maxPatch > static_cast<int>(totalFaces * 0.6))
					{
						fail("meshSurfaceAmrtoOnlinePartition", "largest patch > 60% faces");
					}
				}
			}
		}
	}

	{
		const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(15.0, 120.0).Shape();
		MeshDiscretizeParams cylDiscParams;
		cylDiscParams.quality = MeshQualityPreset::Medium;
		std::vector<float> cylSoup;
		MeshDiscretizeReport cylDiscReport;
		std::string cylDiscErr;
		if (!discretizeShapeToMesh(cylinder, cylDiscParams, cylSoup, cylDiscReport, &cylDiscErr))
		{
			fail("tubularGrinding", cylDiscErr.empty() ? "cylinder tessellation failed" : cylDiscErr);
		}
		else
		{
			auto tgSession = createTubularGrindingSession(cylSoup);
			TubularGrindingParams tgParams;
			tgParams.minSegmentFaces = 8.0;
			tgParams.regionGrowAxisAngleDeg = 35.0;
			tgParams.sectionSpacingMm = 4.0;
			std::string tgErr;
			if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Segment, tgParams, &tgErr))
			{
				fail("tubularGrindingSegment", tgErr.empty() ? "segment failed" : tgErr);
			}
			else if (tgSession->report().pipeCount < 1)
			{
				fail("tubularGrindingSegment", "pipeCount < 1");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Centerline, tgParams, &tgErr))
			{
				fail("tubularGrindingCenterline", tgErr.empty() ? "centerline failed" : tgErr);
			}
			else if (tgSession->report().centerlinePointCount < 4)
			{
				fail("tubularGrindingCenterline", "centerlinePointCount < 4");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::TemplatePoints, tgParams, &tgErr))
			{
				fail("tubularGrindingTemplate", tgErr.empty() ? "template failed" : tgErr);
			}
			else if (tgSession->report().templatePointCount < 8)
			{
				fail("tubularGrindingTemplate", "templatePointCount < 8");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Project, tgParams, &tgErr))
			{
				fail("tubularGrindingProject", tgErr.empty() ? "project failed" : tgErr);
			}
			else if (tgSession->report().projectedPointCount < 8)
			{
				fail("tubularGrindingProject", "projectedPointCount < 8");
			}
			else if (tgSession->report().projectionHitRate < 0.5)
			{
				fail("tubularGrindingProject", "projectionHitRate < 0.5");
			}
		std::vector<float> coloredSoup;
		std::vector<float> coloredRgb;
		if (!buildSegmentColoredMeshSoup(*tgSession, coloredSoup, coloredRgb, &tgErr))
		{
			fail("tubularGrindingColoredMesh", tgErr.empty() ? "colored mesh failed" : tgErr);
		}
	}

	// 测试自适应邻域模式（广义管状分析）
	{
		const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(15.0, 120.0).Shape();
		MeshDiscretizeParams cylDiscParams;
		cylDiscParams.quality = MeshQualityPreset::Medium;
		std::vector<float> cylSoup;
		MeshDiscretizeReport cylDiscReport;
		std::string cylDiscErr;
		if (discretizeShapeToMesh(cylinder, cylDiscParams, cylSoup, cylDiscReport, &cylDiscErr))
		{
			auto tgSession = createTubularGrindingSession(cylSoup);
			TubularGrindingParams tgParams;
			tgParams.minSegmentFaces = 8.0;
			tgParams.sectionSpacingMm = 4.0;
			// 使用自适应模式 + 椭圆拟合
			tgParams.neighborhoodMode = NeighborhoodMode::Adaptive;
			tgParams.sectionFitMode = SectionFitMode::Ellipse;
			tgParams.centerlineIterations = 2;
			std::string tgErr;
			if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Segment, tgParams, &tgErr))
			{
				fail("tubularGrindingAdaptiveSegment", tgErr.empty() ? "adaptive segment failed" : tgErr);
			}
			else if (tgSession->report().pipeCount < 1)
			{
				fail("tubularGrindingAdaptiveSegment", "adaptive pipeCount < 1");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Centerline, tgParams, &tgErr))
			{
				fail("tubularGrindingAdaptiveCenterline", tgErr.empty() ? "adaptive centerline failed" : tgErr);
			}
			else if (tgSession->report().centerlinePointCount < 4)
			{
				fail("tubularGrindingAdaptiveCenterline", "adaptive centerlinePointCount < 4");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::TemplatePoints, tgParams, &tgErr))
			{
				fail("tubularGrindingAdaptiveTemplate", tgErr.empty() ? "adaptive template failed" : tgErr);
			}
			else if (tgSession->report().templatePointCount < 8)
			{
				fail("tubularGrindingAdaptiveTemplate", "adaptive templatePointCount < 8");
			}
			else if (!runTubularGrindingStage(*tgSession, TubularGrindingStage::Project, tgParams, &tgErr))
			{
				fail("tubularGrindingAdaptiveProject", tgErr.empty() ? "adaptive project failed" : tgErr);
			}
			else if (tgSession->report().projectedPointCount < 8)
			{
				fail("tubularGrindingAdaptiveProject", "adaptive projectedPointCount < 8");
			}
			else if (tgSession->report().projectionHitRate < 0.5)
			{
				fail("tubularGrindingAdaptiveProject", "adaptive projectionHitRate < 0.5");
			}
		}
	}
}

	{
		TColgp_Array2OfPnt grid(1, 5, 1, 5);
		for (int iu = 1; iu <= 5; ++iu)
		{
			for (int iv = 1; iv <= 5; ++iv)
			{
				const double u = static_cast<double>(iu - 1);
				const double v = static_cast<double>(iv - 1);
				grid.SetValue(iu, iv, gp_Pnt(u, v, 0.1 * u * v));
			}
		}
		Handle(Geom_BSplineSurface) surface;
		if (!meshrecon::fitNurbsSurfaceFromGrid(
				grid,
				6,
				6,
				meshrecon::NurbsFitMode::ApproxCentripetalFixedCtrlpts,
				3,
				3,
				surface)
			|| surface.IsNull())
		{
			fail("nurbsSurfaceFitting", "fitNurbsSurfaceFromGrid failed");
		}
		else if (surface->NbUPoles() < 4 || surface->NbVPoles() < 4)
		{
			fail("nurbsSurfaceFitting", "too few control points");
		}
	}

	{
		// 单位立方体 z=0.5 平面截面
		std::vector<float> boxSoup = {
			0, 0, 0, 1, 0, 0, 1, 1, 0,
			0, 0, 0, 1, 1, 0, 0, 1, 0,
			0, 0, 1, 1, 0, 1, 1, 1, 1,
			0, 0, 1, 1, 1, 1, 0, 1, 1};
		const double origin[3] = {0.5, 0.5, 0.5};
		const double normal[3] = {0, 0, 1};
		std::vector<MeshTrajectoryPolyline> polylines;
		std::string err;
		if (!intersectPlaneWithTriangleSoup(boxSoup, origin, normal, nullptr, polylines, &err)
			|| polylines.empty())
		{
			fail("meshTrajectoryCrossSection", err.empty() ? "no intersection" : err);
		}
		else
		{
			MeshTrajectorySpec spec;
			spec.workpiece.backendIdUtf8 = "test";
			spec.method = MeshTrajectoryMethod::CrossSection;
			spec.crossSection.planeOriginMm[0] = 0.5;
			spec.crossSection.planeOriginMm[1] = 0.5;
			spec.crossSection.planeOriginMm[2] = 0.5;
			spec.crossSection.planeNormal[0] = 0;
			spec.crossSection.planeNormal[1] = 0;
			spec.crossSection.planeNormal[2] = 1;
			spec.discretize.stepMm = 0.25;
			RawPath path;
			if (!generateMeshTrajectory(spec, boxSoup, path, &err) || path.points.size() < 4U)
			{
				fail("meshTrajectoryGenerate", err.empty() ? "too few points" : err);
			}
		}
	}

	{
		std::vector<float> fanSoup;
		std::vector<int> tris;
		for (int i = 0; i < 6; ++i)
		{
			const double a0 = 3.14159265358979323846 * static_cast<double>(i) / 6.0;
			const double a1 = 3.14159265358979323846 * static_cast<double>(i + 1) / 6.0;
			const float r = 30.f;
			const float x0 = static_cast<float>(std::cos(a0) * r);
			const float y0 = static_cast<float>(std::sin(a0) * r);
			const float x1 = static_cast<float>(std::cos(a1) * r);
			const float y1 = static_cast<float>(std::sin(a1) * r);
			const int ti = static_cast<int>(fanSoup.size() / 9U);
			tris.push_back(ti);
			fanSoup.insert(fanSoup.end(), {0.f, 0.f, 0.f, x0, y0, 0.f, x1, y1, 0.f});
		}
		MeshTrajectorySpec bspec;
		bspec.workpiece.backendIdUtf8 = "test";
		bspec.method = MeshTrajectoryMethod::BsplineRegion;
		bspec.region.triangleIndices = tris;
		bspec.bspline.uvCountU = 6;
		bspec.bspline.uvCountV = 6;
		bspec.bspline.traceMode = MeshTrajectoryUvTraceMode::UvGrid;
		std::string err;
		RawPath bpath;
		if (!generateMeshTrajectory(bspec, fanSoup, bpath, &err) || bpath.points.size() != 36U)
		{
			fail("meshTrajectoryBsplineGrid", err.empty() ? "grid point count" : err);
		}
		bspec.bspline.traceMode = MeshTrajectoryUvTraceMode::USerpentine;
		RawPath serpPath;
		if (!generateMeshTrajectory(bspec, fanSoup, serpPath, &err) || serpPath.points.size() != 36U)
		{
			fail("meshTrajectoryBsplineUSerpentine", err.empty() ? "serpentine count" : err);
		}
	}

	{
		ensureFeatureDiscretizersRegistered();
		const std::vector<std::string> ids = featureDiscretizerListStrategyIds();
		if (ids.empty())
		{
			fail("featureDiscretizerRegistry", "no strategies registered");
		}
		const TopoDS_Shape box = BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape();
		FeatureListDocument doc;
		doc.workpiece.stepPathUtf8 = "selftest_box";
		FeatureEntry edgeEntry;
		edgeEntry.featureId = "edge_0";
		edgeEntry.strategyId = "EdgeChain";
		edgeEntry.geometry.edgeIndices = {1, 2};
		edgeEntry.params["stepMm"] = 5.0;
		edgeEntry.params["linearDeflectionMm"] = 0.1;
		doc.features.push_back(edgeEntry);
		RawPath path;
		std::string err;
		if (!discretizeFeatureList(doc, ShapeHandleAccess::fromNativeShape(&box), path, &err) || path.points.size() < 2U)
		{
			fail("featureDiscretizeEdgeChain", err.empty() ? "too few points" : err);
		}
	}

	return failures.empty();
}

} // namespace geoalgo
