#include "detail/OccIncludes.h"

#include "BrepBoolean.h"
#include "Discretize.h"
#include "Intersection.h"
#include "GeoMeshBoolean.h"
#include "MeshDiscretize.h"
#include "SelfTest.h"
#include "ShapeQuery.h"
#include "TemplateBrepUpdate.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ViewTessellate.h"
#include "WireOps.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace geoalgo
{
namespace
{

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

	return failures.empty();
}

} // namespace geoalgo
