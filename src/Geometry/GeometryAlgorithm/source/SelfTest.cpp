#include "detail/OccIncludes.h"

#include "BrepBoolean.h"
#include "Discretize.h"
#include "Intersection.h"
#include "GeoMeshBoolean.h"
#include "MeshDiscretize.h"
#include "SelfTest.h"
#include "ShapeIo.h"
#include "WireOps.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <sstream>

namespace geoalgo
{

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

	return failures.empty();
}

} // namespace geoalgo
