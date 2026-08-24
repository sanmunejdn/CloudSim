/// @file ShapeIo.cpp
/// @brief ShapeIo 实现

#include "ShapeIo.h"

#include "CsgkBackend.h"
#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <gp_Trsf.hxx>

#include <filesystem>
#include <fstream>
#include <string>

namespace geoalgo
{
namespace
{
bool tryOpenInput(const std::filesystem::path& path, std::ifstream& outStream)
{
	outStream.close();
	outStream.clear();
	outStream.open(path, std::ios::in | std::ios::binary);
	return static_cast<bool>(outStream);
}

bool tryOpenOutput(const std::filesystem::path& path, std::ofstream& outStream)
{
	outStream.close();
	outStream.clear();
	outStream.open(path, std::ios::out | std::ios::binary | std::ios::trunc);
	return static_cast<bool>(outStream);
}

/// OCCT 窄路径 API 不吃中文；用 filesystem 宽路径开流再走 Stream 接口
bool openInputBinary(const std::string& pathBytes, std::ifstream& outStream, std::string* errMsg)
{
	std::filesystem::path utf8Path;
	bool haveUtf8 = false;
	try
	{
		utf8Path = std::filesystem::u8path(pathBytes);
		haveUtf8 = true;
	}
	catch (...)
	{
	}
	const std::filesystem::path nativePath(pathBytes);

	if (haveUtf8 && tryOpenInput(utf8Path, outStream))
	{
		return true;
	}
	if (tryOpenInput(nativePath, outStream))
	{
		return true;
	}
	if (errMsg)
	{
		*errMsg = "cannot open STEP/BREP file (path encoding?)";
	}
	return false;
}

bool openOutputBinary(const std::string& pathBytes, std::ofstream& outStream, std::string* errMsg)
{
	std::filesystem::path utf8Path;
	bool haveUtf8 = false;
	try
	{
		utf8Path = std::filesystem::u8path(pathBytes);
		haveUtf8 = true;
	}
	catch (...)
	{
	}
	const std::filesystem::path nativePath(pathBytes);

	if (haveUtf8 && tryOpenOutput(utf8Path, outStream))
	{
		return true;
	}
	if (tryOpenOutput(nativePath, outStream))
	{
		return true;
	}
	if (errMsg)
	{
		*errMsg = "cannot create STEP/BREP file (path encoding?)";
	}
	return false;
}

std::string streamLabel(const std::string& pathBytes)
{
	try
	{
		std::filesystem::path p;
		try
		{
			p = std::filesystem::u8path(pathBytes);
		}
		catch (...)
		{
			p = std::filesystem::path(pathBytes);
		}
		const auto name = p.filename().u8string();
		return name.empty() ? std::string("model") : std::string(name.begin(), name.end());
	}
	catch (...)
	{
		return "model";
	}
}
} // namespace

bool readStepShape(const std::string& pathLocal, TopoDS_Shape& outShape, std::string* errMsg)
{
	std::ifstream in;
	if (!openInputBinary(pathLocal, in, errMsg))
	{
		return false;
	}

	STEPControl_Reader reader;
	const std::string label = streamLabel(pathLocal);
	const IFSelect_ReturnStatus status = reader.ReadStream(label.c_str(), in);
	if (status != IFSelect_RetDone)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP read failed";
		}
		return false;
	}
	if (!reader.TransferRoots())
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP transfer failed";
		}
		return false;
	}
	outShape = reader.OneShape();
	if (outShape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP produced empty shape";
		}
		return false;
	}
	return true;
}

bool readStepIntoHandle(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&shape);
	return !outShape.isNull();
}

bool readBrepFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg)
{
	outShape = ShapeHandle{};
#ifdef CLOUDSIM_USE_CSGK
	if(pathLocal.size() >= 5 && pathLocal.compare(pathLocal.size() - 5, 5, ".csgb") == 0)
		return readCsgkNativeFile(pathLocal, outShape, errMsg);
#endif
	std::ifstream in;
	if (!openInputBinary(pathLocal, in, errMsg))
	{
		return false;
	}
	TopoDS_Shape shape;
	BRep_Builder builder;
	BRepTools::Read(shape, in, builder);
	if (shape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "OCCT BREP read failed";
		}
		return false;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&shape);
	return true;
}

bool writeBrepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg)
{
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	std::ofstream out;
	if (!openOutputBinary(pathLocal, out, errMsg))
	{
		return false;
	}
	BRepTools::Write(native, out);
	if (!out)
	{
		if (errMsg)
		{
			*errMsg = "OCCT BREP write failed";
		}
		return false;
	}
	return true;
}

bool writeStepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg)
{
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	STEPControl_Writer writer;
	const IFSelect_ReturnStatus transferStatus = writer.Transfer(native, STEPControl_AsIs);
	if (transferStatus != IFSelect_RetDone)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP transfer failed";
		}
		return false;
	}
	std::ofstream out;
	if (!openOutputBinary(pathLocal, out, errMsg))
	{
		return false;
	}
	const IFSelect_ReturnStatus writeStatus = writer.WriteStream(out);
	if (writeStatus != IFSelect_RetDone || !out)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP write failed";
		}
		return false;
	}
	return true;
}

ShapeHandle transformShape(const ShapeHandle& shape, const Eigen::Isometry3d& iso)
{
	if (shape.isNull())
	{
		return {};
	}

	// 构建 OCCT gp_Trsf
	gp_Trsf trsf;
	trsf.SetValues(iso(0, 0), iso(0, 1), iso(0, 2), iso(0, 3), iso(1, 0), iso(1, 1), iso(1, 2), iso(1, 3), iso(2, 0),
				   iso(2, 1), iso(2, 2), iso(2, 3));

	// 获取原始 shape 并应用变换
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		return {};
	}

	BRepBuilderAPI_Transform transformer(native, trsf, true);
	return ShapeHandleAccess::fromNativeShape(&transformer.Shape());
}

} // namespace geoalgo
