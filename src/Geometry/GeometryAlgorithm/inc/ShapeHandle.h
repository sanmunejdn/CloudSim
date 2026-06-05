#pragma once

#include "geometry_algorithm_global.h"

#include <memory>

namespace geoalgo
{

/// 不透明 B-rep 共享句柄；Data/Visual 侧不暴露 OCCT 类型
class GEOMETRY_ALGORITHM_API ShapeHandle
{
public:
	ShapeHandle();
	ShapeHandle(const ShapeHandle& other);
	ShapeHandle(ShapeHandle&& other) noexcept;
	ShapeHandle& operator=(const ShapeHandle& other);
	ShapeHandle& operator=(ShapeHandle&& other) noexcept;
	~ShapeHandle();

	bool isNull() const;
	bool isSame(const ShapeHandle& other) const;

	/// 深拷贝 TopoDS，供会修改 triangulation 的路径使用
	ShapeHandle clone() const;

	struct BoundsMm
	{
		double minX = 0.0;
		double minY = 0.0;
		double minZ = 0.0;
		double maxX = 0.0;
		double maxY = 0.0;
		double maxZ = 0.0;
		bool valid = false;
	};
	BoundsMm boundingBoxMm() const;

private:
	friend class ShapeHandleAccess;
	struct Impl;
	std::shared_ptr<Impl> m_impl;
};

/// GeometryAlgorithm 内部访问 TopoDS_Shape
class GEOMETRY_ALGORITHM_API ShapeHandleAccess
{
public:
	static bool nativeShape(const ShapeHandle& handle, void* outTopoDsShapeStorage);
	static ShapeHandle fromNativeShape(const void* topoDsShapeStorage);
};

} // namespace geoalgo
