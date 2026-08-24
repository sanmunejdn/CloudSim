/// @file ShapeHandle.cpp
/// @brief ShapeHandle 实现

#include "ShapeHandle.h"

#include "detail/OccIncludes.h"

#ifdef CLOUDSIM_USE_CSGK
#include <csgk/body_handle.h>
#endif

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <Bnd_Box.hxx>

namespace geoalgo
{
struct ShapeHandle::Impl
{
	TopoDS_Shape shape;
#ifdef CLOUDSIM_USE_CSGK
	bool csgkBackend = false;
	csgk::BodyHandle csgkBody;
#endif
};

ShapeHandle::ShapeHandle() = default;

ShapeHandle::ShapeHandle(const ShapeHandle& other) : m_impl(other.m_impl) {}

ShapeHandle::ShapeHandle(ShapeHandle&& other) noexcept : m_impl(std::move(other.m_impl)) {}

ShapeHandle& ShapeHandle::operator=(const ShapeHandle& other)
{
	m_impl = other.m_impl;
	return *this;
}

ShapeHandle& ShapeHandle::operator=(ShapeHandle&& other) noexcept
{
	m_impl = std::move(other.m_impl);
	return *this;
}

ShapeHandle::~ShapeHandle() = default;

bool ShapeHandle::isNull() const
{
	if(!m_impl)
		return true;
#ifdef CLOUDSIM_USE_CSGK
	if(m_impl->csgkBackend)
		return m_impl->csgkBody.isNull();
#endif
	return m_impl->shape.IsNull();
}

bool ShapeHandle::isSame(const ShapeHandle& other) const
{
	if (isNull() || other.isNull())
	{
		return false;
	}
	return m_impl.get() == other.m_impl.get();
}

ShapeHandle ShapeHandle::clone() const
{
	ShapeHandle out;
	if(isNull())
		return out;
#ifdef CLOUDSIM_USE_CSGK
	if(m_impl->csgkBackend)
	{
		out.m_impl = std::make_shared<Impl>();
		out.m_impl->csgkBackend = true;
		out.m_impl->csgkBody = m_impl->csgkBody.clone();
		return out;
	}
#endif
	BRepBuilderAPI_Copy copier(m_impl->shape);
	out.m_impl = std::make_shared<Impl>();
	out.m_impl->shape = copier.Shape();
	return out;
}

ShapeHandle::BoundsMm ShapeHandle::boundingBoxMm() const
{
	BoundsMm b;
	if(isNull())
		return b;
#ifdef CLOUDSIM_USE_CSGK
	if(m_impl->csgkBackend)
	{
		const csgk::Bounds3d box = m_impl->csgkBody.boundingBoxMm();
		if(!box.valid)
			return b;
		b.minX = box.minX;
		b.minY = box.minY;
		b.minZ = box.minZ;
		b.maxX = box.maxX;
		b.maxY = box.maxY;
		b.maxZ = box.maxZ;
		b.valid = true;
		return b;
	}
#endif
	Bnd_Box box;
	BRepBndLib::Add(m_impl->shape, box);
	if (box.IsVoid())
	{
		return b;
	}
	Standard_Real xmin = 0.0;
	Standard_Real ymin = 0.0;
	Standard_Real zmin = 0.0;
	Standard_Real xmax = 0.0;
	Standard_Real ymax = 0.0;
	Standard_Real zmax = 0.0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	b.minX = xmin;
	b.minY = ymin;
	b.minZ = zmin;
	b.maxX = xmax;
	b.maxY = ymax;
	b.maxZ = zmax;
	b.valid = true;
	return b;
}

bool ShapeHandleAccess::nativeShape(const ShapeHandle& handle, void* outTopoDsShapeStorage)
{
	if(!outTopoDsShapeStorage || handle.isNull())
		return false;
#ifdef CLOUDSIM_USE_CSGK
	if(handle.m_impl && handle.m_impl->csgkBackend)
		return false;
#endif
	auto* out = static_cast<TopoDS_Shape*>(outTopoDsShapeStorage);
	*out = handle.m_impl->shape;
	return true;
}

ShapeHandle ShapeHandleAccess::fromNativeShape(const void* topoDsShapeStorage)
{
	ShapeHandle out;
	if (!topoDsShapeStorage)
	{
		return out;
	}
	const auto* in = static_cast<const TopoDS_Shape*>(topoDsShapeStorage);
	if (in->IsNull())
	{
		return out;
	}
	out.m_impl = std::make_shared<ShapeHandle::Impl>();
	out.m_impl->shape = *in;
	return out;
}

#ifdef CLOUDSIM_USE_CSGK
ShapeHandle ShapeHandleAccess::fromCsgkBody(const csgk::BodyHandle& body)
{
	ShapeHandle out;
	if(body.isNull())
		return out;
	out.m_impl = std::make_shared<ShapeHandle::Impl>();
	out.m_impl->csgkBackend = true;
	out.m_impl->csgkBody = body;
	return out;
}

bool ShapeHandleAccess::tryGetCsgkBody(const ShapeHandle& handle, csgk::BodyHandle& outBody)
{
	if(handle.isNull() || !handle.m_impl || !handle.m_impl->csgkBackend)
		return false;
	outBody = handle.m_impl->csgkBody;
	return !outBody.isNull();
}

bool ShapeHandleAccess::isCsgkBackend(const ShapeHandle& handle)
{
	return !handle.isNull() && handle.m_impl && handle.m_impl->csgkBackend;
}
#endif

} // namespace geoalgo
