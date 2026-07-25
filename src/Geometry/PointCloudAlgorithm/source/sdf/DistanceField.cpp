#include "sdf/DistanceField.h"

#include "Measure.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pclalgo
{
namespace sdf
{
namespace
{

double autoVoxelMm(const std::vector<float>& xyz, double requested)
{
	if (requested > 0.0)
	{
		return requested;
	}
	const double spacing = computeAverageSpacingMm(xyz, 6U);
	if (spacing > 1e-9)
	{
		return spacing;
	}
	return 1.0;
}

} // namespace

bool DistanceField::buildFromPointCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
										double fieldVoxelMm, std::string* errMsg)
{
	if (xyz.size() < 9U || xyz.size() % 3U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: invalid target xyz";
		}
		return false;
	}
	if (normals.size() != xyz.size())
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: normals size mismatch";
		}
		return false;
	}
	xyz_ = xyz;
	normals_ = normals;
	tree_ = std::make_unique<KdTreePointSet>(xyz_);
	if (tree_->empty())
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: empty KdTree";
		}
		return false;
	}
	fieldVoxelMmUsed_ = autoVoxelMm(xyz_, fieldVoxelMm);
	rebuildVoxelCache();
	return true;
}

bool DistanceField::buildFromMeshSoup(const std::vector<float>& soup, double fieldVoxelMm, std::string* errMsg)
{
	if (soup.size() < 9U || soup.size() % 9U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: invalid mesh soup";
		}
		return false;
	}
	std::vector<float> xyz;
	std::vector<float> normals;
	xyz.reserve(soup.size() / 3U);
	normals.reserve(soup.size() / 3U);
	const std::size_t triCount = soup.size() / 9U;
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const float* p = soup.data() + t * 9U;
		Eigen::Vector3d a(p[0], p[1], p[2]);
		Eigen::Vector3d b(p[3], p[4], p[5]);
		Eigen::Vector3d c(p[6], p[7], p[8]);
		Eigen::Vector3d n = (b - a).cross(c - a);
		const double nlen = n.norm();
		if (nlen < 1e-12)
		{
			continue;
		}
		n /= nlen;
		const Eigen::Vector3d verts[3] = {a, b, c};
		for (const Eigen::Vector3d& v : verts)
		{
			xyz.push_back(static_cast<float>(v.x()));
			xyz.push_back(static_cast<float>(v.y()));
			xyz.push_back(static_cast<float>(v.z()));
			normals.push_back(static_cast<float>(n.x()));
			normals.push_back(static_cast<float>(n.y()));
			normals.push_back(static_cast<float>(n.z()));
		}
	}
	return buildFromPointCloud(xyz, normals, fieldVoxelMm, errMsg);
}

FieldSample DistanceField::queryExact(const Eigen::Vector3d& x) const
{
	FieldSample s;
	if (!tree_ || tree_->empty())
	{
		return s;
	}
	std::size_t idx = 0;
	double dist2 = 0.0;
	idx = tree_->findNearest(x.x(), x.y(), x.z(), std::numeric_limits<double>::max(), dist2);
	if (idx == static_cast<std::size_t>(-1))
	{
		return s;
	}
	const std::size_t b = idx * 3U;
	s.closest = Eigen::Vector3d(xyz_[b], xyz_[b + 1U], xyz_[b + 2U]);
	s.normal = Eigen::Vector3d(normals_[b], normals_[b + 1U], normals_[b + 2U]);
	const double nlen = s.normal.norm();
	if (nlen > 1e-12)
	{
		s.normal /= nlen;
	}
	else
	{
		s.normal = Eigen::Vector3d(0, 0, 1);
	}
	const Eigen::Vector3d delta = x - s.closest;
	s.signedDistance = delta.dot(s.normal);
	s.directed = s.signedDistance * s.normal;
	s.valid = true;
	return s;
}

void DistanceField::rebuildVoxelCache()
{
	useVoxel_ = false;
	voxelPhi_.clear();
	voxelDx_.clear();
	voxelDy_.clear();
	voxelDz_.clear();
	voxelNx_.clear();
	voxelNy_.clear();
	voxelNz_.clear();
	if (xyz_.empty() || fieldVoxelMmUsed_ <= 0.0)
	{
		return;
	}

	Eigen::AlignedBox3d box;
	for (std::size_t i = 0; i + 2U < xyz_.size(); i += 3U)
	{
		box.extend(Eigen::Vector3d(xyz_[i], xyz_[i + 1U], xyz_[i + 2U]));
	}
	const double pad = fieldVoxelMmUsed_ * 3.0;
	box.min() -= Eigen::Vector3d::Constant(pad);
	box.max() += Eigen::Vector3d::Constant(pad);

	const Eigen::Vector3d ext = box.sizes();
	int nx = std::max(2, static_cast<int>(std::ceil(ext.x() / fieldVoxelMmUsed_)) + 1);
	int ny = std::max(2, static_cast<int>(std::ceil(ext.y() / fieldVoxelMmUsed_)) + 1);
	int nz = std::max(2, static_cast<int>(std::ceil(ext.z() / fieldVoxelMmUsed_)) + 1);
	// 限制体素数，避免超大点云 OOM
	constexpr int kMaxDim = 96;
	nx = std::min(nx, kMaxDim);
	ny = std::min(ny, kMaxDim);
	nz = std::min(nz, kMaxDim);
	dims_ = Eigen::Vector3i(nx, ny, nz);
	origin_ = box.min();
	voxelStep_ = Eigen::Vector3d(ext.x() / std::max(1, nx - 1), ext.y() / std::max(1, ny - 1),
								 ext.z() / std::max(1, nz - 1));

	const int nVox = nx * ny * nz;
	voxelPhi_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelDx_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelDy_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelDz_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelNx_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelNy_.assign(static_cast<std::size_t>(nVox), 0.f);
	voxelNz_.assign(static_cast<std::size_t>(nVox), 0.f);

	for (int iz = 0; iz < nz; ++iz)
	{
		for (int iy = 0; iy < ny; ++iy)
		{
			for (int ix = 0; ix < nx; ++ix)
			{
				const Eigen::Vector3d p =
					origin_ + Eigen::Vector3d(ix * voxelStep_.x(), iy * voxelStep_.y(), iz * voxelStep_.z());
				const FieldSample s = queryExact(p);
				const int id = ix + nx * (iy + ny * iz);
				if (!s.valid)
				{
					continue;
				}
				voxelPhi_[static_cast<std::size_t>(id)] = static_cast<float>(s.signedDistance);
				voxelDx_[static_cast<std::size_t>(id)] = static_cast<float>(s.directed.x());
				voxelDy_[static_cast<std::size_t>(id)] = static_cast<float>(s.directed.y());
				voxelDz_[static_cast<std::size_t>(id)] = static_cast<float>(s.directed.z());
				voxelNx_[static_cast<std::size_t>(id)] = static_cast<float>(s.normal.x());
				voxelNy_[static_cast<std::size_t>(id)] = static_cast<float>(s.normal.y());
				voxelNz_[static_cast<std::size_t>(id)] = static_cast<float>(s.normal.z());
			}
		}
	}
	useVoxel_ = true;
}

FieldSample DistanceField::queryVoxel(const Eigen::Vector3d& x) const
{
	FieldSample s;
	if (!useVoxel_ || dims_.x() < 2)
	{
		return queryExact(x);
	}
	const Eigen::Vector3d local = x - origin_;
	const double fx = local.x() / std::max(1e-12, voxelStep_.x());
	const double fy = local.y() / std::max(1e-12, voxelStep_.y());
	const double fz = local.z() / std::max(1e-12, voxelStep_.z());
	if (fx < 0.0 || fy < 0.0 || fz < 0.0 || fx > dims_.x() - 1 || fy > dims_.y() - 1 || fz > dims_.z() - 1)
	{
		return queryExact(x);
	}
	const int x0 = static_cast<int>(std::floor(fx));
	const int y0 = static_cast<int>(std::floor(fy));
	const int z0 = static_cast<int>(std::floor(fz));
	const int x1 = std::min(x0 + 1, dims_.x() - 1);
	const int y1 = std::min(y0 + 1, dims_.y() - 1);
	const int z1 = std::min(z0 + 1, dims_.z() - 1);
	const double tx = fx - x0;
	const double ty = fy - y0;
	const double tz = fz - z0;

	auto at = [&](int ix, int iy, int iz) -> int { return ix + dims_.x() * (iy + dims_.y() * iz); };
	auto lerp = [](double a, double b, double t) { return a * (1.0 - t) + b * t; };
	auto sampleComp = [&](const std::vector<float>& buf) {
		const double c000 = buf[static_cast<std::size_t>(at(x0, y0, z0))];
		const double c100 = buf[static_cast<std::size_t>(at(x1, y0, z0))];
		const double c010 = buf[static_cast<std::size_t>(at(x0, y1, z0))];
		const double c110 = buf[static_cast<std::size_t>(at(x1, y1, z0))];
		const double c001 = buf[static_cast<std::size_t>(at(x0, y0, z1))];
		const double c101 = buf[static_cast<std::size_t>(at(x1, y0, z1))];
		const double c011 = buf[static_cast<std::size_t>(at(x0, y1, z1))];
		const double c111 = buf[static_cast<std::size_t>(at(x1, y1, z1))];
		const double c00 = lerp(c000, c100, tx);
		const double c10 = lerp(c010, c110, tx);
		const double c01 = lerp(c001, c101, tx);
		const double c11 = lerp(c011, c111, tx);
		const double c0 = lerp(c00, c10, ty);
		const double c1 = lerp(c01, c11, ty);
		return lerp(c0, c1, tz);
	};

	s.signedDistance = sampleComp(voxelPhi_);
	s.directed = Eigen::Vector3d(sampleComp(voxelDx_), sampleComp(voxelDy_), sampleComp(voxelDz_));
	s.normal = Eigen::Vector3d(sampleComp(voxelNx_), sampleComp(voxelNy_), sampleComp(voxelNz_));
	const double nlen = s.normal.norm();
	if (nlen > 1e-12)
	{
		s.normal /= nlen;
	}
	s.closest = x - s.directed;
	s.valid = true;
	return s;
}

FieldSample DistanceField::query(const Eigen::Vector3d& x) const
{
	if (useVoxel_)
	{
		return queryVoxel(x);
	}
	return queryExact(x);
}

} // namespace sdf
} // namespace pclalgo
