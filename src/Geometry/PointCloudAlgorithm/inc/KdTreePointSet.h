#ifndef POINTCLOUDALGORITHM_KDTREEPOINTSET_H
#define POINTCLOUDALGORITHM_KDTREEPOINTSET_H

/// @file KdTreePointSet.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief KdTreePointSet 接口

// 基于 CGAL Kd_tree 的点云空间索引加速
// 用于 ICP 最近邻搜索和 FPFH 特征匹配
// 性能优化：使用索引映射避免线性查找

#include <limits>
#include <map>
#include <memory>
#include <vector>

#include <CGAL/Kd_tree.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Simple_cartesian.h>

namespace pclalgo
{
// 点云适配器：将 std::vector<float> 的 3D 点云适配为 CGAL Kd_tree 可用的点集
class KdTreePointSet
{
public:
	using Kernel = CGAL::Simple_cartesian<double>;
	using Point_3 = Kernel::Point_3;
	using Traits = CGAL::Search_traits_3<Kernel>;
	using K_neighbor_search = CGAL::Orthogonal_k_neighbor_search<Traits>;
	using Tree = CGAL::Kd_tree<Traits>;

	KdTreePointSet() = default;

	// 从 xyz 缓冲构建 KD-tree
	explicit KdTreePointSet(const std::vector<float>& xyz) { build(xyz); }

	// 从 xyz 缓冲和索引子集构建 KD-tree
	KdTreePointSet(const std::vector<float>& xyz, const std::vector<std::size_t>& indices) { build(xyz, indices); }

	// 构建 KD-tree（使用所有点）
	void build(const std::vector<float>& xyz)
	{
		const std::size_t n = xyz.size() / 3U;
		std::vector<std::size_t> indices(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			indices[i] = i;
		}
		build(xyz, indices);
	}

	// 构建 KD-tree（使用指定索引子集）
	void build(const std::vector<float>& xyz, const std::vector<std::size_t>& indices)
	{
		points_.clear();
		indexMap_.clear();

		points_.reserve(indices.size());

		for (std::size_t idx = 0; idx < indices.size(); ++idx)
		{
			const std::size_t i = indices[idx];
			const std::size_t b = i * 3U;
			points_.emplace_back(static_cast<double>(xyz[b]), static_cast<double>(xyz[b + 1U]),
								 static_cast<double>(xyz[b + 2U]));
			// 精确坐标做 map 键：完全同坐标的重复点坍缩为一个键；用 emplace 保首现索引，
			// 避免后写点覆盖先写点导致原始索引永久丢失（ICP 只取位置无害，回查法向/颜色需注意）
			indexMap_.emplace(points_.back(), i);
		}

		if (!points_.empty())
		{
			tree_.reset(new Tree(points_.begin(), points_.end()));
		}
	}

	bool empty() const { return !tree_ || tree_->empty(); }

	std::size_t size() const { return points_.size(); }

	/// 最近邻；未命中返回 size_t(-1)
	std::size_t findNearest(double qx, double qy, double qz, double maxDistSq, double& outDistSq) const
	{
		if (empty())
		{
			outDistSq = std::numeric_limits<double>::max();
			return static_cast<std::size_t>(-1);
		}

		const Point_3 query(qx, qy, qz);
		K_neighbor_search search(*tree_, query, 1);

		auto it = search.begin();
		if (it == search.end())
		{
			outDistSq = std::numeric_limits<double>::max();
			return static_cast<std::size_t>(-1);
		}

		outDistSq = it->second;
		if (outDistSq > maxDistSq)
		{
			return static_cast<std::size_t>(-1);
		}

		return findIndex(it->first);
	}

	// 查找 K 近邻
	void findKNearest(double qx, double qy, double qz, unsigned int k, std::vector<std::size_t>& outIndices,
					  std::vector<double>& outDistSq) const
	{
		outIndices.clear();
		outDistSq.clear();

		if (empty() || k == 0)
		{
			return;
		}

		const Point_3 query(qx, qy, qz);
		K_neighbor_search search(*tree_, query, k);

		outIndices.reserve(k);
		outDistSq.reserve(k);

		for (auto it = search.begin(); it != search.end(); ++it)
		{
			outDistSq.push_back(it->second);
			outIndices.push_back(findIndex(it->first));
		}
	}

private:
	// 查找点的原始索引
	std::size_t findIndex(const Point_3& point) const
	{
		auto it = indexMap_.find(point);
		if (it != indexMap_.end())
		{
			return it->second;
		}
		return static_cast<std::size_t>(-1);
	}

	// 用于 map 比较的点比较器
	struct PointCompare
	{
		bool operator()(const Point_3& a, const Point_3& b) const
		{
			if (a.x() != b.x())
				return a.x() < b.x();
			if (a.y() != b.y())
				return a.y() < b.y();
			return a.z() < b.z();
		}
	};

	std::vector<Point_3> points_;
	std::map<Point_3, std::size_t, PointCompare> indexMap_;
	std::unique_ptr<Tree> tree_;
};

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_KDTREEPOINTSET_H
