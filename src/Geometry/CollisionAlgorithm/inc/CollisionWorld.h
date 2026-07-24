#ifndef COLLISIONALGORITHM_COLLISIONWORLD_H
#define COLLISIONALGORITHM_COLLISIONWORLD_H

/// @file CollisionWorld.h
/// @brief 多体网格碰撞：模型系 soup + 世界 Mat4，ACM 排除对

#include "collision_algorithm_global.h"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace collision
{
using Mat4 = std::array<double, 16>;

struct COLLISION_ALGORITHM_API CollisionBodyId
{
	std::string kind; // robotLink | scene
	std::string backendId;
	std::string linkName;

	bool operator==(const CollisionBodyId& o) const
	{
		return kind == o.kind && backendId == o.backendId && linkName == o.linkName;
	}
};

struct COLLISION_ALGORITHM_API ContactHit
{
	CollisionBodyId a;
	CollisionBodyId b;
	double pointMm[3]{0.0, 0.0, 0.0};
	double normal[3]{0.0, 0.0, 1.0};
	double depthMm = 0.0;
};

struct COLLISION_ALGORITHM_API CollisionQueryResult
{
	bool inCollision = false;
	std::vector<ContactHit> contacts;
	std::string summary;
};

/// 多体碰撞世界（内置 AABB+三角；可选 CLOUDSIM_HAS_COAL）
class COLLISION_ALGORITHM_API CollisionWorld
{
public:
	CollisionWorld();
	~CollisionWorld();

	CollisionWorld(const CollisionWorld&) = delete;
	CollisionWorld& operator=(const CollisionWorld&) = delete;

	void clear();

	/// soup：模型系，每三角 9 float（xyz×3）；colMajor16 模型→世界
	void upsertMeshBody(const CollisionBodyId& id, const float* soup, std::size_t nFloats, const Mat4& worldMm);

	void setWorldPose(const CollisionBodyId& id, const Mat4& worldMm);
	void removeBody(const CollisionBodyId& id);

	void clearExcludePairs();
	void setExcludePair(const CollisionBodyId& a, const CollisionBodyId& b);

	void setSecurityMarginMm(double m);
	double securityMarginMm() const;

	/// 最多收集 maxContacts 个接触（默认 8）
	CollisionQueryResult checkAll(int maxContacts = 8) const;

	std::size_t bodyCount() const;

	/// 运行时是否链到 coal（否则为内置实现）
	static bool hasCoalBackend();

private:
	struct Impl;
	Impl* m_impl = nullptr;
};

COLLISION_ALGORITHM_API std::string bodyIdKey(const CollisionBodyId& id);

} // namespace collision

#endif // COLLISIONALGORITHM_COLLISIONWORLD_H
