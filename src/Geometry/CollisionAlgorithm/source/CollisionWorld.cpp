/// @file CollisionWorld.cpp
/// @brief 内置网格碰撞：宽相 AABB（含安全余量）+ 三角-三角窄相

#include "CollisionWorld.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace collision
{
namespace
{
struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 operator-(const Vec3& a, const Vec3& b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 operator*(const Vec3& a, const double s)
{
	return {a.x * s, a.y * s, a.z * s};
}
double dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 cross(const Vec3& a, const Vec3& b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 transformPoint(const Mat4& m, const Vec3& p)
{
	// 列主序：p' = R*p + t
	const double x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
	const double y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
	const double z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
	return {x, y, z};
}

struct Aabb
{
	Vec3 mn{1e300, 1e300, 1e300};
	Vec3 mx{-1e300, -1e300, -1e300};
	bool valid = false;

	void expand(const Vec3& p)
	{
		mn.x = std::min(mn.x, p.x);
		mn.y = std::min(mn.y, p.y);
		mn.z = std::min(mn.z, p.z);
		mx.x = std::max(mx.x, p.x);
		mx.y = std::max(mx.y, p.y);
		mx.z = std::max(mx.z, p.z);
		valid = true;
	}

	void inflate(const double m)
	{
		if (!valid)
			return;
		mn.x -= m;
		mn.y -= m;
		mn.z -= m;
		mx.x += m;
		mx.y += m;
		mx.z += m;
	}

	bool overlaps(const Aabb& o) const
	{
		if (!valid || !o.valid)
			return false;
		return mn.x <= o.mx.x && mx.x >= o.mn.x && mn.y <= o.mx.y && mx.y >= o.mn.y && mn.z <= o.mx.z &&
			   mx.z >= o.mn.z;
	}
};

// Möller 三角-三角：投影分离轴
bool triTriIntersect(const Vec3& V0, const Vec3& V1, const Vec3& V2, const Vec3& U0, const Vec3& U1, const Vec3& U2)
{
	const Vec3 e1 = V1 - V0;
	const Vec3 e2 = V2 - V0;
	const Vec3 n1 = cross(e1, e2);
	const double d1 = -dot(n1, V0);
	double du0 = dot(n1, U0) + d1;
	double du1 = dot(n1, U1) + d1;
	double du2 = dot(n1, U2) + d1;
	const double eps = 1e-8;
	if (std::fabs(du0) < eps)
		du0 = 0.0;
	if (std::fabs(du1) < eps)
		du1 = 0.0;
	if (std::fabs(du2) < eps)
		du2 = 0.0;
	if ((du0 > 0.0 && du1 > 0.0 && du2 > 0.0) || (du0 < 0.0 && du1 < 0.0 && du2 < 0.0))
		return false;

	const Vec3 f1 = U1 - U0;
	const Vec3 f2 = U2 - U0;
	const Vec3 n2 = cross(f1, f2);
	const double d2 = -dot(n2, U0);
	double dv0 = dot(n2, V0) + d2;
	double dv1 = dot(n2, V1) + d2;
	double dv2 = dot(n2, V2) + d2;
	if (std::fabs(dv0) < eps)
		dv0 = 0.0;
	if (std::fabs(dv1) < eps)
		dv1 = 0.0;
	if (std::fabs(dv2) < eps)
		dv2 = 0.0;
	if ((dv0 > 0.0 && dv1 > 0.0 && dv2 > 0.0) || (dv0 < 0.0 && dv1 < 0.0 && dv2 < 0.0))
		return false;

	const Vec3 D = cross(n1, n2);
	const double adx = std::fabs(D.x);
	const double ady = std::fabs(D.y);
	const double adz = std::fabs(D.z);
	int maxc = 0;
	if (ady > adx)
		maxc = 1;
	if (adz > (maxc == 0 ? adx : ady))
		maxc = 2;

	auto proj = [maxc](const Vec3& p) {
		if (maxc == 0)
			return p.x;
		if (maxc == 1)
			return p.y;
		return p.z;
	};

	const double pv0 = proj(V0);
	const double pv1 = proj(V1);
	const double pv2 = proj(V2);
	const double pu0 = proj(U0);
	const double pu1 = proj(U1);
	const double pu2 = proj(U2);

	auto interval = [](double a, double b, double c, double da, double db, double dc, double& lo, double& hi) {
		lo = 1e300;
		hi = -1e300;
		if (da * db < 0.0)
		{
			const double t = a + (b - a) * (da / (da - db));
			lo = std::min(lo, t);
			hi = std::max(hi, t);
		}
		if (da * dc < 0.0)
		{
			const double t = a + (c - a) * (da / (da - dc));
			lo = std::min(lo, t);
			hi = std::max(hi, t);
		}
		if (db * dc < 0.0)
		{
			const double t = b + (c - b) * (db / (db - dc));
			lo = std::min(lo, t);
			hi = std::max(hi, t);
		}
		if (da == 0.0)
		{
			lo = std::min(lo, a);
			hi = std::max(hi, a);
		}
		if (db == 0.0)
		{
			lo = std::min(lo, b);
			hi = std::max(hi, b);
		}
		if (dc == 0.0)
		{
			lo = std::min(lo, c);
			hi = std::max(hi, c);
		}
	};

	double isect1[2];
	double isect2[2];
	interval(pv0, pv1, pv2, dv0, dv1, dv2, isect1[0], isect1[1]);
	interval(pu0, pu1, pu2, du0, du1, du2, isect2[0], isect2[1]);
	if (isect1[0] > isect1[1])
		std::swap(isect1[0], isect1[1]);
	if (isect2[0] > isect2[1])
		std::swap(isect2[0], isect2[1]);
	return !(isect1[1] < isect2[0] - eps || isect2[1] < isect1[0] - eps);
}

struct Body
{
	CollisionBodyId id;
	std::vector<Vec3> localVerts; // 每三角 3 顶点连续
	Mat4 world = {};
	Aabb worldAabb;
	bool dirtyPose = true;
	std::string poseSource;

	void rebuildWorldAabb(const double margin)
	{
		worldAabb = {};
		for (const Vec3& lp : localVerts)
			worldAabb.expand(transformPoint(world, lp));
		worldAabb.inflate(margin);
		dirtyPose = false;
	}
};

std::string pairKey(const std::string& a, const std::string& b)
{
	if (a < b)
		return a + "|" + b;
	return b + "|" + a;
}

} // namespace

std::string bodyIdKey(const CollisionBodyId& id)
{
	return id.kind + ":" + id.backendId + ":" + id.linkName;
}

struct CollisionWorld::Impl
{
	std::unordered_map<std::string, Body> bodies;
	std::unordered_set<std::string> exclude;
	double marginMm = 1.0;
};

CollisionWorld::CollisionWorld() : m_impl(new Impl) {}
CollisionWorld::~CollisionWorld()
{
	delete m_impl;
	m_impl = nullptr;
}

void CollisionWorld::clear()
{
	m_impl->bodies.clear();
	m_impl->exclude.clear();
}

void CollisionWorld::upsertMeshBody(const CollisionBodyId& id, const float* soup, const std::size_t nFloats,
									const Mat4& worldMm, const std::string& poseSource)
{
	if (!soup || nFloats < 9 || (nFloats % 9) != 0)
		return;
	Body& b = m_impl->bodies[bodyIdKey(id)];
	b.id = id;
	b.localVerts.clear();
	b.localVerts.reserve(nFloats / 3);
	for (std::size_t i = 0; i + 2 < nFloats; i += 3)
		b.localVerts.push_back({soup[i], soup[i + 1], soup[i + 2]});
	b.world = worldMm;
	if (!poseSource.empty())
		b.poseSource = poseSource;
	b.rebuildWorldAabb(m_impl->marginMm);
}

void CollisionWorld::setWorldPose(const CollisionBodyId& id, const Mat4& worldMm, const std::string& poseSource)
{
	const auto it = m_impl->bodies.find(bodyIdKey(id));
	if (it == m_impl->bodies.end())
		return;
	it->second.world = worldMm;
	if (!poseSource.empty())
		it->second.poseSource = poseSource;
	it->second.rebuildWorldAabb(m_impl->marginMm);
}

void CollisionWorld::removeBody(const CollisionBodyId& id)
{
	m_impl->bodies.erase(bodyIdKey(id));
}

void CollisionWorld::clearExcludePairs()
{
	m_impl->exclude.clear();
}

void CollisionWorld::setExcludePair(const CollisionBodyId& a, const CollisionBodyId& b)
{
	m_impl->exclude.insert(pairKey(bodyIdKey(a), bodyIdKey(b)));
}

void CollisionWorld::setSecurityMarginMm(const double m)
{
	m_impl->marginMm = std::max(0.0, m);
	for (auto& kv : m_impl->bodies)
		kv.second.rebuildWorldAabb(m_impl->marginMm);
}

double CollisionWorld::securityMarginMm() const
{
	return m_impl->marginMm;
}

std::size_t CollisionWorld::bodyCount() const
{
	return m_impl->bodies.size();
}

bool CollisionWorld::hasCoalBackend()
{
#if defined(CLOUDSIM_HAS_COAL)
	return true;
#else
	return false;
#endif
}

CollisionQueryResult CollisionWorld::checkAll(const int maxContacts) const
{
	CollisionQueryResult out;
	std::vector<const Body*> list;
	list.reserve(m_impl->bodies.size());
	for (const auto& kv : m_impl->bodies)
		list.push_back(&kv.second);

	auto fillDiag = [](ContactHit& c, const Body& A, const Body& B) {
		c.aOriginMm[0] = A.world[12];
		c.aOriginMm[1] = A.world[13];
		c.aOriginMm[2] = A.world[14];
		c.bOriginMm[0] = B.world[12];
		c.bOriginMm[1] = B.world[13];
		c.bOriginMm[2] = B.world[14];
		c.aAabbCenterMm[0] = 0.5 * (A.worldAabb.mn.x + A.worldAabb.mx.x);
		c.aAabbCenterMm[1] = 0.5 * (A.worldAabb.mn.y + A.worldAabb.mx.y);
		c.aAabbCenterMm[2] = 0.5 * (A.worldAabb.mn.z + A.worldAabb.mx.z);
		c.bAabbCenterMm[0] = 0.5 * (B.worldAabb.mn.x + B.worldAabb.mx.x);
		c.bAabbCenterMm[1] = 0.5 * (B.worldAabb.mn.y + B.worldAabb.mx.y);
		c.bAabbCenterMm[2] = 0.5 * (B.worldAabb.mn.z + B.worldAabb.mx.z);
		c.aPoseSource = A.poseSource.empty() ? "?" : A.poseSource;
		c.bPoseSource = B.poseSource.empty() ? "?" : B.poseSource;
	};

	auto formatSummary = [](const ContactHit& c, const bool more) {
		std::ostringstream oss;
		oss << "collision: " << c.a.backendId << " vs " << c.b.backendId;
		if (more)
			oss << " (+more)";
		oss << " | A.t=(" << c.aOriginMm[0] << "," << c.aOriginMm[1] << "," << c.aOriginMm[2] << ") aabbC=("
			<< c.aAabbCenterMm[0] << "," << c.aAabbCenterMm[1] << "," << c.aAabbCenterMm[2] << ") pose="
			<< c.aPoseSource << " | B.t=(" << c.bOriginMm[0] << "," << c.bOriginMm[1] << "," << c.bOriginMm[2]
			<< ") aabbC=(" << c.bAabbCenterMm[0] << "," << c.bAabbCenterMm[1] << "," << c.bAabbCenterMm[2]
			<< ") pose=" << c.bPoseSource;
		return oss.str();
	};

	const int cap = std::max(1, maxContacts);
	for (std::size_t i = 0; i < list.size(); ++i)
	{
		for (std::size_t j = i + 1; j < list.size(); ++j)
		{
			const Body& A = *list[i];
			const Body& B = *list[j];
			if (m_impl->exclude.count(pairKey(bodyIdKey(A.id), bodyIdKey(B.id))) > 0)
				continue;
			if (!A.worldAabb.overlaps(B.worldAabb))
				continue;

			const std::size_t nA = A.localVerts.size() / 3;
			const std::size_t nB = B.localVerts.size() / 3;
			bool hit = false;
			Vec3 hitPt{};
			for (std::size_t ta = 0; ta < nA && !hit; ++ta)
			{
				const Vec3 a0 = transformPoint(A.world, A.localVerts[ta * 3 + 0]);
				const Vec3 a1 = transformPoint(A.world, A.localVerts[ta * 3 + 1]);
				const Vec3 a2 = transformPoint(A.world, A.localVerts[ta * 3 + 2]);
				for (std::size_t tb = 0; tb < nB; ++tb)
				{
					const Vec3 b0 = transformPoint(B.world, B.localVerts[tb * 3 + 0]);
					const Vec3 b1 = transformPoint(B.world, B.localVerts[tb * 3 + 1]);
					const Vec3 b2 = transformPoint(B.world, B.localVerts[tb * 3 + 2]);
					if (triTriIntersect(a0, a1, a2, b0, b1, b2))
					{
						hit = true;
						hitPt = (a0 + a1 + a2 + b0 + b1 + b2) * (1.0 / 6.0);
						break;
					}
				}
			}
			if (!hit)
				continue;

			out.inCollision = true;
			ContactHit c;
			c.a = A.id;
			c.b = B.id;
			c.pointMm[0] = hitPt.x;
			c.pointMm[1] = hitPt.y;
			c.pointMm[2] = hitPt.z;
			c.depthMm = m_impl->marginMm;
			fillDiag(c, A, B);
			out.contacts.push_back(c);
			if (static_cast<int>(out.contacts.size()) >= cap)
			{
				out.summary = formatSummary(out.contacts.front(), true);
				return out;
			}
		}
	}

	if (out.inCollision && !out.contacts.empty())
	{
		out.summary = formatSummary(out.contacts.front(), false);
		if (out.contacts.size() > 1)
		{
			std::ostringstream oss;
			oss << out.summary << " (" << out.contacts.size() << " contacts)";
			out.summary = oss.str();
		}
	}
	return out;
}

} // namespace collision
