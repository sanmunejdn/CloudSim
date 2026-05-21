static inline int64_t quantVertexCoord(float v)
{
	return static_cast<int64_t>(std::llround(static_cast<double>(v) * 1000000.0));
}

struct WeldKey
{
	int64_t x, y, z;
	bool operator==(const WeldKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct WeldKeyHash
{
	size_t operator()(const WeldKey& k) const noexcept
	{
		size_t h = 1469598103934665603ull;
		h ^= static_cast<size_t>(k.x);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.y);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.z);
		h *= 1099511628211ull;
		return h;
	}
};

struct EdgeIdxKey
{
	std::uint32_t lo, hi;
	bool operator==(const EdgeIdxKey& o) const { return lo == o.lo && hi == o.hi; }
};

struct EdgeIdxKeyHash
{
	size_t operator()(const EdgeIdxKey& k) const noexcept
	{
		return (static_cast<size_t>(k.lo) << 32) | static_cast<size_t>(k.hi);
	}
};

/// Boundary edges plus creases where adjacent triangle normals differ by more than \a creaseAngleDeg.
osg::ref_ptr<osg::Geometry> buildMeshFeatureEdgeGeometry(const std::vector<float>& soup, const osg::Vec4& fillColor,
	float creaseAngleDeg)
{
	const std::size_t nTri = soup.size() / 9U;
	if (nTri == 0U)
	{
		return nullptr;
	}
	std::unordered_map<WeldKey, std::uint32_t, WeldKeyHash> weld;
	weld.reserve(nTri * 2U);
	std::vector<osg::Vec3> verts;
	verts.reserve(nTri * 2U);
	std::vector<std::array<std::uint32_t, 3>> triIdx(nTri);

	auto addVertex = [&](float px, float py, float pz) -> std::uint32_t
	{
		const WeldKey k{ quantVertexCoord(px), quantVertexCoord(py), quantVertexCoord(pz) };
		const auto it = weld.find(k);
		if (it != weld.end())
		{
			return it->second;
		}
		const std::uint32_t id = static_cast<std::uint32_t>(verts.size());
		verts.emplace_back(px, py, pz);
		weld.emplace(k, id);
		return id;
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const std::size_t b = t * 9U;
		triIdx[t][0] = addVertex(soup[b + 0], soup[b + 1], soup[b + 2]);
		triIdx[t][1] = addVertex(soup[b + 3], soup[b + 4], soup[b + 5]);
		triIdx[t][2] = addVertex(soup[b + 6], soup[b + 7], soup[b + 8]);
	}

	std::unordered_map<EdgeIdxKey, std::vector<std::uint32_t>, EdgeIdxKeyHash> edgeTris;
	edgeTris.reserve(nTri * 2U);

	auto addEdge = [&](std::uint32_t a, std::uint32_t b, std::uint32_t triId)
	{
		if (a == b)
		{
			return;
		}
		const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
		edgeTris[ek].push_back(triId);
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const auto& tr = triIdx[t];
		const auto tid = static_cast<std::uint32_t>(t);
		addEdge(tr[0], tr[1], tid);
		addEdge(tr[1], tr[2], tid);
		addEdge(tr[2], tr[0], tid);
	}

	std::vector<osg::Vec3f> triNor(nTri);
	for (std::size_t t = 0; t < nTri; ++t)
	{
		const osg::Vec3& p0 = verts[triIdx[t][0]];
		const osg::Vec3& p1 = verts[triIdx[t][1]];
		const osg::Vec3& p2 = verts[triIdx[t][2]];
		const osg::Vec3 e1 = p1 - p0;
		const osg::Vec3 e2 = p2 - p0;
		osg::Vec3 n = e1 ^ e2;
		const float len2 = n.length2();
		if (len2 > 1e-20f)
		{
			n.normalize();
			triNor[t] = osg::Vec3f(n.x(), n.y(), n.z());
		}
		else
		{
			triNor[t] = osg::Vec3f(0.0f, 0.0f, 1.0f);
		}
	}

	const float cosThreshold = static_cast<float>(
		std::cos(static_cast<double>(creaseAngleDeg) * 3.14159265358979323846 / 180.0));

	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;

	for (const auto& ep : edgeTris)
	{
		const std::vector<std::uint32_t>& triList = ep.second;
		const std::size_t cnt = triList.size();
		if (cnt == 1U)
		{
			lineVerts->push_back(verts[ep.first.lo]);
			lineVerts->push_back(verts[ep.first.hi]);
		}
		else if (cnt == 2U)
		{
			const std::uint32_t t0 = triList[0];
			const std::uint32_t t1 = triList[1];
			const osg::Vec3f& n0 = triNor[t0];
			const osg::Vec3f& n1 = triNor[t1];
			const float dot = n0.x() * n1.x() + n0.y() * n1.y() + n0.z() * n1.z();
			if (dot < cosThreshold)
			{
				lineVerts->push_back(verts[ep.first.lo]);
				lineVerts->push_back(verts[ep.first.hi]);
			}
		}
		else if (cnt > 2U)
		{
			lineVerts->push_back(verts[ep.first.lo]);
			lineVerts->push_back(verts[ep.first.hi]);
		}
	}

	if (lineVerts->empty())
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(lineVerts.get());
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVerts->size())));
	osg::ref_ptr<osg::Vec4Array> mcWire = new osg::Vec4Array;
	mcWire->push_back(osg::Vec4(
		std::max(0.12f, fillColor.r() * 0.38f),
		std::max(0.12f, fillColor.g() * 0.38f),
		std::max(0.12f, fillColor.b() * 0.38f),
		fillColor.a()));
	geom->setColorArray(mcWire.get(), osg::Array::BIND_OVERALL);
	return geom;
}

/// 鎸夈€屽叡闈㈣繛閫氶潰鐗囥€嶅缓澶栬疆寤擄細鍚屼竴闈㈢墖鍐呬笁瑙掑叡浜竟鍑虹幇 2 娆★紙鍐呴儴鍓栧垎绾匡級涓嶇敾锛涗粎鍑虹幇 1 娆＄殑杈逛负璇ョ墖澶栬竟鐣岋紙鍚笌閭婚潰/缃戞牸寮€鍙ｇ殑杈癸級銆?
osg::ref_ptr<osg::Geometry> buildMeshOutlineWireGeometry(const std::vector<float>& soup, const osg::Vec4& fillColor)
{
	const std::size_t nTri = soup.size() / 9U;
	if (nTri == 0U)
	{
		return nullptr;
	}
	std::unordered_map<WeldKey, std::uint32_t, WeldKeyHash> weld;
	weld.reserve(nTri * 2U);
	std::vector<osg::Vec3> verts;
	verts.reserve(nTri * 2U);
	std::vector<std::array<std::uint32_t, 3>> triIdx(nTri);

	auto addVertex = [&](float px, float py, float pz) -> std::uint32_t
	{
		const WeldKey k{ quantVertexCoord(px), quantVertexCoord(py), quantVertexCoord(pz) };
		const auto it = weld.find(k);
		if (it != weld.end())
		{
			return it->second;
		}
		const std::uint32_t id = static_cast<std::uint32_t>(verts.size());
		verts.emplace_back(px, py, pz);
		weld.emplace(k, id);
		return id;
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const std::size_t b = t * 9U;
		triIdx[t][0] = addVertex(soup[b + 0], soup[b + 1], soup[b + 2]);
		triIdx[t][1] = addVertex(soup[b + 3], soup[b + 4], soup[b + 5]);
		triIdx[t][2] = addVertex(soup[b + 6], soup[b + 7], soup[b + 8]);
	}

	std::unordered_map<EdgeIdxKey, std::vector<std::uint32_t>, EdgeIdxKeyHash> edgeTris;
	edgeTris.reserve(nTri * 2U);

	auto addEdge = [&](std::uint32_t a, std::uint32_t b, std::uint32_t triId)
	{
		if (a == b)
		{
			return;
		}
		const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
		edgeTris[ek].push_back(triId);
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const auto& tr = triIdx[t];
		const auto tid = static_cast<std::uint32_t>(t);
		addEdge(tr[0], tr[1], tid);
		addEdge(tr[1], tr[2], tid);
		addEdge(tr[2], tr[0], tid);
	}

	std::vector<osg::Vec3f> triNor(nTri);
	for (std::size_t t = 0; t < nTri; ++t)
	{
		const osg::Vec3& p0 = verts[triIdx[t][0]];
		const osg::Vec3& p1 = verts[triIdx[t][1]];
		const osg::Vec3& p2 = verts[triIdx[t][2]];
		const osg::Vec3 e1 = p1 - p0;
		const osg::Vec3 e2 = p2 - p0;
		osg::Vec3 n = e1 ^ e2;
		const float len2 = n.length2();
		if (len2 > 1e-20f)
		{
			n.normalize();
			triNor[t] = osg::Vec3f(n.x(), n.y(), n.z());
		}
		else
		{
			triNor[t] = osg::Vec3f(0.0f, 0.0f, 1.0f);
		}
	}

	float minx = verts[0].x(), maxx = verts[0].x();
	float miny = verts[0].y(), maxy = verts[0].y();
	float minz = verts[0].z(), maxz = verts[0].z();
	for (const osg::Vec3& p : verts)
	{
		minx = std::min(minx, p.x());
		maxx = std::max(maxx, p.x());
		miny = std::min(miny, p.y());
		maxy = std::max(maxy, p.y());
		minz = std::min(minz, p.z());
		maxz = std::max(maxz, p.z());
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	const float planeEps = std::max(1e-7f, diag * 1e-6f);

	auto neighborsOf = [&](std::uint32_t t) -> std::vector<std::uint32_t>
	{
		std::vector<std::uint32_t> nb;
		for (int e = 0; e < 3; ++e)
		{
			const std::uint32_t a = triIdx[t][static_cast<std::size_t>(e)];
			const std::uint32_t b = triIdx[t][static_cast<std::size_t>((e + 1) % 3)];
			const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
			const auto it = edgeTris.find(ek);
			if (it == edgeTris.end())
			{
				continue;
			}
			for (const std::uint32_t tid : it->second)
			{
				if (tid != t)
				{
					nb.push_back(tid);
				}
			}
		}
		return nb;
	};

	auto coplanarWithSeed = [&](std::uint32_t seed, std::uint32_t u) -> bool
	{
		const osg::Vec3f& nSeed = triNor[seed];
		const osg::Vec3 p0Seed = verts[triIdx[seed][0]];
		const osg::Vec3f& nu = triNor[u];
		if (std::fabs(static_cast<double>(nSeed.x() * nu.x() + nSeed.y() * nu.y() + nSeed.z() * nu.z())) < 0.998)
		{
			return false;
		}
		for (int k = 0; k < 3; ++k)
		{
			const osg::Vec3& v = verts[triIdx[u][static_cast<std::size_t>(k)]];
			const osg::Vec3 w = v - p0Seed;
			const float d = std::fabs(
				nSeed.x() * static_cast<float>(w.x()) + nSeed.y() * static_cast<float>(w.y()) + nSeed.z() * static_cast<float>(w.z()));
			if (d > planeEps)
			{
				return false;
			}
		}
		return true;
	};

	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;
	std::vector<int> patchOf(static_cast<std::size_t>(nTri), -1);

	for (std::size_t seed = 0; seed < nTri; ++seed)
	{
		if (patchOf[seed] != -1)
		{
			continue;
		}
		const int patchId = static_cast<int>(seed);
		std::vector<std::uint32_t> patchTris;
		std::queue<std::uint32_t> q;
		patchOf[seed] = patchId;
		patchTris.push_back(static_cast<std::uint32_t>(seed));
		q.push(static_cast<std::uint32_t>(seed));
		while (!q.empty())
		{
			const std::uint32_t cur = q.front();
			q.pop();
			for (const std::uint32_t u : neighborsOf(cur))
			{
				if (patchOf[u] != -1)
				{
					continue;
				}
				if (!coplanarWithSeed(static_cast<std::uint32_t>(seed), u))
				{
					continue;
				}
				patchOf[u] = patchId;
				patchTris.push_back(u);
				q.push(u);
			}
		}

		std::unordered_map<EdgeIdxKey, int, EdgeIdxKeyHash> edgeCountInPatch;
		for (const std::uint32_t ti : patchTris)
		{
			for (int e = 0; e < 3; ++e)
			{
				const std::uint32_t a = triIdx[ti][static_cast<std::size_t>(e)];
				const std::uint32_t b = triIdx[ti][static_cast<std::size_t>((e + 1) % 3)];
				const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
				edgeCountInPatch[ek]++;
			}
		}
		for (const auto& ep : edgeCountInPatch)
		{
			if (ep.second == 1)
			{
				lineVerts->push_back(verts[ep.first.lo]);
				lineVerts->push_back(verts[ep.first.hi]);
			}
		}
	}

	if (lineVerts->empty())
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(lineVerts.get());
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVerts->size())));
	osg::ref_ptr<osg::Vec4Array> mcWire = new osg::Vec4Array;
	mcWire->push_back(osg::Vec4(
		std::max(0.12f, fillColor.r() * 0.38f),
		std::max(0.12f, fillColor.g() * 0.38f),
		std::max(0.12f, fillColor.b() * 0.38f),
		fillColor.a()));
	geom->setColorArray(mcWire.get(), osg::Array::BIND_OVERALL);
	return geom;
}
