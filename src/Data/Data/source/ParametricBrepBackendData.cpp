// @file ParametricBrepBackendData.cpp

#include "ParametricBrepBackendData.h"

#include "BackendTypeIdentity.h"
#include "ShapeQuery.h"
#include "SketchExtrude.h"
#include "SketchPlane.h"
#include "SketchSweep.h"

#include <cstdint>

namespace
{
geoalgo::SketchExtrudeEndCondition toAlgoEnd(ParametricExtrudeEnd e)
{
	switch (e)
	{
	case ParametricExtrudeEnd::UpToFace:
		return geoalgo::SketchExtrudeEndCondition::UpToFace;
	case ParametricExtrudeEnd::MidPlane:
		return geoalgo::SketchExtrudeEndCondition::MidPlane;
	case ParametricExtrudeEnd::ThroughAll:
		return geoalgo::SketchExtrudeEndCondition::ThroughAll;
	default:
		return geoalgo::SketchExtrudeEndCondition::Blind;
	}
}

void applyBakedUpToFace(const ParametricFeature& feat, geoalgo::SketchExtrudeParams& ep)
{
	ep.hasUpToFace = feat.hasUpToFacePlane;
	ep.upOriginX = feat.upToFacePlane.originX;
	ep.upOriginY = feat.upToFacePlane.originY;
	ep.upOriginZ = feat.upToFacePlane.originZ;
	ep.upNormalX = feat.upToFacePlane.normalX;
	ep.upNormalY = feat.upToFacePlane.normalY;
	ep.upNormalZ = feat.upToFacePlane.normalZ;
}
} // namespace

ParametricBrepBackendData::ParametricBrepBackendData() = default;

std::string ParametricBrepBackendData::className() const
{
	return backend_type::kClassParametricBrep;
}

std::string ParametricBrepBackendData::nextId(const char* prefix)
{
	return std::string(prefix) + "_" + std::to_string(m_seq++);
}

std::string ParametricBrepBackendData::addSketch(const ParametricSketchPlane& plane, const std::string& name)
{
	ParametricFeature f;
	f.id = nextId("Sketch");
	f.name = name.empty() ? f.id : name;
	f.kind = ParametricFeatureKind::Sketch;
	f.plane = plane;
	m_features.push_back(std::move(f));
	return m_features.back().id;
}

std::string ParametricBrepBackendData::addPad(const std::string& sketchId, double lengthMm, bool reversed)
{
	ParametricFeature f;
	f.id = nextId("Pad");
	f.name = f.id;
	f.kind = ParametricFeatureKind::Pad;
	f.sketchRefId = sketchId;
	f.lengthMm = lengthMm;
	f.reversed = reversed;
	m_features.push_back(std::move(f));
	return m_features.back().id;
}

std::string ParametricBrepBackendData::addPocket(const std::string& sketchId, double lengthMm, bool reversed)
{
	ParametricFeature f;
	f.id = nextId("Pocket");
	f.name = f.id;
	f.kind = ParametricFeatureKind::Pocket;
	f.sketchRefId = sketchId;
	f.lengthMm = lengthMm;
	f.reversed = reversed;
	m_features.push_back(std::move(f));
	return m_features.back().id;
}

std::string ParametricBrepBackendData::addSweep(const std::string& profileSketchId, const std::string& pathSketchId,
											   bool cut)
{
	ParametricFeature f;
	f.id = nextId(cut ? "SweepCut" : "Sweep");
	f.name = f.id;
	f.kind = cut ? ParametricFeatureKind::SweepCut : ParametricFeatureKind::Sweep;
	f.sketchRefId = profileSketchId;
	f.pathSketchRefId = pathSketchId;
	m_features.push_back(std::move(f));
	return m_features.back().id;
}

bool ParametricBrepBackendData::setProfile(const std::string& sketchId, const std::vector<float>& xyz)
{
	ParametricFeature* f = findFeature(sketchId);
	if (!f || f->kind != ParametricFeatureKind::Sketch)
		return false;
	f->profileXyzMm = xyz;
	return true;
}

bool ParametricBrepBackendData::setLength(const std::string& featureId, double lengthMm)
{
	ParametricFeature* f = findFeature(featureId);
	if (!f || (f->kind != ParametricFeatureKind::Pad && f->kind != ParametricFeatureKind::Pocket))
		return false;
	f->lengthMm = lengthMm;
	return true;
}

ParametricFeature* ParametricBrepBackendData::findFeature(const std::string& id)
{
	for (auto& f : m_features)
	{
		if (f.id == id)
			return &f;
	}
	return nullptr;
}

const ParametricFeature* ParametricBrepBackendData::findFeature(const std::string& id) const
{
	for (const auto& f : m_features)
	{
		if (f.id == id)
			return &f;
	}
	return nullptr;
}

void ParametricBrepBackendData::setFeatures(std::vector<ParametricFeature> features)
{
	m_features = std::move(features);
	m_seq = 1;
	m_faceOwnerByIndex.clear();
	for (const auto& f : m_features)
	{
		// 保持 id 序号单调，避免与已有冲突
		const auto pos = f.id.find_last_of('_');
		if (pos != std::string::npos)
		{
			try
			{
				const int n = std::stoi(f.id.substr(pos + 1));
				if (n >= m_seq)
					m_seq = n + 1;
			}
			catch (...)
			{
			}
		}
	}
}

void ParametricBrepBackendData::clearFeatures()
{
	m_features.clear();
	m_seq = 1;
	m_faceOwnerByIndex.clear();
}

const ParametricFeature* ParametricBrepBackendData::findSketchFor(const ParametricFeature& feat) const
{
	return findFeature(feat.sketchRefId);
}

std::string ParametricBrepBackendData::featureIdForFace(int faceIndex) const
{
	const auto it = m_faceOwnerByIndex.find(faceIndex);
	if (it == m_faceOwnerByIndex.end())
		return {};
	return it->second;
}

bool ParametricBrepBackendData::rebuild(std::string* errMsg)
{
	geoalgo::ShapeHandle tip;
	std::unordered_map<std::uintptr_t, std::string> tshapeOwners;
	m_faceOwnerByIndex.clear();

	for (const auto& feat : m_features)
	{
		if (feat.suppressed || feat.kind == ParametricFeatureKind::Sketch)
			continue;

		const geoalgo::ShapeHandle* basePtr = tip.isNull() ? nullptr : &tip;
		geoalgo::ShapeHandle next;
		std::string err;

		if (feat.kind == ParametricFeatureKind::Sweep || feat.kind == ParametricFeatureKind::SweepCut)
		{
			const ParametricFeature* profileSk = findSketchFor(feat);
			const ParametricFeature* pathSk = findFeature(feat.pathSketchRefId);
			// 草图改后优先活几何，烤坐标仅作离线兜底
			std::vector<float> profile =
				(profileSk && profileSk->profileXyzMm.size() >= 12) ? profileSk->profileXyzMm : feat.profileXyzMm;
			std::vector<float> path =
				(pathSk && pathSk->profileXyzMm.size() >= 6) ? pathSk->profileXyzMm : feat.pathXyzMm;
			if (profile.size() < 12)
			{
				if (errMsg)
					*errMsg = "missing sweep profile for " + feat.id;
				return false;
			}
			if (feat.kind == ParametricFeatureKind::SweepCut && tip.isNull())
			{
				if (errMsg)
					*errMsg = "SweepCut requires existing solid tip";
				return false;
			}
			geoalgo::SketchSweepParams sp;
			sp.mode = (feat.kind == ParametricFeatureKind::SweepCut) ? geoalgo::SketchSweepMode::Cut
																	: geoalgo::SketchSweepMode::Boss;
			bool ok = false;
			if (!feat.pathSegments.empty())
			{
				std::vector<geoalgo::SketchSweepPathSegment> segs;
				segs.reserve(feat.pathSegments.size());
				for (const auto& ps : feat.pathSegments)
				{
					geoalgo::SketchSweepPathSegment g;
					g.kind = (ps.kind == 1)	 ? geoalgo::SketchSweepPathSegKind::Arc
							 : (ps.kind == 2) ? geoalgo::SketchSweepPathSegKind::SplineThrough
											 : geoalgo::SketchSweepPathSegKind::Line;
					g.ax = ps.ax;
					g.ay = ps.ay;
					g.az = ps.az;
					g.bx = ps.bx;
					g.by = ps.by;
					g.bz = ps.bz;
					g.mx = ps.mx;
					g.my = ps.my;
					g.mz = ps.mz;
					segs.push_back(g);
				}
				ok = geoalgo::sketchSweepSegmentsToHandle(profile, segs, sp, basePtr, next, &err);
			}
			else
			{
				if (path.size() < 6)
				{
					if (errMsg)
						*errMsg = "missing sweep path for " + feat.id;
					return false;
				}
				ok = geoalgo::sketchSweepPolylineToHandle(profile, path, sp, basePtr, next, &err);
			}
			if (!ok || next.isNull())
			{
				if (errMsg)
					*errMsg = err.empty() ? ("rebuild failed at " + feat.id) : err;
				return false;
			}
		}
		else
		{
			const ParametricFeature* sk = findSketchFor(feat);
			if (!sk || sk->profileXyzMm.size() < 12)
			{
				if (errMsg)
					*errMsg = "missing sketch profile for " + feat.id;
				return false;
			}
			geoalgo::SketchExtrudeParams ep;
			ep.mode = (feat.kind == ParametricFeatureKind::Pocket) ? geoalgo::SketchExtrudeMode::Pocket
																  : geoalgo::SketchExtrudeMode::Pad;
			ep.lengthMm = feat.lengthMm;
			ep.reversed = feat.reversed;
			ep.draftAngleDeg = feat.draftAngleDeg;
			ep.endCondition = toAlgoEnd(feat.endCondition);
			ep.originX = sk->plane.originX;
			ep.originY = sk->plane.originY;
			ep.originZ = sk->plane.originZ;
			ep.normalX = sk->plane.normalX;
			ep.normalY = sk->plane.normalY;
			ep.normalZ = sk->plane.normalZ;

			if (feat.endCondition == ParametricExtrudeEnd::UpToFace)
			{
				bool resolved = false;
				const bool selfRef = feat.upToFaceBackendId.empty() || feat.upToFaceBackendId == id();
				if (selfRef && feat.upToFaceIndex >= 0 && !tip.isNull())
				{
					geoalgo::SketchPlaneMm plane;
					if (geoalgo::queryPlanarFaceSketchPlane(tip, feat.upToFaceIndex, plane, nullptr) && plane.planar)
					{
						ep.hasUpToFace = true;
						ep.upOriginX = plane.ox;
						ep.upOriginY = plane.oy;
						ep.upOriginZ = plane.oz;
						ep.upNormalX = plane.nx;
						ep.upNormalY = plane.ny;
						ep.upNormalZ = plane.nz;
						resolved = true;
					}
				}
				if (!resolved)
					applyBakedUpToFace(feat, ep);
			}
			else
			{
				ep.hasUpToFace = false;
			}

			if (feat.kind == ParametricFeatureKind::Pocket && tip.isNull())
			{
				if (errMsg)
					*errMsg = "Pocket requires existing solid tip";
				return false;
			}
			if (!geoalgo::sketchExtrudePolylineToHandle(sk->profileXyzMm, ep, basePtr, next, &err) || next.isNull())
			{
				if (errMsg)
					*errMsg = err.empty() ? ("rebuild failed at " + feat.id) : err;
				return false;
			}
		}
		tip = std::move(next);
		geoalgo::mergeFaceOwnershipByTShape(tip, feat.id, tshapeOwners, m_faceOwnerByIndex);
	}

	if (tip.isNull())
	{
		clearGeometry();
		return true;
	}
	setShape(std::move(tip));
	return true;
}

nlohmann::json ParametricBrepBackendData::historyToJson() const
{
	nlohmann::json root;
	root["seq"] = m_seq;
	nlohmann::json arr = nlohmann::json::array();
	for (const auto& f : m_features)
		arr.push_back(parametricFeatureToJson(f));
	root["features"] = std::move(arr);
	return root;
}

bool ParametricBrepBackendData::historyFromJson(const nlohmann::json& in, std::string* errMsg)
{
	if (!in.is_object())
	{
		if (errMsg)
			*errMsg = "parametricHistory must be object";
		return false;
	}
	std::vector<ParametricFeature> loaded;
	if (in.contains("features") && in["features"].is_array())
	{
		for (const auto& item : in["features"])
		{
			ParametricFeature f;
			if (!parametricFeatureFromJson(item, f))
			{
				if (errMsg)
					*errMsg = "invalid feature entry";
				return false;
			}
			loaded.push_back(std::move(f));
		}
	}
	setFeatures(std::move(loaded));
	if (in.contains("seq") && in["seq"].is_number_integer())
	{
		const int seq = in["seq"].get<int>();
		if (seq > m_seq)
			m_seq = seq;
	}
	return true;
}

void ParametricBrepBackendData::saveDerivedJson(nlohmann::json& out) const
{
	BrepBackendData::saveDerivedJson(out);
	out["parametricHistory"] = historyToJson();
}

bool ParametricBrepBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	if (!BrepBackendData::loadDerivedJson(in, errMsg))
		return false;
	if (!in.contains("parametricHistory"))
		return true;
	return historyFromJson(in["parametricHistory"], errMsg);
}

bool ParametricBrepBackendData::runParametricHistorySelfTest(std::string* errMsg)
{
	auto fail = [&](const char* msg) -> bool
	{
		if (errMsg)
			*errMsg = msg;
		return false;
	};

	ParametricBrepBackendData body;
	body.setName("SelfTestBody");
	ParametricSketchPlane plane;
	plane.isPlanar = true;
	const std::string sk0 = body.addSketch(plane, "S0");
	// XY 平面 40mm 方轮廓
	const std::vector<float> square = {0, 0, 0, 40, 0, 0, 40, 40, 0, 0, 40, 0, 0, 0, 0};
	if (!body.setProfile(sk0, square))
		return fail("setProfile sk0");
	const std::string padId = body.addPad(sk0, 10.0);
	if (!body.rebuild(errMsg) || body.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "pad rebuild");

	ParametricSketchPlane plane2 = plane;
	plane2.originZ = 10.0;
	const std::string sk1 = body.addSketch(plane2, "S1");
	const std::vector<float> pocketProf = {10, 10, 10, 30, 10, 10, 30, 30, 10, 10, 30, 10, 10, 10, 10};
	if (!body.setProfile(sk1, pocketProf))
		return fail("setProfile sk1");
	body.addPocket(sk1, 5.0);
	if (!body.rebuild(errMsg) || body.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "pocket rebuild");

	if (body.features().size() != 4)
		return fail("feature count != 4");
	if (!body.setLength(padId, 20.0))
		return fail("setLength");
	if (!body.rebuild(errMsg) || body.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "length rebuild");

	// Sweep Boss：YZ 平面路径 + XY 矩形轮廓
	ParametricSketchPlane pathPlane;
	pathPlane.isPlanar = true;
	pathPlane.originX = 0;
	pathPlane.originY = 0;
	pathPlane.originZ = 0;
	pathPlane.axisXX = 0;
	pathPlane.axisXY = 0;
	pathPlane.axisXZ = 1;
	pathPlane.axisYX = 0;
	pathPlane.axisYY = 1;
	pathPlane.axisYZ = 0;
	pathPlane.normalX = 1;
	pathPlane.normalY = 0;
	pathPlane.normalZ = 0;
	const std::string pathSk = body.addSketch(pathPlane, "Path");
	const std::vector<float> pathLine = {50, 20, 0, 50, 20, 40};
	if (!body.setProfile(pathSk, pathLine))
		return fail("setProfile path");
	const std::string sweepProfSk = body.addSketch(plane, "SweepProf");
	const std::vector<float> sweepProf = {45, 15, 0, 55, 15, 0, 55, 25, 0, 45, 25, 0, 45, 15, 0};
	if (!body.setProfile(sweepProfSk, sweepProf))
		return fail("setProfile sweepProf");
	const std::string sweepId = body.addSweep(sweepProfSk, pathSk, false);
	if (ParametricFeature* sw = body.findFeature(sweepId))
	{
		sw->profileXyzMm = sweepProf;
		sw->pathXyzMm = pathLine;
		ParametricFeature::PathSegment seg;
		seg.kind = 0;
		seg.ax = 50;
		seg.ay = 20;
		seg.az = 0;
		seg.bx = 50;
		seg.by = 20;
		seg.bz = 40;
		sw->pathSegments.push_back(seg);
	}
	if (!body.rebuild(errMsg) || body.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "sweep rebuild");

	const std::string cutId = body.addSweep(sweepProfSk, pathSk, true);
	if (ParametricFeature* cut = body.findFeature(cutId))
	{
		cut->profileXyzMm = {48, 18, 0, 52, 18, 0, 52, 22, 0, 48, 22, 0, 48, 18, 0};
		cut->pathXyzMm = pathLine;
		cut->pathSegments = body.findFeature(sweepId)->pathSegments;
	}
	if (!body.rebuild(errMsg) || body.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "sweepCut rebuild");

	nlohmann::json hist = body.historyToJson();
	bool foundSweep = false;
	bool foundCut = false;
	for (const auto& fo : hist["features"])
	{
		const std::string kind = fo.value("kind", std::string());
		if (kind == "Sweep")
		{
			foundSweep = true;
			if (fo.value("pathSketchRefId", std::string()).empty())
				return fail("Sweep missing pathSketchRefId");
		}
		if (kind == "SweepCut")
		{
			foundCut = true;
			if (fo.value("pathSketchRefId", std::string()).empty())
				return fail("SweepCut missing pathSketchRefId");
		}
	}
	if (!foundSweep || !foundCut)
		return fail("history missing Sweep/SweepCut");

	nlohmann::json wrapped;
	body.saveDerivedJson(wrapped);
	if (!wrapped.contains("parametricHistory"))
		return fail("saveDerivedJson missing parametricHistory");

	ParametricBrepBackendData loaded;
	if (!loaded.historyFromJson(hist, errMsg))
		return false;
	if (loaded.features().size() != body.features().size())
		return fail("history roundtrip size");
	if (!loaded.rebuild(errMsg) || loaded.worldShape().isNull())
		return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "loaded rebuild");
	if (loaded.className() != backend_type::kClassParametricBrep)
		return fail("className");

	// 对称 Pad + 拔模（与侧栏 MidPlane/draftAngleDeg 对齐）
	{
		ParametricBrepBackendData midBody;
		midBody.setName("SelfTestMidPlaneDraft");
		ParametricSketchPlane midPlane;
		midPlane.isPlanar = true;
		const std::string midSk = midBody.addSketch(midPlane, "MidSk");
		const std::vector<float> midSq = {0, 0, 0, 40, 0, 0, 40, 40, 0, 0, 40, 0, 0, 0, 0};
		if (!midBody.setProfile(midSk, midSq))
			return fail("mid setProfile");
		const std::string midPadId = midBody.addPad(midSk, 20.0);
		if (ParametricFeature* midFeat = midBody.findFeature(midPadId))
		{
			midFeat->endCondition = ParametricExtrudeEnd::MidPlane;
			midFeat->draftAngleDeg = 5.0;
		}
		else
			return fail("mid pad missing");
		if (!midBody.rebuild(errMsg) || midBody.worldShape().isNull())
			return fail(errMsg && !errMsg->empty() ? errMsg->c_str() : "midPlane draft rebuild");
		const auto bb = midBody.worldShape().boundingBoxMm();
		if (!bb.valid || std::abs(bb.maxZ - bb.minZ - 20.0) > 0.5)
			return fail("midPlane draft bbox Z span");
		if (std::abs(bb.minZ + bb.maxZ) > 1.0)
			return fail("midPlane draft not centered on sketch");
	}
	return true;
}
