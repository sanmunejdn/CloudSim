/// @file DesignPartsCatalog.cpp
/// @brief 标准件库加载与模板实例化

#include "Ai/DesignPartsCatalog.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPair>
#include <QRegularExpression>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <json.hpp>

namespace
{
using nlohmann::json;

QByteArray readFileBytes(const QString& path)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
		return {};
	return f.readAll();
}

double hexVertexRadius(double acrossFlatsMm)
{
	return acrossFlatsMm / std::sqrt(3.0);
}

json fillTemplate(const json& node, const json& vars)
{
	if (node.is_string())
	{
		const std::string s = node.get<std::string>();
		if (s.size() >= 4 && s.front() == '{' && s[1] == '{' && s.back() == '}' && s[s.size() - 2] == '}')
		{
			const std::string key = s.substr(2, s.size() - 4);
			// trim
			size_t a = 0, b = key.size();
			while (a < b && key[a] == ' ')
				++a;
			while (b > a && key[b - 1] == ' ')
				--b;
			const std::string k = key.substr(a, b - a);
			if (!vars.contains(k))
				throw std::runtime_error("missing template var: " + k);
			return vars.at(k);
		}
		return node;
	}
	if (node.is_array())
	{
		json out = json::array();
		for (const auto& x : node)
			out.push_back(fillTemplate(x, vars));
		return out;
	}
	if (node.is_object())
	{
		json out = json::object();
		for (auto it = node.begin(); it != node.end(); ++it)
			out[it.key()] = fillTemplate(it.value(), vars);
		return out;
	}
	return node;
}

DesignPartSpec parseSpec(const json& s)
{
	DesignPartSpec sp;
	if (s.contains("thread") && s["thread"].is_string())
		sp.thread = QString::fromStdString(s["thread"].get<std::string>());
	sp.dMm = s.value("d_mm", 0.0);
	sp.sMm = s.value("s_mm", 0.0);
	sp.kMm = s.value("k_mm", 0.0);
	sp.mMm = s.value("m_mm", 0.0);
	sp.dInnerMm = s.value("d_inner_mm", 0.0);
	sp.dOuterMm = s.value("d_outer_mm", 0.0);
	sp.thicknessMm = s.value("thickness_mm", 0.0);
	sp.Z = s.value("Z", 0);
	sp.bMm = s.value("b_mm", 0.0);
	sp.boreMm = s.value("bore_mm", 0.0);
	if (s.contains("length_series_mm") && s["length_series_mm"].is_array())
	{
		for (const auto& v : s["length_series_mm"])
			sp.lengthSeriesMm.push_back(v.get<double>());
	}
	return sp;
}
} // namespace

QString DesignPartsCatalog::resolvePartsRoot()
{
	const QByteArray env = qgetenv("CLOUDSIM_DESIGN_PARTS");
	if (!env.isEmpty())
	{
		const QString p = QString::fromLocal8Bit(env);
		if (QDir(p).exists())
			return p;
	}
	const QString app = QCoreApplication::applicationDirPath();
	const QStringList candidates = {
		app + QStringLiteral("/resource/design-parts"),
		app + QStringLiteral("/design-parts"),
		// 开发树：bin/x64(d) → 仓库 CloudSim/tools/design-calc/parts
		QDir(app + QStringLiteral("/../../CloudSim/tools/design-calc/parts")).absolutePath(),
		QDir(app + QStringLiteral("/../../../CloudSim/tools/design-calc/parts")).absolutePath(),
	};
	for (const QString& c : candidates)
	{
		if (QDir(c).exists() && QFileInfo::exists(c + QStringLiteral("/index.json")))
			return QDir(c).absolutePath();
		// index 可选：有任意 part.json 即可
		if (QDir(c).exists())
		{
			const QFileInfoList dirs = QDir(c).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);
			for (const QFileInfo& di : dirs)
			{
				if (QFileInfo::exists(di.absoluteFilePath() + QStringLiteral("/part.json")))
					return QDir(c).absolutePath();
			}
		}
	}
	return {};
}

bool DesignPartsCatalog::load(const QString& partsRoot, QString* err)
{
	m_parts.clear();
	m_root = partsRoot;
	if (partsRoot.isEmpty() || !QDir(partsRoot).exists())
	{
		if (err)
			*err = QStringLiteral("design-parts root missing.");
		return false;
	}
	const QFileInfoList dirs = QDir(partsRoot).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
	for (const QFileInfo& di : dirs)
	{
		const QString partPath = di.absoluteFilePath() + QStringLiteral("/part.json");
		const QByteArray partBytes = readFileBytes(partPath);
		if (partBytes.isEmpty())
			continue;
		json root;
		try
		{
			root = json::parse(partBytes.constData(), nullptr, true);
		}
		catch (...)
		{
			continue;
		}
		DesignPartInfo info;
		info.id = QString::fromStdString(root.value("id", ""));
		info.displayName = QString::fromStdString(root.value("display_name", ""));
		info.modelRef = QString::fromStdString(root.value("model_ref", "model.compose.json"));
		info.modelFidelity = QString::fromStdString(root.value("model_fidelity", "blank"));
		info.dirPath = di.absoluteFilePath();
		info.partJsonUtf8 = partBytes;
		if (root.contains("keywords") && root["keywords"].is_array())
		{
			for (const auto& k : root["keywords"])
				info.keywords.push_back(QString::fromStdString(k.get<std::string>()));
		}
		if (root.contains("specs") && root["specs"].is_array())
		{
			for (const auto& s : root["specs"])
				info.specs.push_back(parseSpec(s));
		}
		if (root.contains("defaults") && root["defaults"].is_object())
			info.defaultsJsonUtf8 = QByteArray::fromStdString(root["defaults"].dump());
		const QString modelPath = info.dirPath + QLatin1Char('/') + info.modelRef;
		info.modelComposeUtf8 = readFileBytes(modelPath);
		if (info.id.isEmpty() || info.modelComposeUtf8.isEmpty())
			continue;
		m_parts.push_back(std::move(info));
	}
	if (m_parts.isEmpty())
	{
		if (err)
			*err = QStringLiteral("no parts loaded under %1").arg(partsRoot);
		return false;
	}
	return true;
}

const DesignPartInfo* DesignPartsCatalog::findById(const QString& partId) const
{
	for (const DesignPartInfo& p : m_parts)
	{
		if (p.id == partId)
			return &p;
	}
	return nullptr;
}

QVector<const DesignPartInfo*> DesignPartsCatalog::search(const QString& query) const
{
	const QString q = query.trimmed().toLower();
	QVector<QPair<int, const DesignPartInfo*>> scored;
	for (const DesignPartInfo& p : m_parts)
	{
		int score = 0;
		const QString pid = p.id.toLower();
		if (pid == q)
			score += 10;
		for (const QString& kw : p.keywords)
		{
			const QString k = kw.toLower();
			if (!k.isEmpty() && q.contains(k))
				score += 5;
		}
		if (q.contains(QStringLiteral("螺栓")) && pid.contains(QStringLiteral("bolt")))
			score += 3;
		if (q.contains(QStringLiteral("螺母")) && pid.contains(QStringLiteral("nut")))
			score += 3;
		if (q.contains(QStringLiteral("垫")) && pid.contains(QStringLiteral("washer")))
			score += 3;
		if ((q.contains(QStringLiteral("销")) || q.contains(QStringLiteral("pin"))) && pid.startsWith(QStringLiteral("pin.")))
			score += 3;
		if (q.contains(QStringLiteral("齿轮")) && pid.startsWith(QStringLiteral("gear.")))
			score += 3;
		if (score > 0)
			scored.push_back({score, &p});
	}
	std::sort(scored.begin(), scored.end(),
			  [](const QPair<int, const DesignPartInfo*>& a, const QPair<int, const DesignPartInfo*>& b)
			  { return a.first > b.first; });
	QVector<const DesignPartInfo*> out;
	for (const auto& s : scored)
		out.push_back(s.second);
	return out;
}

bool DesignPartsCatalog::instantiate(const QString& partId, const QByteArray& paramsJsonUtf8, QByteArray* outPlanUtf8,
									 QString* err) const
{
	const DesignPartInfo* part = findById(partId);
	if (!part)
	{
		if (err)
			*err = QStringLiteral("unknown part_id: %1").arg(partId);
		return false;
	}
	json params = json::object();
	if (!paramsJsonUtf8.isEmpty())
	{
		try
		{
			params = json::parse(paramsJsonUtf8.constData(), nullptr, true);
		}
		catch (...)
		{
			if (err)
				*err = QStringLiteral("invalid params JSON");
			return false;
		}
	}
	json defaults = json::object();
	if (!part->defaultsJsonUtf8.isEmpty())
	{
		try
		{
			defaults = json::parse(part->defaultsJsonUtf8.constData(), nullptr, true);
		}
		catch (...)
		{
		}
	}
	json vars = defaults;
	for (auto it = params.begin(); it != params.end(); ++it)
		vars[it.key()] = it.value();

	// 合并 specs
	const DesignPartSpec* matched = nullptr;
	if (vars.contains("thread") && vars["thread"].is_string())
	{
		const QString th = QString::fromStdString(vars["thread"].get<std::string>());
		for (const DesignPartSpec& s : part->specs)
		{
			if (s.thread == th)
			{
				matched = &s;
				break;
			}
		}
	}
	else if (part->id.startsWith(QStringLiteral("pin.")) && vars.contains("d_mm"))
	{
		const double d = vars["d_mm"].get<double>();
		for (const DesignPartSpec& s : part->specs)
		{
			if (std::abs(s.dMm - d) < 1e-9)
			{
				matched = &s;
				break;
			}
		}
	}
	if (matched)
	{
		auto setIfMissing = [&](const char* key, double v)
		{
			if (v != 0.0 && !vars.contains(key))
				vars[key] = v;
		};
		if (!matched->thread.isEmpty() && !vars.contains("thread"))
			vars["thread"] = matched->thread.toStdString();
		setIfMissing("d_mm", matched->dMm);
		setIfMissing("s_mm", matched->sMm);
		setIfMissing("k_mm", matched->kMm);
		setIfMissing("m_mm", matched->mMm);
		setIfMissing("d_inner_mm", matched->dInnerMm);
		setIfMissing("d_outer_mm", matched->dOuterMm);
		setIfMissing("thickness_mm", matched->thicknessMm);
		if (matched->Z && !vars.contains("Z"))
			vars["Z"] = matched->Z;
		setIfMissing("b_mm", matched->bMm);
		setIfMissing("bore_mm", matched->boreMm);
	}

	if (part->id.contains(QStringLiteral("hex_bolt")) || part->id.contains(QStringLiteral("hex_nut")))
	{
		if (!vars.contains("s_mm"))
		{
			if (err)
				*err = QStringLiteral("missing s_mm / thread spec");
			return false;
		}
		vars["head_r_mm"] = hexVertexRadius(vars["s_mm"].get<double>());
	}
	if (part->id.startsWith(QStringLiteral("pin.")))
	{
		if (!vars.contains("d_mm") || !vars.contains("length_mm"))
		{
			if (err)
				*err = QStringLiteral("pin needs d_mm and length_mm");
			return false;
		}
		vars["radius_mm"] = vars["d_mm"].get<double>() * 0.5;
	}
	if (part->id.startsWith(QStringLiteral("gear.")))
	{
		if (!vars.contains("m_mm") || !vars.contains("Z"))
		{
			if (err)
				*err = QStringLiteral("gear needs m_mm and Z");
			return false;
		}
		const double m = vars["m_mm"].get<double>();
		const int Z = vars["Z"].get<int>();
		vars["d_mm"] = m * Z;
		vars["da_mm"] = m * (Z + 2);
		vars["radius_mm"] = vars["da_mm"].get<double>() * 0.5;
		if (!vars.contains("b_mm"))
			vars["b_mm"] = 10.0 * m;
	}

	if (part->id.contains(QStringLiteral("hex_bolt")))
	{
		// 单步旋转阶梯截面：螺杆 + 圆柱头（一体毛坯）。两步 Pad 易只留下六角头。
		const double d = vars.value("d_mm", 0.0);
		const double L = vars.value("length_mm", 0.0);
		const double k = vars.value("k_mm", 0.0);
		const double Rh = vars.value("head_r_mm", d);
		if (d <= 0.0 || L <= 0.0 || k <= 0.0 || Rh <= 0.0)
		{
			if (err)
				*err = QStringLiteral("bolt needs d_mm/length_mm/k_mm/head_r_mm");
			return false;
		}
		const double rShank = d * 0.5;
		nlohmann::json xyz = nlohmann::json::array();
		auto addPt = [&](double x, double y)
		{
			xyz.push_back(x);
			xyz.push_back(y);
			xyz.push_back(0.0);
		};
		// 截面在 XY：X=半径，Y=轴向；绕 +Y 旋转
		addPt(0.0, 0.0);
		addPt(rShank, 0.0);
		addPt(rShank, L);
		addPt(Rh, L);
		addPt(Rh, L + k);
		addPt(0.0, L + k);
		addPt(0.0, 0.0);

		nlohmann::json plan = nlohmann::json::object();
		plan["version"] = 2;
		plan["domain"] = "feature.compose";
		plan["meta"] = {{"part_id", part->id.toStdString()},
						{"blank_only", true},
						{"model_fidelity", "blank"},
						{"head_style", "cylinder_approx"},
						{"params", vars}};
		plan["steps"] = nlohmann::json::array();
		plan["steps"].push_back(
			{{"id", "bolt"},
			 {"api", "revolveSketchProfileToBrep"},
			 {"args",
			  {{"mode", "boss"},
			   {"profile_xyz_mm", xyz},
			   {"angle_deg", 360.0},
			   {"axis_dx", 0.0},
			   {"axis_dy", 1.0},
			   {"axis_dz", 0.0},
			   {"name", "HexBoltBlank"}}}});
		if (outPlanUtf8)
			*outPlanUtf8 = QByteArray::fromStdString(plan.dump());
		return true;
	}

	json tpl;
	try
	{
		tpl = json::parse(part->modelComposeUtf8.constData(), nullptr, true);
		json plan = fillTemplate(tpl, vars);
		plan["version"] = 2;
		plan["domain"] = "feature.compose";
		if (!plan.contains("meta") || !plan["meta"].is_object())
			plan["meta"] = json::object();
		plan["meta"]["part_id"] = part->id.toStdString();
		plan["meta"]["blank_only"] = true;
		plan["meta"]["model_fidelity"] = part->modelFidelity.toStdString();
		plan["meta"]["params"] = vars;
		if (outPlanUtf8)
			*outPlanUtf8 = QByteArray::fromStdString(plan.dump());
		return true;
	}
	catch (const std::exception& ex)
	{
		if (err)
			*err = QString::fromUtf8(ex.what());
		return false;
	}
}

bool DesignPartsCatalog::tryParseUserText(const QString& text, QByteArray* outPlanUtf8, QString* partId, QString* hint,
										  QString* err) const
{
	if (m_parts.isEmpty())
	{
		const_cast<DesignPartsCatalog*>(this)->load(resolvePartsRoot(), err);
		if (m_parts.isEmpty())
		{
			if (err && err->isEmpty())
				*err = QStringLiteral("标准件库未加载（检查 resource/design-parts 或 CLOUDSIM_DESIGN_PARTS）");
			return false;
		}
	}
	const auto hits = search(text);
	if (hits.isEmpty())
	{
		if (err)
			*err = QStringLiteral("未匹配到标准件");
		if (hint)
			*hint = QStringLiteral("可试：六角螺栓 M8×30、螺母 M8、垫圈 M8、销 d6 长20、齿轮 模数2 齿数20");
		return false;
	}
	const DesignPartInfo* part = hits.front();
	json params = json::object();

	QRegularExpression threadRe(QStringLiteral("\\bM\\s*(\\d+)\\b"), QRegularExpression::CaseInsensitiveOption);
	const auto tm = threadRe.match(text);
	if (tm.hasMatch())
		params["thread"] = QStringLiteral("M%1").arg(tm.captured(1)).toStdString();

	QRegularExpression lenRe(QStringLiteral("(?:长|长度|L\\s*[=:]?\\s*|x|×)\\s*(\\d+(?:\\.\\d+)?)"),
							 QRegularExpression::CaseInsensitiveOption);
	auto lm = lenRe.match(text);
	if (!lm.hasMatch())
	{
		QRegularExpression len2(QStringLiteral("[x×]\\s*(\\d+(?:\\.\\d+)?)"));
		lm = len2.match(text);
	}
	if (lm.hasMatch())
		params["length_mm"] = lm.captured(1).toDouble();

	if (part->id.startsWith(QStringLiteral("pin.")))
	{
		QRegularExpression dRe(QStringLiteral("(?:直径|d)\\s*[=:]?\\s*(\\d+(?:\\.\\d+)?)"),
							   QRegularExpression::CaseInsensitiveOption);
		const auto dm = dRe.match(text);
		if (dm.hasMatch())
			params["d_mm"] = dm.captured(1).toDouble();
	}
	if (part->id.startsWith(QStringLiteral("gear.")))
	{
		QRegularExpression mRe(QStringLiteral("(?:模数|m)\\s*[=:]?\\s*(\\d+(?:\\.\\d+)?)"),
							   QRegularExpression::CaseInsensitiveOption);
		QRegularExpression zRe(QStringLiteral("(?:齿数|Z)\\s*[=:]?\\s*(\\d+)"), QRegularExpression::CaseInsensitiveOption);
		const auto mm = mRe.match(text);
		const auto zm = zRe.match(text);
		if (mm.hasMatch())
			params["m_mm"] = mm.captured(1).toDouble();
		if (zm.hasMatch())
			params["Z"] = zm.captured(1).toInt();
		QRegularExpression bRe(QStringLiteral("(?:齿宽|b)\\s*[=:]?\\s*(\\d+(?:\\.\\d+)?)"),
							   QRegularExpression::CaseInsensitiveOption);
		const auto bm = bRe.match(text);
		if (bm.hasMatch())
			params["b_mm"] = bm.captured(1).toDouble();
	}

	QString iErr;
	if (!instantiate(part->id, QByteArray::fromStdString(params.dump()), outPlanUtf8, &iErr))
	{
		if (err)
			*err = iErr;
		return false;
	}
	if (partId)
		*partId = part->id;
	if (hint)
		*hint = QStringLiteral("已匹配标准件 %1（毛坯）").arg(part->displayName.isEmpty() ? part->id : part->displayName);
	return true;
}
