#include "SketchLine.h"

#include <QJsonObject>
#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSketchLine, "onecad.core.sketch.line")

SketchLine::SketchLine()
    : SketchEntity() {
}

SketchLine::SketchLine(const PointID& startPointId, const PointID& endPointId)
    : SketchEntity(),
      m_startPointId(startPointId),
      m_endPointId(endPointId) {
}

double SketchLine::length(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    return startPos.Distance(endPos);
}

gp_Vec2d SketchLine::direction(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    gp_Vec2d vec(startPos, endPos);
    double magnitude = vec.Magnitude();
    if (magnitude <= 0.0) {
        return gp_Vec2d(0.0, 0.0);
    }
    vec /= magnitude;
    return vec;
}

gp_Pnt2d SketchLine::midpoint(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    return gp_Pnt2d((startPos.X() + endPos.X()) * 0.5,
                    (startPos.Y() + endPos.Y()) * 0.5);
}

double SketchLine::angle(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    return std::atan2(endPos.Y() - startPos.Y(), endPos.X() - startPos.X());
}

bool SketchLine::isHorizontal(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                              double tolerance) {
    double ang = angle(startPos, endPos);
    double pi = std::numbers::pi_v<double>;
    return std::abs(ang) <= tolerance || std::abs(std::abs(ang) - pi) <= tolerance;
}

bool SketchLine::isVertical(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                            double tolerance) {
    double ang = angle(startPos, endPos);
    double halfPi = std::numbers::pi_v<double> * 0.5;
    return std::abs(std::abs(ang) - halfPi) <= tolerance;
}

double SketchLine::distanceToPoint(const gp_Pnt2d& point,
                                   const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    double vx = endPos.X() - startPos.X();
    double vy = endPos.Y() - startPos.Y();
    double wx = point.X() - startPos.X();
    double wy = point.Y() - startPos.Y();

    double c1 = vx * wx + vy * wy;
    if (c1 <= 0.0) {
        return std::hypot(point.X() - startPos.X(), point.Y() - startPos.Y());
    }

    double c2 = vx * vx + vy * vy;
    if (c2 <= c1) {
        return std::hypot(point.X() - endPos.X(), point.Y() - endPos.Y());
    }

    double t = c1 / c2;
    double projX = startPos.X() + t * vx;
    double projY = startPos.Y() + t * vy;
    return std::hypot(point.X() - projX, point.Y() - projY);
}

BoundingBox2d SketchLine::bounds() const {
    return {};
}

bool SketchLine::isNear(const gp_Pnt2d&, double) const {
    return false;
}

void SketchLine::serialize(QJsonObject& json) const {
    json["id"] = QString::fromStdString(m_id);
    json["type"] = QString::fromStdString(typeName());
    json["construction"] = m_isConstruction;
    json["referenceLocked"] = m_isReferenceLocked;
    json["start"] = QString::fromStdString(m_startPointId);
    json["end"] = QString::fromStdString(m_endPointId);
}

bool SketchLine::deserialize(const QJsonObject& json) {
    if (!json.contains("start") || !json.contains("end")) {
        return false;
    }

    if (!json["start"].isString() || !json["end"].isString()) {
        return false;
    }

    if (json.contains("id") && !json["id"].isString()) {
        return false;
    }

    if (json.contains("construction") && !json["construction"].isBool()) {
        return false;
    }
    if (json.contains("referenceLocked") && !json["referenceLocked"].isBool()) {
        qCWarning(logSketchLine) << "deserialize:invalid-referenceLocked-type";
        return false;
    }

    EntityID newId = json.contains("id")
                         ? json["id"].toString().toStdString()
                         : generateId();
    bool newConstruction = json.contains("construction")
                               ? json["construction"].toBool()
                               : m_isConstruction;
    bool newReferenceLocked = json.contains("referenceLocked")
                                  ? json["referenceLocked"].toBool()
                                  : m_isReferenceLocked;
    PointID startId = json["start"].toString().toStdString();
    PointID endId = json["end"].toString().toStdString();

    m_id = std::move(newId);
    m_isConstruction = newConstruction;
    m_isReferenceLocked = newReferenceLocked;
    m_startPointId = std::move(startId);
    m_endPointId = std::move(endId);
    qCDebug(logSketchLine) << "deserialize:done"
                           << "id=" << QString::fromStdString(m_id)
                           << "referenceLocked=" << m_isReferenceLocked
                           << "start=" << QString::fromStdString(m_startPointId)
                           << "end=" << QString::fromStdString(m_endPointId);
    return true;
}

BoundingBox2d SketchLine::boundsWithPoints(const gp_Pnt2d& startPos, const gp_Pnt2d& endPos) {
    BoundingBox2d box;
    box.minX = std::min(startPos.X(), endPos.X());
    box.maxX = std::max(startPos.X(), endPos.X());
    box.minY = std::min(startPos.Y(), endPos.Y());
    box.maxY = std::max(startPos.Y(), endPos.Y());
    return box;
}

bool SketchLine::isNearWithPoints(const gp_Pnt2d& testPoint,
                                  const gp_Pnt2d& startPos, const gp_Pnt2d& endPos,
                                  double tolerance) {
    return distanceToPoint(testPoint, startPos, endPos) <= tolerance;
}

} // namespace onecad::core::sketch
