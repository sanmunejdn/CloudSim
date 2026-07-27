#include "Constraints.h"
#include "../Sketch.h"
#include "../SketchPoint.h"
#include "../SketchLine.h"
#include "../SketchArc.h"
#include "../SketchCircle.h"

#include <QJsonObject>
#include <QString>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>

namespace onecad::core::sketch::constraints {

namespace {

constexpr double kAngleEpsilon = 1e-6;

bool getPointPosition(const Sketch& sketch, const PointID& id, gp_Pnt2d& out) {
    auto* point = sketch.getEntityAs<SketchPoint>(id);
    if (!point) {
        return false;
    }
    out = point->position();
    return true;
}

bool getLinePoints(const Sketch& sketch, const EntityID& id, gp_Pnt2d& start, gp_Pnt2d& end) {
    auto* line = sketch.getEntityAs<SketchLine>(id);
    if (!line) {
        return false;
    }

    if (!getPointPosition(sketch, line->startPointId(), start)) {
        return false;
    }
    if (!getPointPosition(sketch, line->endPointId(), end)) {
        return false;
    }

    return true;
}

bool getArcCenter(const Sketch& sketch, const SketchArc& arc, gp_Pnt2d& center) {
    return getPointPosition(sketch, arc.centerPointId(), center);
}

bool getCircleCenter(const Sketch& sketch, const SketchCircle& circle, gp_Pnt2d& center) {
    return getPointPosition(sketch, circle.centerPointId(), center);
}

bool getCurveData(const Sketch& sketch, const EntityID& id, gp_Pnt2d& center, double& radius) {
    if (auto* arc = sketch.getEntityAs<SketchArc>(id)) {
        if (!getArcCenter(sketch, *arc, center)) {
            return false;
        }
        radius = arc->radius();
        return true;
    }

    if (auto* circle = sketch.getEntityAs<SketchCircle>(id)) {
        if (!getCircleCenter(sketch, *circle, center)) {
            return false;
        }
        radius = circle->radius();
        return true;
    }

    return false;
}

gp_Pnt2d midpoint(const gp_Pnt2d& a, const gp_Pnt2d& b) {
    return gp_Pnt2d((a.X() + b.X()) * 0.5, (a.Y() + b.Y()) * 0.5);
}

double normalizeAngle(double angle) {
    double twoPi = 2.0 * std::numbers::pi_v<double>;
    angle = std::fmod(angle, twoPi);
    if (angle <= -std::numbers::pi_v<double>) {
        angle += twoPi;
    } else if (angle > std::numbers::pi_v<double>) {
        angle -= twoPi;
    }
    return angle;
}

double angleDifference(double a, double b) {
    return std::abs(normalizeAngle(a - b));
}

double parallelAngleError(const gp_Pnt2d& s1, const gp_Pnt2d& e1,
                          const gp_Pnt2d& s2, const gp_Pnt2d& e2) {
    double a1 = SketchLine::angle(s1, e1);
    double a2 = SketchLine::angle(s2, e2);
    double diff = std::abs(normalizeAngle(a2 - a1));
    return std::min(diff, std::abs(std::numbers::pi_v<double> - diff));
}

double perpendicularAngleError(const gp_Pnt2d& s1, const gp_Pnt2d& e1,
                               const gp_Pnt2d& s2, const gp_Pnt2d& e2) {
    double a1 = SketchLine::angle(s1, e1);
    double a2 = SketchLine::angle(s2, e2);
    double diff = std::abs(normalizeAngle(a2 - a1));
    return std::abs(diff - std::numbers::pi_v<double> * 0.5);
}

double pointToLineDistance(const gp_Pnt2d& point, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    double dx = b.X() - a.X();
    double dy = b.Y() - a.Y();
    double length = std::hypot(dx, dy);
    if (length <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    double cross = (point.X() - a.X()) * dy - (point.Y() - a.Y()) * dx;
    return std::abs(cross) / length;
}

gp_Pnt2d projectPointToLine(const gp_Pnt2d& point, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    double dx = b.X() - a.X();
    double dy = b.Y() - a.Y();
    double denom = dx * dx + dy * dy;
    if (denom <= 0.0) {
        return a;
    }
    double t = ((point.X() - a.X()) * dx + (point.Y() - a.Y()) * dy) / denom;
    return gp_Pnt2d(a.X() + t * dx, a.Y() + t * dy);
}

gp_Pnt2d reflectPointAcrossLine(const gp_Pnt2d& point, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    gp_Pnt2d projection = projectPointToLine(point, a, b);
    return gp_Pnt2d(2.0 * projection.X() - point.X(),
                    2.0 * projection.Y() - point.Y());
}

std::string formatValue(double value, const std::string& units) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << value;
    if (!units.empty()) {
        stream << " " << units;
    }
    return stream.str();
}

} // namespace

//==============================================================================
// FixedConstraint
//==============================================================================

FixedConstraint::FixedConstraint(const PointID& pointId, double x, double y)
    : SketchConstraint(),
      m_pointId(pointId),
      m_fixedX(x),
      m_fixedY(y) {
}

std::string FixedConstraint::toString() const {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << "Fixed(" << m_fixedX << ", " << m_fixedY << ")";
    return stream.str();
}

std::vector<EntityID> FixedConstraint::referencedEntities() const {
    return {m_pointId};
}

bool FixedConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d pos;
    if (!getPointPosition(sketch, m_pointId, pos)) {
        return false;
    }
    gp_Pnt2d target(m_fixedX, m_fixedY);
    return pos.Distance(target) <= tolerance;
}

double FixedConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d pos;
    if (!getPointPosition(sketch, m_pointId, pos)) {
        return std::numeric_limits<double>::infinity();
    }
    gp_Pnt2d target(m_fixedX, m_fixedY);
    return pos.Distance(target);
}

void FixedConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point"] = QString::fromStdString(m_pointId);
    json["x"] = m_fixedX;
    json["y"] = m_fixedY;
}

bool FixedConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("point") || !json.contains("x") || !json.contains("y")) {
        return false;
    }
    if (!json["point"].isString() || !json["x"].isDouble() || !json["y"].isDouble()) {
        return false;
    }
    if (!deserializeBase(json, "Fixed")) {
        return false;
    }
    m_pointId = json["point"].toString().toStdString();
    m_fixedX = json["x"].toDouble();
    m_fixedY = json["y"].toDouble();
    return true;
}

gp_Pnt2d FixedConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d pos;
    if (!getPointPosition(sketch, m_pointId, pos)) {
        return gp_Pnt2d(m_fixedX, m_fixedY);
    }
    return pos;
}

void FixedConstraint::translate(double dx, double dy) {
    m_fixedX += dx;
    m_fixedY += dy;
}

//==============================================================================
// MidpointConstraint
//==============================================================================

MidpointConstraint::MidpointConstraint(const PointID& pointId, const EntityID& lineId)
    : SketchConstraint(),
      m_pointId(pointId),
      m_lineId(lineId) {
}

std::vector<EntityID> MidpointConstraint::referencedEntities() const {
    return {m_pointId, m_lineId};
}

bool MidpointConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d point;
    gp_Pnt2d lineStart;
    gp_Pnt2d lineEnd;
    if (!getPointPosition(sketch, m_pointId, point)) {
        return false;
    }
    if (!getLinePoints(sketch, m_lineId, lineStart, lineEnd)) {
        return false;
    }
    gp_Pnt2d mid = midpoint(lineStart, lineEnd);
    return point.Distance(mid) <= tolerance;
}

double MidpointConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d point;
    gp_Pnt2d lineStart;
    gp_Pnt2d lineEnd;
    if (!getPointPosition(sketch, m_pointId, point)) {
        return std::numeric_limits<double>::infinity();
    }
    if (!getLinePoints(sketch, m_lineId, lineStart, lineEnd)) {
        return std::numeric_limits<double>::infinity();
    }
    gp_Pnt2d mid = midpoint(lineStart, lineEnd);
    return point.Distance(mid);
}

void MidpointConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point"] = QString::fromStdString(m_pointId);
    json["line"] = QString::fromStdString(m_lineId);
}

bool MidpointConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("point") || !json.contains("line")) {
        return false;
    }
    if (!json["point"].isString() || !json["line"].isString()) {
        return false;
    }
    if (!deserializeBase(json, "Midpoint")) {
        return false;
    }
    m_pointId = json["point"].toString().toStdString();
    m_lineId = json["line"].toString().toStdString();
    return true;
}

gp_Pnt2d MidpointConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d lineStart;
    gp_Pnt2d lineEnd;
    if (!getLinePoints(sketch, m_lineId, lineStart, lineEnd)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(lineStart, lineEnd);
}

//==============================================================================
// CoincidentConstraint
//==============================================================================

CoincidentConstraint::CoincidentConstraint(const PointID& point1, const PointID& point2)
    : SketchConstraint(),
      m_point1(point1),
      m_point2(point2) {
}

std::vector<EntityID> CoincidentConstraint::referencedEntities() const {
    return {m_point1, m_point2};
}

bool CoincidentConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return false;
    }
    return p1.Distance(p2) <= tolerance;
}

double CoincidentConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return std::numeric_limits<double>::infinity();
    }
    return p1.Distance(p2);
}

void CoincidentConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point1"] = QString::fromStdString(m_point1);
    json["point2"] = QString::fromStdString(m_point2);
}

bool CoincidentConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("point1") || !json.contains("point2")) {
        return false;
    }

    if (!json["point1"].isString() || !json["point2"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Coincident")) {
        return false;
    }

    m_point1 = json["point1"].toString().toStdString();
    m_point2 = json["point2"].toString().toStdString();
    return true;
}

gp_Pnt2d CoincidentConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(p1, p2);
}

HorizontalConstraint::HorizontalConstraint(const EntityID& lineId)
    : SketchConstraint(),
      m_lineId(lineId) {
}

std::vector<EntityID> HorizontalConstraint::referencedEntities() const {
    return {m_lineId};
}

bool HorizontalConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return false;
    }
    return std::abs(start.Y() - end.Y()) <= tolerance;
}

double HorizontalConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(start.Y() - end.Y());
}

void HorizontalConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["line"] = QString::fromStdString(m_lineId);
}

bool HorizontalConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("line")) {
        return false;
    }

    if (!json["line"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Horizontal")) {
        return false;
    }

    m_lineId = json["line"].toString().toStdString();
    return true;
}

gp_Pnt2d HorizontalConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(start, end);
}

VerticalConstraint::VerticalConstraint(const EntityID& lineId)
    : SketchConstraint(),
      m_lineId(lineId) {
}

std::vector<EntityID> VerticalConstraint::referencedEntities() const {
    return {m_lineId};
}

bool VerticalConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return false;
    }
    return std::abs(start.X() - end.X()) <= tolerance;
}

double VerticalConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(start.X() - end.X());
}

void VerticalConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["line"] = QString::fromStdString(m_lineId);
}

bool VerticalConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("line")) {
        return false;
    }

    if (!json["line"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Vertical")) {
        return false;
    }

    m_lineId = json["line"].toString().toStdString();
    return true;
}

gp_Pnt2d VerticalConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d start;
    gp_Pnt2d end;
    if (!getLinePoints(sketch, m_lineId, start, end)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(start, end);
}

ParallelConstraint::ParallelConstraint(const EntityID& line1, const EntityID& line2)
    : SketchConstraint(),
      m_line1(line1),
      m_line2(line2) {
}

std::vector<EntityID> ParallelConstraint::referencedEntities() const {
    return {m_line1, m_line2};
}

bool ParallelConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    if (!getLinePoints(sketch, m_line1, s1, e1) || !getLinePoints(sketch, m_line2, s2, e2)) {
        return false;
    }
    return parallelAngleError(s1, e1, s2, e2) <= tolerance;
}

double ParallelConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    if (!getLinePoints(sketch, m_line1, s1, e1) || !getLinePoints(sketch, m_line2, s2, e2)) {
        return std::numeric_limits<double>::infinity();
    }
    return parallelAngleError(s1, e1, s2, e2);
}

void ParallelConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["line1"] = QString::fromStdString(m_line1);
    json["line2"] = QString::fromStdString(m_line2);
}

bool ParallelConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("line1") || !json.contains("line2")) {
        return false;
    }

    if (!json["line1"].isString() || !json["line2"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Parallel")) {
        return false;
    }

    m_line1 = json["line1"].toString().toStdString();
    m_line2 = json["line2"].toString().toStdString();
    return true;
}

gp_Pnt2d ParallelConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    if (!getLinePoints(sketch, m_line1, s1, e1)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(s1, e1);
}

PerpendicularConstraint::PerpendicularConstraint(const EntityID& line1, const EntityID& line2)
    : SketchConstraint(),
      m_line1(line1),
      m_line2(line2) {
}

std::vector<EntityID> PerpendicularConstraint::referencedEntities() const {
    return {m_line1, m_line2};
}

bool PerpendicularConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    if (!getLinePoints(sketch, m_line1, s1, e1) || !getLinePoints(sketch, m_line2, s2, e2)) {
        return false;
    }
    return perpendicularAngleError(s1, e1, s2, e2) <= tolerance;
}

double PerpendicularConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    if (!getLinePoints(sketch, m_line1, s1, e1) || !getLinePoints(sketch, m_line2, s2, e2)) {
        return std::numeric_limits<double>::infinity();
    }
    return perpendicularAngleError(s1, e1, s2, e2);
}

void PerpendicularConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["line1"] = QString::fromStdString(m_line1);
    json["line2"] = QString::fromStdString(m_line2);
}

bool PerpendicularConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("line1") || !json.contains("line2")) {
        return false;
    }

    if (!json["line1"].isString() || !json["line2"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Perpendicular")) {
        return false;
    }

    m_line1 = json["line1"].toString().toStdString();
    m_line2 = json["line2"].toString().toStdString();
    return true;
}

gp_Pnt2d PerpendicularConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    if (!getLinePoints(sketch, m_line1, s1, e1)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(s1, e1);
}

TangentConstraint::TangentConstraint(const EntityID& entity1, const EntityID& entity2)
    : SketchConstraint(),
      m_entity1(entity1),
      m_entity2(entity2) {
}

std::vector<EntityID> TangentConstraint::referencedEntities() const {
    return {m_entity1, m_entity2};
}

bool TangentConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double TangentConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d line1Start;
    gp_Pnt2d line1End;
    gp_Pnt2d line2Start;
    gp_Pnt2d line2End;
    gp_Pnt2d center1;
    gp_Pnt2d center2;
    double radius1 = 0.0;
    double radius2 = 0.0;

    bool hasLine1 = getLinePoints(sketch, m_entity1, line1Start, line1End);
    bool hasLine2 = getLinePoints(sketch, m_entity2, line2Start, line2End);
    bool hasCurve1 = getCurveData(sketch, m_entity1, center1, radius1);
    bool hasCurve2 = getCurveData(sketch, m_entity2, center2, radius2);

    if (hasLine1 && hasCurve2) {
        return std::abs(pointToLineDistance(center2, line1Start, line1End) - radius2);
    }

    if (hasLine2 && hasCurve1) {
        return std::abs(pointToLineDistance(center1, line2Start, line2End) - radius1);
    }

    if (hasCurve1 && hasCurve2) {
        double centerDistance = center1.Distance(center2);
        double external = std::abs(centerDistance - (radius1 + radius2));
        double internal = std::abs(centerDistance - std::abs(radius1 - radius2));
        return std::min(external, internal);
    }

    return std::numeric_limits<double>::infinity();
}

void TangentConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity1"] = QString::fromStdString(m_entity1);
    json["entity2"] = QString::fromStdString(m_entity2);
}

bool TangentConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("entity1") || !json.contains("entity2")) {
        return false;
    }

    if (!json["entity1"].isString() || !json["entity2"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Tangent")) {
        return false;
    }

    m_entity1 = json["entity1"].toString().toStdString();
    m_entity2 = json["entity2"].toString().toStdString();
    return true;
}

gp_Pnt2d TangentConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d c1;
    gp_Pnt2d c2;
    double r1 = 0.0;
    double r2 = 0.0;

    if (getLinePoints(sketch, m_entity1, s1, e1) && getCurveData(sketch, m_entity2, c2, r2)) {
        return midpoint(midpoint(s1, e1), c2);
    }

    if (getLinePoints(sketch, m_entity2, s1, e1) && getCurveData(sketch, m_entity1, c1, r1)) {
        return midpoint(midpoint(s1, e1), c1);
    }

    if (getCurveData(sketch, m_entity1, c1, r1) && getCurveData(sketch, m_entity2, c2, r2)) {
        return midpoint(c1, c2);
    }

    return gp_Pnt2d(0.0, 0.0);
}

EqualConstraint::EqualConstraint(const EntityID& entity1, const EntityID& entity2)
    : SketchConstraint(),
      m_entity1(entity1),
      m_entity2(entity2) {
}

std::vector<EntityID> EqualConstraint::referencedEntities() const {
    return {m_entity1, m_entity2};
}

bool EqualConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double EqualConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    gp_Pnt2d c1;
    gp_Pnt2d c2;
    double r1 = 0.0;
    double r2 = 0.0;

    if (getLinePoints(sketch, m_entity1, s1, e1) && getLinePoints(sketch, m_entity2, s2, e2)) {
        return std::abs(SketchLine::length(s1, e1) - SketchLine::length(s2, e2));
    }

    bool hasCurve1 = getCurveData(sketch, m_entity1, c1, r1);
    bool hasCurve2 = getCurveData(sketch, m_entity2, c2, r2);
    if (hasCurve1 && hasCurve2) {
        return std::abs(r1 - r2);
    }

    return std::numeric_limits<double>::infinity();
}

void EqualConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity1"] = QString::fromStdString(m_entity1);
    json["entity2"] = QString::fromStdString(m_entity2);
}

bool EqualConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("entity1") || !json.contains("entity2")) {
        return false;
    }

    if (!json["entity1"].isString() || !json["entity2"].isString()) {
        return false;
    }

    if (!deserializeBase(json, "Equal")) {
        return false;
    }

    m_entity1 = json["entity1"].toString().toStdString();
    m_entity2 = json["entity2"].toString().toStdString();
    return true;
}

gp_Pnt2d EqualConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    if (getLinePoints(sketch, m_entity1, s1, e1)) {
        return midpoint(s1, e1);
    }

    gp_Pnt2d center;
    double radius = 0.0;
    if (getCurveData(sketch, m_entity1, center, radius)) {
        return center;
    }

    return gp_Pnt2d(0.0, 0.0);
}

DistanceConstraint::DistanceConstraint(const EntityID& entity1, const EntityID& entity2, double distance)
    : DimensionalConstraint(distance),
      m_entity1(entity1),
      m_entity2(entity2) {
}

std::string DistanceConstraint::toString() const {
    return "Distance: " + formatValue(value(), units());
}

std::vector<EntityID> DistanceConstraint::referencedEntities() const {
    return {m_entity1, m_entity2};
}

bool DistanceConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double DistanceConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;

    bool entity1IsPoint = getPointPosition(sketch, m_entity1, p1);
    bool entity2IsPoint = getPointPosition(sketch, m_entity2, p2);
    if (entity1IsPoint && entity2IsPoint) {
        return std::abs(p1.Distance(p2) - value());
    }

    bool entity1IsLine = getLinePoints(sketch, m_entity1, s1, e1);
    bool entity2IsLine = getLinePoints(sketch, m_entity2, s2, e2);

    if (entity1IsPoint && entity2IsLine) {
        return std::abs(pointToLineDistance(p1, s2, e2) - value());
    }

    if (entity2IsPoint && entity1IsLine) {
        return std::abs(pointToLineDistance(p2, s1, e1) - value());
    }

    if (entity1IsLine && entity2IsLine) {
        double angleError = parallelAngleError(s1, e1, s2, e2);
        if (angleError > kAngleEpsilon) {
            return std::numeric_limits<double>::infinity();
        }
        return std::abs(pointToLineDistance(s1, s2, e2) - value());
    }

    return std::numeric_limits<double>::infinity();
}

void DistanceConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity1"] = QString::fromStdString(m_entity1);
    json["entity2"] = QString::fromStdString(m_entity2);
    json["distance"] = value();
}

bool DistanceConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("entity1") || !json.contains("entity2") || !json.contains("distance")) {
        return false;
    }

    if (!json["entity1"].isString() || !json["entity2"].isString() || !json["distance"].isDouble()) {
        return false;
    }

    if (!deserializeBase(json, "Distance")) {
        return false;
    }

    m_entity1 = json["entity1"].toString().toStdString();
    m_entity2 = json["entity2"].toString().toStdString();
    setValue(json["distance"].toDouble());
    return true;
}

gp_Pnt2d DistanceConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;

    bool entity1IsPoint = getPointPosition(sketch, m_entity1, p1);
    bool entity2IsPoint = getPointPosition(sketch, m_entity2, p2);
    if (entity1IsPoint && entity2IsPoint) {
        return midpoint(p1, p2);
    }

    bool entity1IsLine = getLinePoints(sketch, m_entity1, s1, e1);
    bool entity2IsLine = getLinePoints(sketch, m_entity2, s2, e2);

    if (entity1IsPoint && entity2IsLine) {
        gp_Pnt2d proj = projectPointToLine(p1, s2, e2);
        return midpoint(p1, proj);
    }

    if (entity2IsPoint && entity1IsLine) {
        gp_Pnt2d proj = projectPointToLine(p2, s1, e1);
        return midpoint(p2, proj);
    }

    if (entity1IsLine && entity2IsLine) {
        return midpoint(midpoint(s1, e1), midpoint(s2, e2));
    }

    return gp_Pnt2d(0.0, 0.0);
}

gp_Pnt2d DistanceConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

//==============================================================================
// HorizontalDistanceConstraint
//==============================================================================

HorizontalDistanceConstraint::HorizontalDistanceConstraint(const EntityID& point1,
                                                           const EntityID& point2,
                                                           double distance)
    : DimensionalConstraint(distance),
      m_point1(point1),
      m_point2(point2) {
}

std::string HorizontalDistanceConstraint::toString() const {
    return "H-Distance: " + formatValue(value(), units());
}

std::vector<EntityID> HorizontalDistanceConstraint::referencedEntities() const {
    return {m_point1, m_point2};
}

bool HorizontalDistanceConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double HorizontalDistanceConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs((p2.X() - p1.X()) - value());
}

void HorizontalDistanceConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point1"] = QString::fromStdString(m_point1);
    json["point2"] = QString::fromStdString(m_point2);
    json["distance"] = value();
}

bool HorizontalDistanceConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("point1") || !json.contains("point2") || !json.contains("distance")) {
        return false;
    }
    if (!json["point1"].isString() || !json["point2"].isString() || !json["distance"].isDouble()) {
        return false;
    }
    if (!deserializeBase(json, "HorizontalDistance")) {
        return false;
    }
    m_point1 = json["point1"].toString().toStdString();
    m_point2 = json["point2"].toString().toStdString();
    setValue(json["distance"].toDouble());
    return true;
}

gp_Pnt2d HorizontalDistanceConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return gp_Pnt2d((p1.X() + p2.X()) * 0.5, p1.Y());
}

gp_Pnt2d HorizontalDistanceConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

//==============================================================================
// VerticalDistanceConstraint
//==============================================================================

VerticalDistanceConstraint::VerticalDistanceConstraint(const EntityID& point1,
                                                       const EntityID& point2,
                                                       double distance)
    : DimensionalConstraint(distance),
      m_point1(point1),
      m_point2(point2) {
}

std::string VerticalDistanceConstraint::toString() const {
    return "V-Distance: " + formatValue(value(), units());
}

std::vector<EntityID> VerticalDistanceConstraint::referencedEntities() const {
    return {m_point1, m_point2};
}

bool VerticalDistanceConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double VerticalDistanceConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs((p2.Y() - p1.Y()) - value());
}

void VerticalDistanceConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point1"] = QString::fromStdString(m_point1);
    json["point2"] = QString::fromStdString(m_point2);
    json["distance"] = value();
}

bool VerticalDistanceConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("point1") || !json.contains("point2") || !json.contains("distance")) {
        return false;
    }
    if (!json["point1"].isString() || !json["point2"].isString() || !json["distance"].isDouble()) {
        return false;
    }
    if (!deserializeBase(json, "VerticalDistance")) {
        return false;
    }
    m_point1 = json["point1"].toString().toStdString();
    m_point2 = json["point2"].toString().toStdString();
    setValue(json["distance"].toDouble());
    return true;
}

gp_Pnt2d VerticalDistanceConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    if (!getPointPosition(sketch, m_point1, p1) || !getPointPosition(sketch, m_point2, p2)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return gp_Pnt2d(p1.X(), (p1.Y() + p2.Y()) * 0.5);
}

gp_Pnt2d VerticalDistanceConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

AngleConstraint::AngleConstraint(const EntityID& line1, const EntityID& line2, double angleRadians)
    : DimensionalConstraint(angleRadians),
      m_line1(line1),
      m_line2(line2) {
}

std::string AngleConstraint::toString() const {
    return "Angle: " + formatValue(angleDegrees(), units());
}

std::vector<EntityID> AngleConstraint::referencedEntities() const {
    return {m_line1, m_line2};
}

bool AngleConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double AngleConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    gp_Pnt2d s2;
    gp_Pnt2d e2;
    if (!getLinePoints(sketch, m_line1, s1, e1) || !getLinePoints(sketch, m_line2, s2, e2)) {
        return std::numeric_limits<double>::infinity();
    }
    double a1 = SketchLine::angle(s1, e1);
    double a2 = SketchLine::angle(s2, e2);
    double current = normalizeAngle(a2 - a1);
    return angleDifference(current, value());
}

void AngleConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["line1"] = QString::fromStdString(m_line1);
    json["line2"] = QString::fromStdString(m_line2);
    json["angle"] = value();
}

bool AngleConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("line1") || !json.contains("line2") || !json.contains("angle")) {
        return false;
    }

    if (!json["line1"].isString() || !json["line2"].isString() || !json["angle"].isDouble()) {
        return false;
    }

    if (!deserializeBase(json, "Angle")) {
        return false;
    }

    m_line1 = json["line1"].toString().toStdString();
    m_line2 = json["line2"].toString().toStdString();
    setValue(json["angle"].toDouble());
    return true;
}

gp_Pnt2d AngleConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d s1;
    gp_Pnt2d e1;
    if (!getLinePoints(sketch, m_line1, s1, e1)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(s1, e1);
}

gp_Pnt2d AngleConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

double AngleConstraint::angleDegrees() const {
    return value() * 180.0 / std::numbers::pi_v<double>;
}

void AngleConstraint::setAngleDegrees(double deg) {
    setValue(deg * std::numbers::pi_v<double> / 180.0);
}

RadiusConstraint::RadiusConstraint(const EntityID& circleOrArc, double radius)
    : DimensionalConstraint(radius),
      m_entityId(circleOrArc) {
}

std::string RadiusConstraint::toString() const {
    return "Radius: " + formatValue(value(), units());
}

std::vector<EntityID> RadiusConstraint::referencedEntities() const {
    return {m_entityId};
}

bool RadiusConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double RadiusConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d center;
    double radius = 0.0;
    if (!getCurveData(sketch, m_entityId, center, radius)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(radius - value());
}

void RadiusConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity"] = QString::fromStdString(m_entityId);
    json["radius"] = value();
}

bool RadiusConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("entity") || !json.contains("radius")) {
        return false;
    }

    if (!json["entity"].isString() || !json["radius"].isDouble()) {
        return false;
    }

    if (!deserializeBase(json, "Radius")) {
        return false;
    }

    m_entityId = json["entity"].toString().toStdString();
    setValue(json["radius"].toDouble());
    return true;
}

gp_Pnt2d RadiusConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d center;
    double radius = 0.0;
    if (!getCurveData(sketch, m_entityId, center, radius)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return gp_Pnt2d(center.X() + radius, center.Y());
}

gp_Pnt2d RadiusConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

//==============================================================================
// DiameterConstraint
//==============================================================================

DiameterConstraint::DiameterConstraint(const EntityID& circleOrArc, double diameter)
    : DimensionalConstraint(diameter),
      m_entityId(circleOrArc) {
}

std::string DiameterConstraint::toString() const {
    return "Diameter: " + formatValue(value(), units());
}

std::vector<EntityID> DiameterConstraint::referencedEntities() const {
    return {m_entityId};
}

bool DiameterConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double DiameterConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d center;
    double radius = 0.0;
    if (!getCurveData(sketch, m_entityId, center, radius)) {
        return std::numeric_limits<double>::infinity();
    }
    // Diameter = 2 * radius
    return std::abs(2.0 * radius - value());
}

void DiameterConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity"] = QString::fromStdString(m_entityId);
    json["diameter"] = value();
}

bool DiameterConstraint::deserialize(const QJsonObject& json) {
    if (!json.contains("entity") || !json.contains("diameter")) {
        return false;
    }

    if (!json["entity"].isString() || !json["diameter"].isDouble()) {
        return false;
    }

    if (!deserializeBase(json, "Diameter")) {
        return false;
    }

    m_entityId = json["entity"].toString().toStdString();
    setValue(json["diameter"].toDouble());
    return true;
}

gp_Pnt2d DiameterConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d center;
    double radius = 0.0;
    if (!getCurveData(sketch, m_entityId, center, radius)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    // Position icon at right edge of circle
    return gp_Pnt2d(center.X() + radius, center.Y());
}

gp_Pnt2d DiameterConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

//==============================================================================
// ConcentricConstraint
//==============================================================================

ConcentricConstraint::ConcentricConstraint(const EntityID& entity1, const EntityID& entity2)
    : m_entity1(entity1), m_entity2(entity2) {
}

std::vector<EntityID> ConcentricConstraint::referencedEntities() const {
    return {m_entity1, m_entity2};
}

bool ConcentricConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double ConcentricConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d center1, center2;
    double radius1 = 0.0, radius2 = 0.0;

    if (!getCurveData(sketch, m_entity1, center1, radius1)) {
        return std::numeric_limits<double>::infinity();
    }
    if (!getCurveData(sketch, m_entity2, center2, radius2)) {
        return std::numeric_limits<double>::infinity();
    }

    // Error is distance between centers
    return center1.Distance(center2);
}

void ConcentricConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["entity1"] = QString::fromStdString(m_entity1);
    json["entity2"] = QString::fromStdString(m_entity2);
}

bool ConcentricConstraint::deserialize(const QJsonObject& json) {
    if (!deserializeBase(json, "Concentric")) {
        return false;
    }

    if (!json.contains("entity1") || !json.contains("entity2")) {
        return false;
    }

    if (!json["entity1"].isString() || !json["entity2"].isString()) {
        return false;
    }

    m_entity1 = json["entity1"].toString().toStdString();
    m_entity2 = json["entity2"].toString().toStdString();
    return true;
}

gp_Pnt2d ConcentricConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d center;
    double radius = 0.0;
    if (!getCurveData(sketch, m_entity1, center, radius)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return center;
}

//==============================================================================
// SymmetricConstraint
//==============================================================================

SymmetricConstraint::SymmetricConstraint(const EntityID& point1,
                                         const EntityID& point2,
                                         const EntityID& axisLine)
    : m_point1(point1),
      m_point2(point2),
      m_axisLine(axisLine) {
}

std::vector<EntityID> SymmetricConstraint::referencedEntities() const {
    return {m_point1, m_point2, m_axisLine};
}

bool SymmetricConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double SymmetricConstraint::getError(const Sketch& sketch) const {
    gp_Pnt2d p1;
    gp_Pnt2d p2;
    gp_Pnt2d axisStart;
    gp_Pnt2d axisEnd;
    if (!getPointPosition(sketch, m_point1, p1) ||
        !getPointPosition(sketch, m_point2, p2) ||
        !getLinePoints(sketch, m_axisLine, axisStart, axisEnd)) {
        return std::numeric_limits<double>::infinity();
    }
    if (axisStart.Distance(axisEnd) <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return reflectPointAcrossLine(p1, axisStart, axisEnd).Distance(p2);
}

void SymmetricConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["point1"] = QString::fromStdString(m_point1);
    json["point2"] = QString::fromStdString(m_point2);
    json["axisLine"] = QString::fromStdString(m_axisLine);
}

bool SymmetricConstraint::deserialize(const QJsonObject& json) {
    if (!deserializeBase(json, "Symmetric")) {
        return false;
    }
    if (!json.contains("point1") || !json.contains("point2") || !json.contains("axisLine")) {
        return false;
    }
    if (!json["point1"].isString() || !json["point2"].isString() || !json["axisLine"].isString()) {
        return false;
    }
    m_point1 = json["point1"].toString().toStdString();
    m_point2 = json["point2"].toString().toStdString();
    m_axisLine = json["axisLine"].toString().toStdString();
    return true;
}

gp_Pnt2d SymmetricConstraint::getIconPosition(const Sketch& sketch) const {
    gp_Pnt2d axisStart;
    gp_Pnt2d axisEnd;
    if (!getLinePoints(sketch, m_axisLine, axisStart, axisEnd)) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return midpoint(axisStart, axisEnd);
}

//==============================================================================
// PointOnCurveConstraint
//==============================================================================

PointOnCurveConstraint::PointOnCurveConstraint(const EntityID& pointId, const EntityID& curveId,
                                               CurvePosition position)
    : m_pointId(pointId), m_curveId(curveId), m_position(position) {
}

std::vector<EntityID> PointOnCurveConstraint::referencedEntities() const {
    return {m_pointId, m_curveId};
}

int PointOnCurveConstraint::degreesRemoved() const {
    // Start/End positions fully constrain the point (2 DOF)
    // Arbitrary position allows sliding along curve (1 DOF)
    return (m_position == CurvePosition::Arbitrary) ? 1 : 2;
}

std::string PointOnCurveConstraint::toString() const {
    std::string posStr;
    switch (m_position) {
        case CurvePosition::Start:
            posStr = "Start";
            break;
        case CurvePosition::End:
            posStr = "End";
            break;
        case CurvePosition::Arbitrary:
            posStr = "Arbitrary";
            break;
    }
    return "PointOnCurve(" + posStr + ")";
}

bool PointOnCurveConstraint::isSatisfied(const Sketch& sketch, double tolerance) const {
    return getError(sketch) <= tolerance;
}

double PointOnCurveConstraint::getError(const Sketch& sketch) const {
    auto* point = sketch.getEntityAs<SketchPoint>(m_pointId);
    if (!point) {
        return std::numeric_limits<double>::infinity();
    }

    gp_Pnt2d pointPos = point->position();
    auto* curve = sketch.getEntity(m_curveId);
    if (!curve) {
        return std::numeric_limits<double>::infinity();
    }

    // Check curve type and compute error based on position
    switch (curve->type()) {
        case EntityType::Arc: {
            auto* arc = sketch.getEntityAs<SketchArc>(m_curveId);
            auto* centerPt = sketch.getEntityAs<SketchPoint>(arc->centerPointId());
            if (!centerPt) return std::numeric_limits<double>::infinity();

            gp_Pnt2d centerPos = centerPt->position();

            if (m_position == CurvePosition::Start) {
                gp_Pnt2d startPos = arc->startPoint(centerPos);
                return pointPos.Distance(startPos);
            } else if (m_position == CurvePosition::End) {
                gp_Pnt2d endPos = arc->endPoint(centerPos);
                return pointPos.Distance(endPos);
            } else {
                // Arbitrary - point should lie on arc's circle
                double dist = pointPos.Distance(centerPos);
                return std::abs(dist - arc->radius());
            }
        }
        case EntityType::Circle: {
            auto* circle = sketch.getEntityAs<SketchCircle>(m_curveId);
            auto* centerPt = sketch.getEntityAs<SketchPoint>(circle->centerPointId());
            if (!centerPt) return std::numeric_limits<double>::infinity();

            gp_Pnt2d centerPos = centerPt->position();
            double dist = pointPos.Distance(centerPos);
            return std::abs(dist - circle->radius());
        }
        case EntityType::Line: {
            auto* line = sketch.getEntityAs<SketchLine>(m_curveId);
            auto* p1 = sketch.getEntityAs<SketchPoint>(line->startPointId());
            auto* p2 = sketch.getEntityAs<SketchPoint>(line->endPointId());
            if (!p1 || !p2) return std::numeric_limits<double>::infinity();

            gp_Pnt2d lineStart = p1->position();
            gp_Pnt2d lineEnd = p2->position();

            if (m_position == CurvePosition::Start) {
                return pointPos.Distance(lineStart);
            } else if (m_position == CurvePosition::End) {
                return pointPos.Distance(lineEnd);
            } else {
                // Arbitrary - distance from point to line
                gp_Vec2d lineVec(lineStart, lineEnd);
                gp_Vec2d pointVec(lineStart, pointPos);
                double lineLen = lineVec.Magnitude();
                if (lineLen < 1e-10) return pointVec.Magnitude();

                // Project point onto line
                double t = pointVec.Dot(lineVec) / (lineLen * lineLen);
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;

                gp_Pnt2d projection(lineStart.X() + t * lineVec.X(),
                                    lineStart.Y() + t * lineVec.Y());
                return pointPos.Distance(projection);
            }
        }
        default:
            return std::numeric_limits<double>::infinity();
    }
}

void PointOnCurveConstraint::serialize(QJsonObject& json) const {
    serializeBase(json);
    json["pointId"] = QString::fromStdString(m_pointId);
    json["curveId"] = QString::fromStdString(m_curveId);
    json["position"] = static_cast<int>(m_position);
}

bool PointOnCurveConstraint::deserialize(const QJsonObject& json) {
    if (!deserializeBase(json, "PointOnCurve")) {
        return false;
    }

    if (!json.contains("pointId") || !json.contains("curveId") || !json.contains("position")) {
        return false;
    }

    if (!json["pointId"].isString() || !json["curveId"].isString() || !json["position"].isDouble()) {
        return false;
    }

    m_pointId = json["pointId"].toString().toStdString();
    m_curveId = json["curveId"].toString().toStdString();
    m_position = static_cast<CurvePosition>(json["position"].toInt());
    return true;
}

gp_Pnt2d PointOnCurveConstraint::getIconPosition(const Sketch& sketch) const {
    auto* point = sketch.getEntityAs<SketchPoint>(m_pointId);
    if (!point) {
        return gp_Pnt2d(0.0, 0.0);
    }
    return point->position();
}

} // namespace onecad::core::sketch::constraints
