"""JSON 全量导入导出 + CSV bug 导入导出。"""
from __future__ import annotations

import csv
import io
from datetime import datetime

from sqlalchemy import DateTime, select
from sqlalchemy.orm import Session

from .. import models as m

# 导出顺序 = 依赖顺序；导入按此插入，清空按逆序
TABLES = [
    m.User, m.Project, m.Module, m.Milestone, m.Label, m.Bug, m.BugLabel,
    m.Comment, m.Attachment, m.AuditLog, m.BugWatcher, m.CommitLink,
    m.TestLink, m.Notification, m.SavedFilter, m.Setting,
]

CSV_FIELDS = [
    "id", "project_key", "module", "milestone", "title", "status", "resolution",
    "severity", "priority", "environment", "reporter", "assignee",
    "version_found", "version_fixed", "created_at", "closed_at",
    "description", "repro_steps", "expected", "actual",
]


def _row_dict(row) -> dict:
    data = {}
    for col in row.__table__.columns:
        value = getattr(row, col.name)
        if isinstance(value, datetime):
            value = value.isoformat()
        data[col.name] = value
    return data


def _make_row(model, data: dict):
    kwargs = {}
    for col in model.__table__.columns:
        value = data.get(col.name)
        if value is not None and isinstance(col.type, DateTime) \
                and isinstance(value, str):
            value = datetime.fromisoformat(value)
        kwargs[col.name] = value
    return model(**kwargs)


def export_json(sess: Session) -> dict:
    payload = {"version": 1, "exported_at": m.utcnow().isoformat(),
               "tables": {}}
    for model in TABLES:
        rows = sess.scalars(select(model)).all()
        payload["tables"][model.__tablename__] = [_row_dict(r) for r in rows]
    return payload


def import_json(sess: Session, payload: dict) -> dict:
    """全量导入：清空后按依赖顺序重建。调用方负责权限与确认。"""
    tables = payload.get("tables")
    if not isinstance(tables, dict):
        raise ValueError("导入文件格式错误：缺少 tables")

    for model in reversed(TABLES):
        sess.execute(model.__table__.delete())
    # 清空 identity map，避免新插入行与缓存中的旧对象主键冲突
    sess.expunge_all()

    counts: dict[str, int] = {}
    for model in TABLES:
        rows = tables.get(model.__tablename__, [])
        if model is m.Module:
            # 自引用外键两轮插入：先置空 parent_id，再回填
            pending = {row["id"]: row.get("parent_id") for row in rows}
            for row in rows:
                sess.add(_make_row(model, dict(row, parent_id=None)))
            sess.flush()
            for module_id, parent_id in pending.items():
                if parent_id is not None:
                    module = sess.get(m.Module, module_id)
                    if module is not None:
                        module.parent_id = parent_id
        else:
            for row in rows:
                sess.add(_make_row(model, row))
        # 逐表提交，保证插入顺序严格等于 TABLES 的依赖顺序
        sess.flush()
        counts[model.__tablename__] = len(rows)
    return counts


def export_bugs_csv(sess: Session, bugs: list[m.Bug]) -> str:
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=CSV_FIELDS)
    writer.writeheader()
    for bug in bugs:
        project = sess.get(m.Project, bug.project_id)
        module = sess.get(m.Module, bug.module_id) if bug.module_id else None
        milestone = sess.get(m.Milestone, bug.milestone_id) if bug.milestone_id else None
        writer.writerow({
            "id": bug.id,
            "project_key": project.key if project else "",
            "module": module.name if module else "",
            "milestone": milestone.name if milestone else "",
            "title": bug.title,
            "status": bug.status,
            "resolution": bug.resolution or "",
            "severity": bug.severity,
            "priority": bug.priority,
            "environment": bug.environment,
            "reporter": bug.reporter.username if bug.reporter else "",
            "assignee": bug.assignee.username if bug.assignee else "",
            "version_found": bug.version_found,
            "version_fixed": bug.version_fixed,
            "created_at": bug.created_at.isoformat() if bug.created_at else "",
            "closed_at": bug.closed_at.isoformat() if bug.closed_at else "",
            "description": bug.description,
            "repro_steps": bug.repro_steps,
            "expected": bug.expected,
            "actual": bug.actual,
        })
    return buf.getvalue()


def import_bugs_csv(sess: Session, text: str, project_id: int,
                    reporter_id: int) -> int:
    """CSV 导入 bug：一律以「新建」状态进入指定项目，返回导入条数。"""
    reader = csv.DictReader(io.StringIO(text))
    count = 0
    for row in reader:
        title = (row.get("title") or "").strip()
        if not title:
            continue
        severity = (row.get("severity") or "major").strip()
        priority = (row.get("priority") or "P2").strip()
        environment = (row.get("environment") or "debug").strip()
        sess.add(m.Bug(
            project_id=project_id, title=title,
            description=row.get("description", "") or "",
            repro_steps=row.get("repro_steps", "") or "",
            expected=row.get("expected", "") or "",
            actual=row.get("actual", "") or "",
            severity=severity if severity in m.SEVERITIES else "major",
            priority=priority if priority in m.PRIORITIES else "P2",
            environment=environment if environment in m.ENVIRONMENTS else "debug",
            version_found=row.get("version_found", "") or "",
            reporter_id=reporter_id, status="new",
        ))
        count += 1
    sess.flush()
    return count
