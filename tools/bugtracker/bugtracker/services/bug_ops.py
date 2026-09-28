"""Bug 序列化与组合查询 —— API 与 Agent 通道（MCP/CLI）共用。"""
from __future__ import annotations

from datetime import datetime

from fastapi import HTTPException
from sqlalchemy import func, or_, select
from sqlalchemy.orm import Session

from ..models import (Bug, BugLabel, Comment, CommitLink, Milestone, Module,
                      Project, TestLink, User)


def get_bug_or_404(sess: Session, bug_id: int) -> Bug:
    bug = sess.get(Bug, bug_id)
    if bug is None:
        raise HTTPException(404, f"bug #{bug_id} 不存在")
    return bug


def bug_dict(sess: Session, bug: Bug) -> dict:
    project = sess.get(Project, bug.project_id)
    module = sess.get(Module, bug.module_id) if bug.module_id else None
    milestone = sess.get(Milestone, bug.milestone_id) if bug.milestone_id else None
    return {
        "id": bug.id,
        "project_id": bug.project_id,
        "project_key": project.key if project else "",
        "module_id": bug.module_id,
        "module_name": module.name if module else "",
        "milestone_id": bug.milestone_id,
        "milestone_name": milestone.name if milestone else "",
        "title": bug.title,
        "description": bug.description,
        "repro_steps": bug.repro_steps,
        "expected": bug.expected,
        "actual": bug.actual,
        "severity": bug.severity,
        "priority": bug.priority,
        "status": bug.status,
        "resolution": bug.resolution,
        "environment": bug.environment,
        "version_found": bug.version_found,
        "version_fixed": bug.version_fixed,
        "reporter_id": bug.reporter_id,
        "reporter_name": bug.reporter.username if bug.reporter else "",
        "assignee_id": bug.assignee_id,
        "assignee_name": bug.assignee.username if bug.assignee else "",
        "labels": [{"id": lb.id, "name": lb.name, "color": lb.color}
                   for lb in bug.labels],
        "created_at": bug.created_at.isoformat() if bug.created_at else None,
        "updated_at": bug.updated_at.isoformat() if bug.updated_at else None,
        "closed_at": bug.closed_at.isoformat() if bug.closed_at else None,
    }


def bug_detail(sess: Session, bug: Bug) -> dict:
    """完整详情：字段 + 评论 + 审计历史 + commit/回归用例关联。"""
    from ..models import AuditLog
    data = bug_dict(sess, bug)
    data["comments"] = [
        {"id": c.id, "username": c.user.username if c.user else "",
         "body": c.body,
         "created_at": c.created_at.isoformat() if c.created_at else None}
        for c in sess.scalars(select(Comment).where(Comment.bug_id == bug.id)
                              .order_by(Comment.id)).all()]
    data["history"] = [
        {"id": h.id, "field": h.field, "old_value": h.old_value,
         "new_value": h.new_value,
         "username": h.user.username if h.user else "",
         "created_at": h.created_at.isoformat() if h.created_at else None}
        for h in sess.scalars(select(AuditLog).where(AuditLog.bug_id == bug.id)
                              .order_by(AuditLog.id.desc())).all()]
    data["commits"] = [
        {"id": c.id, "repo": c.repo, "commit_hash": c.commit_hash,
         "message": c.message,
         "created_at": c.created_at.isoformat() if c.created_at else None}
        for c in sess.scalars(select(CommitLink)
                              .where(CommitLink.bug_id == bug.id)
                              .order_by(CommitLink.id)).all()]
    data["tests"] = [
        {"id": t.id, "test_path": t.test_path, "note": t.note,
         "created_at": t.created_at.isoformat() if t.created_at else None}
        for t in sess.scalars(select(TestLink)
                              .where(TestLink.bug_id == bug.id)
                              .order_by(TestLink.id)).all()]
    return data


def build_query(sess: Session, params: dict):
    """多条件组合查询（列表 / CSV 导出 / Agent 通道共用）。"""
    stmt = select(Bug)
    if params.get("status"):
        values = [s.strip() for s in params["status"].split(",") if s.strip()]
        if values:
            stmt = stmt.where(Bug.status.in_(values))
    if params.get("severity"):
        values = [s.strip() for s in params["severity"].split(",") if s.strip()]
        if values:
            stmt = stmt.where(Bug.severity.in_(values))
    if params.get("priority"):
        values = [s.strip() for s in params["priority"].split(",") if s.strip()]
        if values:
            stmt = stmt.where(Bug.priority.in_(values))
    for field, column in (("project_id", Bug.project_id),
                          ("module_id", Bug.module_id),
                          ("milestone_id", Bug.milestone_id),
                          ("assignee_id", Bug.assignee_id),
                          ("reporter_id", Bug.reporter_id)):
        if params.get(field) is not None:
            stmt = stmt.where(column == params[field])
    if params.get("label_id") is not None:
        stmt = stmt.join(BugLabel, BugLabel.bug_id == Bug.id)\
                   .where(BugLabel.label_id == params["label_id"])
    if params.get("keyword"):
        kw = f"%{params['keyword']}%"
        stmt = stmt.where(or_(Bug.title.like(kw), Bug.description.like(kw),
                              Bug.repro_steps.like(kw)))
    if params.get("created_from"):
        stmt = stmt.where(
            Bug.created_at >= datetime.fromisoformat(params["created_from"]))
    if params.get("created_to"):
        stmt = stmt.where(
            Bug.created_at <= datetime.fromisoformat(params["created_to"]))
    return stmt


def paged(sess: Session, stmt, page: int, size: int) -> dict:
    total = sess.scalar(select(func.count()).select_from(stmt.subquery()))
    rows = sess.scalars(stmt.order_by(Bug.id.desc())
                        .offset((page - 1) * size).limit(size)).all()
    return {"items": [bug_dict(sess, b) for b in rows],
            "total": total, "page": page, "size": size}
