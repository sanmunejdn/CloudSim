"""Bug：CRUD / 状态流转 / 审计历史 / 关注 / commit 与回归用例关联 / 多条件检索。"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy import asc, desc, func, select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import (AuditLog, Bug, BugWatcher, CommitLink, Label, Milestone,
                      Module, Project, TestLink, User)
from ..schemas import (BugCreate, BugPatch, CommitLinkIn, StatusChange,
                       TestLinkIn)
from ..security import ROLE_RANK, current_user, require
from ..services import workflow
from ..services.bug_ops import (bug_detail, bug_dict, build_query,
                                get_bug_or_404)
from ..services.notify import notify_bug_change

router = APIRouter(prefix="/api/bugs", tags=["bugs"])

SORTABLE = {
    "id": Bug.id, "created_at": Bug.created_at, "updated_at": Bug.updated_at,
    "priority": Bug.priority, "severity": Bug.severity, "status": Bug.status,
}


@router.get("")
def list_bugs(
    sess: Session = Depends(get_db),
    user: User = Depends(current_user),
    status: str | None = None,
    severity: str | None = None,
    priority: str | None = None,
    project_id: int | None = None,
    module_id: int | None = None,
    milestone_id: int | None = None,
    assignee_id: int | None = None,
    reporter_id: int | None = None,
    label_id: int | None = None,
    keyword: str | None = None,
    created_from: str | None = None,
    created_to: str | None = None,
    sort: str = "created_at",
    order: str = "desc",
    page: int = Query(default=1, ge=1),
    size: int = Query(default=20, ge=1, le=200),
):
    params = dict(status=status, severity=severity, priority=priority,
                  project_id=project_id, module_id=module_id,
                  milestone_id=milestone_id, assignee_id=assignee_id,
                  reporter_id=reporter_id, label_id=label_id,
                  keyword=keyword, created_from=created_from,
                  created_to=created_to)
    stmt = build_query(sess, params)
    total = sess.scalar(select(func.count()).select_from(stmt.subquery()))
    sort_col = SORTABLE.get(sort, Bug.created_at)
    stmt = stmt.order_by(desc(sort_col) if order == "desc" else asc(sort_col))
    rows = sess.scalars(stmt.offset((page - 1) * size).limit(size)).all()
    return {"items": [bug_dict(sess, b) for b in rows], "total": total,
            "page": page, "size": size}


@router.post("", status_code=201)
def create_bug(body: BugCreate, sess: Session = Depends(get_db),
               user: User = Depends(require("tester"))):
    workflow.validate_enums(body.severity, body.priority, body.environment)
    if sess.get(Project, body.project_id) is None:
        raise HTTPException(422, "项目不存在")
    if body.module_id is not None:
        module = sess.get(Module, body.module_id)
        if module is None or module.project_id != body.project_id:
            raise HTTPException(422, "模块不存在或不属于该项目")
    if body.milestone_id is not None:
        milestone = sess.get(Milestone, body.milestone_id)
        if milestone is None or milestone.project_id != body.project_id:
            raise HTTPException(422, "里程碑不存在或不属于该项目")
    if body.assignee_id is not None:
        assignee = sess.get(User, body.assignee_id)
        if assignee is None or not assignee.active:
            raise HTTPException(422, "指派人不存在或已停用")

    bug = Bug(project_id=body.project_id, module_id=body.module_id,
              milestone_id=body.milestone_id, title=body.title,
              description=body.description, repro_steps=body.repro_steps,
              expected=body.expected, actual=body.actual,
              severity=body.severity, priority=body.priority,
              environment=body.environment, version_found=body.version_found,
              reporter_id=user.id, assignee_id=body.assignee_id,
              status="new")
    sess.add(bug)
    sess.flush()
    _set_labels(sess, bug, body.label_ids)
    workflow.audit(sess, bug.id, user.id, "created", "", bug.title)
    sess.flush()
    notify_bug_change(sess, bug, user, "创建了 bug")
    return bug_dict(sess, bug)


def _set_labels(sess: Session, bug: Bug, label_ids: list[int]) -> None:
    if not label_ids:
        return
    labels = sess.scalars(select(Label).where(Label.id.in_(label_ids))).all()
    bug.labels = list(labels)
    sess.flush()


@router.get("/{bug_id}")
def get_bug(bug_id: int, sess: Session = Depends(get_db),
            user: User = Depends(current_user)):
    bug = get_bug_or_404(sess, bug_id)
    data = bug_detail(sess, bug)
    data["watching"] = sess.get(BugWatcher, (bug_id, user.id)) is not None
    return data


@router.patch("/{bug_id}")
def patch_bug(bug_id: int, body: BugPatch, sess: Session = Depends(get_db),
              user: User = Depends(current_user)):
    bug = get_bug_or_404(sess, bug_id)
    is_reporter = bug.reporter_id == user.id
    if ROLE_RANK.get(user.role, -1) < ROLE_RANK["dev"] and not is_reporter:
        raise HTTPException(403, "仅 dev 及以上或报告人可修改")
    workflow.validate_enums(body.severity, body.priority, body.environment)

    changes = body.model_dump(exclude_unset=True, exclude={"label_ids"})
    if "assignee_id" in changes and changes["assignee_id"] is not None:
        assignee = sess.get(User, changes["assignee_id"])
        if assignee is None or not assignee.active:
            raise HTTPException(422, "指派人不存在或已停用")
    if "module_id" in changes and changes["module_id"] is not None:
        module = sess.get(Module, changes["module_id"])
        if module is None or module.project_id != bug.project_id:
            raise HTTPException(422, "模块不存在或不属于该项目")
    if "milestone_id" in changes and changes["milestone_id"] is not None:
        milestone = sess.get(Milestone, changes["milestone_id"])
        if milestone is None or milestone.project_id != bug.project_id:
            raise HTTPException(422, "里程碑不存在或不属于该项目")

    changed = workflow.update_fields(sess, bug, changes, user)

    if body.label_ids is not None:
        old = sorted(lb.id for lb in bug.labels)
        new_ids = sorted(set(body.label_ids))
        if old != new_ids:
            labels = sess.scalars(
                select(Label).where(Label.id.in_(new_ids))).all() if new_ids else []
            bug.labels = list(labels)
            workflow.audit(sess, bug.id, user.id, "labels",
                           ",".join(map(str, old)), ",".join(map(str, new_ids)))
            changed.append("labels")
    return {"bug": bug_dict(sess, bug), "changed": changed}


@router.post("/{bug_id}/status")
def change_status(bug_id: int, body: StatusChange,
                  sess: Session = Depends(get_db),
                  user: User = Depends(current_user)):
    bug = get_bug_or_404(sess, bug_id)
    workflow.transition(sess, bug, body.to, user,
                        resolution=body.resolution, comment=body.comment)
    return bug_dict(sess, bug)


@router.get("/{bug_id}/history")
def get_history(bug_id: int, sess: Session = Depends(get_db),
                user: User = Depends(current_user)):
    get_bug_or_404(sess, bug_id)
    rows = sess.scalars(select(AuditLog).where(AuditLog.bug_id == bug_id)
                        .order_by(desc(AuditLog.id))).all()
    return [{"id": r.id, "field": r.field, "old_value": r.old_value,
             "new_value": r.new_value,
             "username": r.user.username if r.user else "",
             "created_at": r.created_at.isoformat() if r.created_at else None}
            for r in rows]


# ---- 关注 ----

@router.post("/{bug_id}/watch")
def watch(bug_id: int, sess: Session = Depends(get_db),
          user: User = Depends(current_user)):
    get_bug_or_404(sess, bug_id)
    if sess.get(BugWatcher, (bug_id, user.id)) is None:
        sess.add(BugWatcher(bug_id=bug_id, user_id=user.id))
    return {"watching": True}


@router.delete("/{bug_id}/watch")
def unwatch(bug_id: int, sess: Session = Depends(get_db),
            user: User = Depends(current_user)):
    row = sess.get(BugWatcher, (bug_id, user.id))
    if row is not None:
        sess.delete(row)
    return {"watching": False}


# ---- commit / 回归用例关联 ----

@router.post("/{bug_id}/links/commit", status_code=201)
def link_commit(bug_id: int, body: CommitLinkIn,
                sess: Session = Depends(get_db),
                user: User = Depends(require("dev"))):
    get_bug_or_404(sess, bug_id)
    link = CommitLink(bug_id=bug_id, repo=body.repo,
                      commit_hash=body.commit_hash, message=body.message)
    sess.add(link)
    sess.flush()
    return {"id": link.id}


@router.delete("/{bug_id}/links/commit/{link_id}")
def unlink_commit(bug_id: int, link_id: int, sess: Session = Depends(get_db),
                  user: User = Depends(require("dev"))):
    link = sess.get(CommitLink, link_id)
    if link is None or link.bug_id != bug_id:
        raise HTTPException(404, "关联不存在")
    sess.delete(link)
    return {"ok": True}


@router.post("/{bug_id}/links/test", status_code=201)
def link_test(bug_id: int, body: TestLinkIn, sess: Session = Depends(get_db),
              user: User = Depends(require("dev"))):
    get_bug_or_404(sess, bug_id)
    link = TestLink(bug_id=bug_id, test_path=body.test_path, note=body.note)
    sess.add(link)
    sess.flush()
    return {"id": link.id}


@router.delete("/{bug_id}/links/test/{link_id}")
def unlink_test(bug_id: int, link_id: int, sess: Session = Depends(get_db),
                user: User = Depends(require("dev"))):
    link = sess.get(TestLink, link_id)
    if link is None or link.bug_id != bug_id:
        raise HTTPException(404, "关联不存在")
    sess.delete(link)
    return {"ok": True}
