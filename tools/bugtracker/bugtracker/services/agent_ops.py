"""Agent（MCP/CLI）操作入口：与 Web 共用状态机、审计与通知逻辑。

所有函数签名统一为 (sess, actor, ...)，写操作归属 actor（通常为种子用户 agent）。
"""
from __future__ import annotations

from fastapi import HTTPException
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from ..models import Bug, Comment, Module, Project, User
from . import workflow
from .bug_ops import bug_detail, build_query, get_bug_or_404, paged
from .notify import notify_bug_change, notify_mentions

OPEN_STATUSES = ("new", "confirmed", "in_progress", "reopened", "suspended")


def agent_user(sess: Session, username: str) -> User:
    user = sess.scalar(select(User).where(User.username == username,
                                          User.active.is_(True)))
    if user is None:
        raise HTTPException(500, f"agent 用户 {username} 不存在或已停用")
    return user


def _user_by_name(sess: Session, username: str) -> User:
    user = sess.scalar(select(User).where(User.username == username))
    if user is None:
        raise HTTPException(404, f"用户 {username} 不存在")
    return user


def _project_by_key(sess: Session, key: str) -> Project:
    project = sess.scalar(select(Project).where(Project.key == key))
    if project is None:
        raise HTTPException(404, f"项目 {key} 不存在")
    return project


# ---- 查询 ----

def list_bugs(sess: Session, actor: User, status: str = "",
              severity: str = "", priority: str = "", project_key: str = "",
              module: str = "", assignee: str = "", reporter: str = "",
              keyword: str = "", page: int = 1, size: int = 20) -> dict:
    params: dict = {"status": status or None, "severity": severity or None,
                    "priority": priority or None, "keyword": keyword or None}
    if project_key:
        params["project_id"] = _project_by_key(sess, project_key).id
    if module:
        mod = sess.scalar(select(Module).where(Module.name == module))
        if mod is None:
            raise HTTPException(404, f"模块 {module} 不存在")
        params["module_id"] = mod.id
    if assignee:
        params["assignee_id"] = _user_by_name(sess, assignee).id
    if reporter:
        params["reporter_id"] = _user_by_name(sess, reporter).id
    stmt = build_query(sess, params)
    if not status:  # 默认排除已关闭
        stmt = stmt.where(Bug.status != "closed")
    page = max(1, page)
    size = min(max(1, size), 200)
    return paged(sess, stmt, page, size)


def get_bug(sess: Session, actor: User, bug_id: int) -> dict:
    return bug_detail(sess, get_bug_or_404(sess, bug_id))


def search(sess: Session, actor: User, keyword: str, size: int = 20) -> dict:
    return list_bugs(sess, actor, keyword=keyword, status="", size=size)


def stats_overview(sess: Session, actor: User) -> dict:
    by_status = dict(sess.execute(
        select(Bug.status, func.count()).group_by(Bug.status)).all())
    by_severity = dict(sess.execute(
        select(Bug.severity, func.count()).group_by(Bug.severity)).all())
    return {
        "by_status": by_status,
        "by_severity": by_severity,
        "open": sum(by_status.get(s, 0) for s in OPEN_STATUSES),
        "closed": by_status.get("closed", 0),
        "total": sum(by_status.values()),
    }


def list_projects(sess: Session, actor: User) -> list[dict]:
    projects = sess.scalars(select(Project).order_by(Project.id)).all()
    return [{"id": p.id, "key": p.key, "name": p.name,
             "description": p.description,
             "bug_count": sess.scalar(
                 select(func.count(Bug.id)).where(Bug.project_id == p.id))}
            for p in projects]


def list_modules(sess: Session, actor: User, project_key: str = "") -> list[dict]:
    stmt = select(Module).order_by(Module.project_id, Module.sort, Module.id)
    if project_key:
        stmt = stmt.where(Module.project_id == _project_by_key(sess, project_key).id)
    return [{"id": mod.id, "project_id": mod.project_id,
             "parent_id": mod.parent_id, "name": mod.name}
            for mod in sess.scalars(stmt).all()]


# ---- 写操作 ----

def create_bug(sess: Session, actor: User, title: str, project_key: str = "",
               severity: str = "major", priority: str = "P2",
               description: str = "", repro_steps: str = "",
               expected: str = "", actual: str = "",
               environment: str = "debug", version_found: str = "",
               module: str = "", assignee: str = "") -> dict:
    workflow.validate_enums(severity, priority, environment)
    if project_key:
        project = _project_by_key(sess, project_key)
    else:
        project = sess.scalar(select(Project).order_by(Project.id).limit(1))
    if project is None:
        raise HTTPException(422, "系统中没有任何项目")

    module_id = None
    if module:
        mod = sess.scalar(select(Module).where(Module.name == module,
                                               Module.project_id == project.id))
        if mod is None:
            raise HTTPException(404, f"项目 {project.key} 下没有模块 {module}")
        module_id = mod.id
    assignee_id = None
    if assignee:
        assignee_id = _user_by_name(sess, assignee).id

    bug = Bug(project_id=project.id, module_id=module_id, title=title,
              description=description, repro_steps=repro_steps,
              expected=expected, actual=actual, severity=severity,
              priority=priority, environment=environment,
              version_found=version_found, reporter_id=actor.id,
              assignee_id=assignee_id, status="new")
    sess.add(bug)
    sess.flush()
    workflow.audit(sess, bug.id, actor.id, "created", "", bug.title)
    sess.flush()
    notify_bug_change(sess, bug, actor, "创建了 bug")
    return {"id": bug.id, "status": bug.status, "project_key": project.key,
            "url": f"/#/bugs/{bug.id}"}


def update_status(sess: Session, actor: User, bug_id: int, to: str,
                  resolution: str | None = None, comment: str = "") -> dict:
    bug = get_bug_or_404(sess, bug_id)
    workflow.transition(sess, bug, to, actor,
                        resolution=resolution, comment=comment)
    return {"id": bug.id, "status": bug.status, "resolution": bug.resolution}


def update_bug(sess: Session, actor: User, bug_id: int,
               title: str | None = None, severity: str | None = None,
               priority: str | None = None, environment: str | None = None,
               description: str | None = None, repro_steps: str | None = None,
               expected: str | None = None, actual: str | None = None,
               version_found: str | None = None, version_fixed: str | None = None,
               assignee: str | None = None, module: str | None = None) -> dict:
    bug = get_bug_or_404(sess, bug_id)
    workflow.validate_enums(severity, priority, environment)
    changes: dict = {}
    for field, value in (("title", title), ("severity", severity),
                         ("priority", priority), ("environment", environment),
                         ("description", description),
                         ("repro_steps", repro_steps), ("expected", expected),
                         ("actual", actual), ("version_found", version_found),
                         ("version_fixed", version_fixed)):
        if value is not None:
            changes[field] = value
    if assignee is not None:
        changes["assignee_id"] = _user_by_name(sess, assignee).id if assignee else None
    if module is not None:
        if module == "":
            changes["module_id"] = None
        else:
            mod = sess.scalar(select(Module).where(
                Module.name == module, Module.project_id == bug.project_id))
            if mod is None:
                raise HTTPException(404, f"模块 {module} 不存在")
            changes["module_id"] = mod.id
    changed = workflow.update_fields(sess, bug, changes, actor)
    return {"id": bug.id, "changed": changed}


def add_comment(sess: Session, actor: User, bug_id: int, body: str) -> dict:
    if not body.strip():
        raise HTTPException(422, "评论内容不能为空")
    bug = get_bug_or_404(sess, bug_id)
    comment = Comment(bug_id=bug_id, user_id=actor.id, body=body)
    sess.add(comment)
    sess.flush()
    notify_bug_change(sess, bug, actor, "发表了评论")
    notify_mentions(sess, bug, actor, body)
    return {"id": comment.id, "bug_id": bug_id}
