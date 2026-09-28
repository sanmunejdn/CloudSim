"""外部集成：git commit 联动 / CI 失败转 bug 草稿。X-Integration-Token 鉴权。"""
from __future__ import annotations

from fastapi import APIRouter, Depends
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import Bug, CommitLink, Project, User
from ..schemas import CiFailureIn, CommitIn
from ..security import verify_integration_token
from ..services import workflow
from ..services.gitlink import parse_commit_message

router = APIRouter(prefix="/api/integrations", tags=["integrations"],
                   dependencies=[Depends(verify_integration_token)])


def _agent_user(sess: Session) -> User:
    from ..db import db
    name = db.settings.agent_username if db.settings else "agent"
    user = sess.scalar(select(User).where(User.username == name))
    if user is None:  # 兜底：任意 admin
        user = sess.scalar(select(User).where(User.role == "admin").limit(1))
    return user


@router.post("/commit")
def commit_hook(body: CommitIn, sess: Session = Depends(get_db)):
    """git hook 上报 commit：fix #ID 置为已修复，ref/裸 #ID 仅关联。"""
    fix_ids, ref_ids = parse_commit_message(body.message)
    agent = _agent_user(sess)
    fixed, linked, missing = [], [], []

    for bug_id in sorted(fix_ids | ref_ids):
        bug = sess.get(Bug, bug_id)
        if bug is None:
            missing.append(bug_id)
            continue
        sess.add(CommitLink(bug_id=bug_id, repo=body.repo,
                            commit_hash=body.hash, message=body.message[:2000]))
        if bug_id in fix_ids:
            if bug.status in ("fixed", "verified", "closed"):
                linked.append(bug_id)  # 已是修复态，只记录关联
                continue
            try:
                workflow.transition(
                    sess, bug, "fixed", agent,
                    comment=f"commit {body.hash[:12]} 自动联动：{body.message[:200]}")
                fixed.append(bug_id)
            except Exception:
                # 状态机不允许时降级为仅关联（如 suspended 需先恢复）
                linked.append(bug_id)
        else:
            linked.append(bug_id)
    sess.flush()
    return {"fixed": fixed, "linked": linked, "missing": missing}


@router.post("/ci-failure", status_code=201)
def ci_failure(body: CiFailureIn, sess: Session = Depends(get_db)):
    """CI 失败 → 生成 bug 草稿（状态=新建，报告人=agent）。"""
    agent = _agent_user(sess)
    project = None
    if body.project_key:
        project = sess.scalar(
            select(Project).where(Project.key == body.project_key))
    if project is None:
        project = sess.scalar(select(Project).order_by(Project.id).limit(1))
    if project is None:
        from fastapi import HTTPException
        raise HTTPException(422, "系统中没有任何项目，请先创建")

    lines = [f"CI 批次: {body.stamp}", "", f"摘要: {body.summary}", ""]
    if body.failures:
        lines.append("失败用例:")
        for f in body.failures[:50]:
            name = f.get("name", "?")
            message = (f.get("message") or "")[:500]
            lines.append(f"- {name}: {message}")
    bug = Bug(project_id=project.id, title=f"[CI] {body.summary}"[:256],
              description="\n".join(lines), severity="major", priority="P1",
              environment="debug", reporter_id=agent.id, status="new")
    sess.add(bug)
    sess.flush()
    workflow.audit(sess, bug.id, agent.id, "created", "",
                   f"CI 自动草稿: {body.stamp}")
    return {"bug_id": bug.id, "project_key": project.key}
