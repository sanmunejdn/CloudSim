"""项目 / 模块树 / 里程碑管理。"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import MILESTONE_STATUSES, Bug, Milestone, Module, Project, User
from ..schemas import (MilestoneIn, MilestoneOut, ModuleIn, ModuleOut,
                       ProjectIn, ProjectOut, ProjectPatch)
from ..security import current_user, require

router = APIRouter(prefix="/api", tags=["projects"])


def _module_tree(modules: list[Module]) -> list[dict]:
    nodes = {m.id: {**ModuleOut.model_validate(m).model_dump(), "children": []}
             for m in modules}
    roots = []
    for m in sorted(modules, key=lambda x: (x.sort, x.id)):
        node = nodes[m.id]
        if m.parent_id and m.parent_id in nodes:
            nodes[m.parent_id]["children"].append(node)
        else:
            roots.append(node)
    return roots


@router.get("/projects")
def list_projects(sess: Session = Depends(get_db),
                  user: User = Depends(current_user)) -> list[dict]:
    projects = sess.scalars(select(Project).order_by(Project.id)).all()
    result = []
    for p in projects:
        modules = sess.scalars(
            select(Module).where(Module.project_id == p.id)).all()
        milestones = sess.scalars(
            select(Milestone).where(Milestone.project_id == p.id)
            .order_by(Milestone.id)).all()
        bug_count = sess.scalar(
            select(func.count(Bug.id)).where(Bug.project_id == p.id))
        result.append({
            **ProjectOut.model_validate(p).model_dump(),
            "modules": _module_tree(modules),
            "milestones": [MilestoneOut.model_validate(ms).model_dump()
                           for ms in milestones],
            "bug_count": bug_count,
        })
    return result


@router.post("/projects", status_code=201)
def create_project(body: ProjectIn, sess: Session = Depends(get_db),
                   admin: User = Depends(require("admin"))) -> ProjectOut:
    if sess.scalar(select(Project).where(Project.key == body.key)):
        raise HTTPException(409, "项目 key 已存在")
    project = Project(key=body.key, name=body.name, description=body.description)
    sess.add(project)
    sess.flush()
    return ProjectOut.model_validate(project)


@router.patch("/projects/{project_id}")
def patch_project(project_id: int, body: ProjectPatch,
                  sess: Session = Depends(get_db),
                  admin: User = Depends(require("admin"))) -> ProjectOut:
    project = sess.get(Project, project_id)
    if project is None:
        raise HTTPException(404, "项目不存在")
    if body.name is not None:
        project.name = body.name
    if body.description is not None:
        project.description = body.description
    sess.flush()
    return ProjectOut.model_validate(project)


@router.delete("/projects/{project_id}")
def delete_project(project_id: int, sess: Session = Depends(get_db),
                   admin: User = Depends(require("admin"))):
    project = sess.get(Project, project_id)
    if project is None:
        raise HTTPException(404, "项目不存在")
    bug_count = sess.scalar(
        select(func.count(Bug.id)).where(Bug.project_id == project_id))
    if bug_count:
        raise HTTPException(409, f"项目下还有 {bug_count} 个 bug，不能删除")
    sess.delete(project)
    return {"ok": True}


# ---- 模块 ----

@router.post("/projects/{project_id}/modules", status_code=201)
def create_module(project_id: int, body: ModuleIn,
                  sess: Session = Depends(get_db),
                  admin: User = Depends(require("admin"))) -> ModuleOut:
    if sess.get(Project, project_id) is None:
        raise HTTPException(404, "项目不存在")
    if body.parent_id is not None:
        parent = sess.get(Module, body.parent_id)
        if parent is None or parent.project_id != project_id:
            raise HTTPException(422, "父模块不存在或不属于该项目")
    module = Module(project_id=project_id, parent_id=body.parent_id,
                    name=body.name, sort=body.sort)
    sess.add(module)
    sess.flush()
    return ModuleOut.model_validate(module)


@router.patch("/modules/{module_id}")
def patch_module(module_id: int, body: ModuleIn,
                 sess: Session = Depends(get_db),
                 admin: User = Depends(require("admin"))) -> ModuleOut:
    module = sess.get(Module, module_id)
    if module is None:
        raise HTTPException(404, "模块不存在")
    if body.parent_id == module_id:
        raise HTTPException(422, "父模块不能是自己")
    module.name = body.name
    module.sort = body.sort
    module.parent_id = body.parent_id
    sess.flush()
    return ModuleOut.model_validate(module)


@router.delete("/modules/{module_id}")
def delete_module(module_id: int, sess: Session = Depends(get_db),
                  admin: User = Depends(require("admin"))):
    module = sess.get(Module, module_id)
    if module is None:
        raise HTTPException(404, "模块不存在")
    child = sess.scalar(select(func.count(Module.id))
                        .where(Module.parent_id == module_id))
    if child:
        raise HTTPException(409, "请先删除子模块")
    used = sess.scalar(select(func.count(Bug.id))
                       .where(Bug.module_id == module_id))
    if used:
        raise HTTPException(409, f"模块已被 {used} 个 bug 引用，不能删除")
    sess.delete(module)
    return {"ok": True}


# ---- 里程碑 ----

@router.post("/projects/{project_id}/milestones", status_code=201)
def create_milestone(project_id: int, body: MilestoneIn,
                     sess: Session = Depends(get_db),
                     admin: User = Depends(require("admin"))) -> MilestoneOut:
    if sess.get(Project, project_id) is None:
        raise HTTPException(404, "项目不存在")
    if body.status not in MILESTONE_STATUSES:
        raise HTTPException(422, f"status 非法，允许值: {list(MILESTONE_STATUSES)}")
    milestone = Milestone(project_id=project_id, name=body.name,
                          due_date=body.due_date, status=body.status)
    sess.add(milestone)
    sess.flush()
    return MilestoneOut.model_validate(milestone)


@router.patch("/milestones/{milestone_id}")
def patch_milestone(milestone_id: int, body: MilestoneIn,
                    sess: Session = Depends(get_db),
                    admin: User = Depends(require("admin"))) -> MilestoneOut:
    milestone = sess.get(Milestone, milestone_id)
    if milestone is None:
        raise HTTPException(404, "里程碑不存在")
    if body.status not in MILESTONE_STATUSES:
        raise HTTPException(422, f"status 非法，允许值: {list(MILESTONE_STATUSES)}")
    milestone.name = body.name
    milestone.due_date = body.due_date
    milestone.status = body.status
    sess.flush()
    return MilestoneOut.model_validate(milestone)


@router.delete("/milestones/{milestone_id}")
def delete_milestone(milestone_id: int, sess: Session = Depends(get_db),
                     admin: User = Depends(require("admin"))):
    milestone = sess.get(Milestone, milestone_id)
    if milestone is None:
        raise HTTPException(404, "里程碑不存在")
    used = sess.scalar(select(func.count(Bug.id))
                       .where(Bug.milestone_id == milestone_id))
    if used:
        raise HTTPException(409, f"里程碑已被 {used} 个 bug 引用，不能删除")
    sess.delete(milestone)
    return {"ok": True}
