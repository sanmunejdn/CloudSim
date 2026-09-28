"""管理：标签 CRUD、JSON/CSV 导入导出。"""
from __future__ import annotations

from fastapi import APIRouter, Body, Depends, HTTPException, Query, UploadFile
from fastapi.responses import JSONResponse, PlainTextResponse
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import db, get_db
from ..models import Bug, Label, Project, User
from ..schemas import LabelIn, LabelOut
from ..security import current_user, require
from ..services import backup as backup_svc
from ..services import export as export_svc
from ..services.bug_ops import build_query

router = APIRouter(prefix="/api", tags=["admin"])


# ---- 标签 ----

@router.get("/labels")
def list_labels(sess: Session = Depends(get_db),
                user: User = Depends(current_user)) -> list[dict]:
    rows = sess.scalars(select(Label).order_by(Label.id)).all()
    return [LabelOut.model_validate(r).model_dump() for r in rows]


@router.post("/labels", status_code=201)
def create_label(body: LabelIn, sess: Session = Depends(get_db),
                 admin: User = Depends(require("admin"))) -> LabelOut:
    if sess.scalar(select(Label).where(Label.name == body.name)):
        raise HTTPException(409, "标签已存在")
    label = Label(name=body.name, color=body.color)
    sess.add(label)
    sess.flush()
    return LabelOut.model_validate(label)


@router.delete("/labels/{label_id}")
def delete_label(label_id: int, sess: Session = Depends(get_db),
                 admin: User = Depends(require("admin"))):
    label = sess.get(Label, label_id)
    if label is None:
        raise HTTPException(404, "标签不存在")
    sess.delete(label)
    return {"ok": True}


# ---- JSON 全量导入导出（admin） ----

@router.get("/export/json")
def export_json(sess: Session = Depends(get_db),
                admin: User = Depends(require("admin"))):
    payload = export_svc.export_json(sess)
    return JSONResponse(payload)


@router.post("/import/json")
def import_json(payload: dict = Body(...),
                confirm: str = Query(default=""),
                sess: Session = Depends(get_db),
                admin: User = Depends(require("admin"))):
    """全量导入（清空重建）。必须 confirm=yes 确认。导入后所有会话失效。"""
    if confirm != "yes":
        raise HTTPException(400, "全量导入会清空现有数据，请加 ?confirm=yes 确认")
    try:
        counts = export_svc.import_json(sess, payload)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    return {"ok": True, "counts": counts}


# ---- CSV 导入导出 ----

@router.get("/export/csv")
def export_csv(sess: Session = Depends(get_db),
               user: User = Depends(current_user),
               status: str | None = None, severity: str | None = None,
               priority: str | None = None, project_id: int | None = None,
               module_id: int | None = None, assignee_id: int | None = None,
               reporter_id: int | None = None, keyword: str | None = None):
    params = dict(status=status, severity=severity, priority=priority,
                  project_id=project_id, module_id=module_id,
                  assignee_id=assignee_id, reporter_id=reporter_id,
                  keyword=keyword)
    bugs = sess.scalars(build_query(sess, params).order_by(Bug.id)).all()
    text = export_svc.export_bugs_csv(sess, list(bugs))
    return PlainTextResponse(text, media_type="text/csv; charset=utf-8-sig")


@router.post("/import/csv")
async def import_csv(file: UploadFile, project_id: int,
                     sess: Session = Depends(get_db),
                     admin: User = Depends(require("admin"))):
    if sess.get(Project, project_id) is None:
        raise HTTPException(422, "项目不存在")
    raw = await file.read()
    if len(raw) > 10 * 1024 * 1024:
        raise HTTPException(413, "CSV 文件过大（上限 10MB）")
    text = raw.decode("utf-8-sig", errors="replace")
    count = export_svc.import_bugs_csv(sess, text, project_id, admin.id)
    return {"ok": True, "imported": count}


@router.post("/backup")
def create_backup(admin: User = Depends(require("admin"))):
    """在线一致备份主库 + 附件到 data/backups/<时间戳>/。"""
    return backup_svc.create_backup(db.settings)
