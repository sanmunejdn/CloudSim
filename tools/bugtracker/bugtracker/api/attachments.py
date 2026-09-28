"""附件：上传（类型白名单 + 大小限制）/ 列表 / 下载 / 删除。"""
from __future__ import annotations

import uuid
from pathlib import Path

from fastapi import APIRouter, Depends, HTTPException, UploadFile
from fastapi.responses import FileResponse
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import db, get_db
from ..models import Attachment, User
from ..security import current_user
from .bugs import get_bug_or_404

router = APIRouter(prefix="/api", tags=["attachments"])

ALLOWED_EXTS = {
    ".png", ".jpg", ".jpeg", ".gif", ".bmp", ".webp",
    ".log", ".txt", ".md", ".csv", ".xml", ".json",
    ".zip", ".7z", ".dmp", ".pdf",
}
CHUNK = 1024 * 1024


def _att_dict(a: Attachment) -> dict:
    return {"id": a.id, "bug_id": a.bug_id, "filename": a.filename,
            "size": a.size, "mime": a.mime, "uploaded_by": a.uploaded_by,
            "created_at": a.created_at.isoformat() if a.created_at else None}


@router.post("/bugs/{bug_id}/attachments", status_code=201)
async def upload(bug_id: int, file: UploadFile,
                 sess: Session = Depends(get_db),
                 user: User = Depends(current_user)):
    get_bug_or_404(sess, bug_id)
    if user.role == "viewer":
        raise HTTPException(403, "只读角色不能上传附件")

    original = Path(file.filename or "unnamed").name
    ext = Path(original).suffix.lower()
    if ext not in ALLOWED_EXTS:
        raise HTTPException(415, f"不支持的文件类型 {ext}，允许: {sorted(ALLOWED_EXTS)}")

    settings = db.settings
    stored_name = f"{uuid.uuid4().hex}{ext}"
    tmp_path = settings.attach_dir / f"{stored_name}.part"
    final_path = settings.attach_dir / stored_name

    size = 0
    try:
        with open(tmp_path, "wb") as out:
            while chunk := await file.read(CHUNK):
                size += len(chunk)
                if size > settings.max_upload_bytes:
                    raise HTTPException(
                        413, f"文件超过大小上限 {settings.max_upload_mb}MB")
                out.write(chunk)
        tmp_path.rename(final_path)
    except BaseException:
        tmp_path.unlink(missing_ok=True)
        raise

    att = Attachment(bug_id=bug_id, filename=original, stored_name=stored_name,
                     size=size, mime=file.content_type or "application/octet-stream",
                     uploaded_by=user.id)
    sess.add(att)
    sess.flush()
    return _att_dict(att)


@router.get("/bugs/{bug_id}/attachments")
def list_attachments(bug_id: int, sess: Session = Depends(get_db),
                     user: User = Depends(current_user)):
    get_bug_or_404(sess, bug_id)
    rows = sess.scalars(select(Attachment).where(Attachment.bug_id == bug_id)
                        .order_by(Attachment.id)).all()
    return [_att_dict(a) for a in rows]


@router.get("/attachments/{att_id}/download")
def download(att_id: int, sess: Session = Depends(get_db),
             user: User = Depends(current_user)):
    att = sess.get(Attachment, att_id)
    if att is None:
        raise HTTPException(404, "附件不存在")
    path = db.settings.attach_dir / att.stored_name
    if not path.is_file():
        raise HTTPException(410, "附件文件已丢失")
    return FileResponse(path, filename=att.filename, media_type=att.mime)


@router.delete("/attachments/{att_id}")
def delete_attachment(att_id: int, sess: Session = Depends(get_db),
                      user: User = Depends(current_user)):
    att = sess.get(Attachment, att_id)
    if att is None:
        raise HTTPException(404, "附件不存在")
    if att.uploaded_by != user.id and user.role != "admin":
        raise HTTPException(403, "只能删除自己上传的附件")
    (db.settings.attach_dir / att.stored_name).unlink(missing_ok=True)
    sess.delete(att)
    return {"ok": True}
