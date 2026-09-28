"""pydantic 请求/响应模型。"""
from __future__ import annotations

from datetime import datetime

from pydantic import BaseModel, Field


# ---- 认证 / 用户 ----

class LoginIn(BaseModel):
    username: str
    password: str


class UserOut(BaseModel):
    id: int
    username: str
    display_name: str
    role: str
    active: bool
    must_change_password: bool = False
    created_at: datetime
    model_config = {"from_attributes": True}


class UserCreate(BaseModel):
    username: str = Field(min_length=2, max_length=64)
    password: str = Field(min_length=6, max_length=128)
    display_name: str = ""
    role: str = "viewer"


class UserPatch(BaseModel):
    display_name: str | None = None
    role: str | None = None
    active: bool | None = None
    password: str | None = Field(default=None, min_length=6, max_length=128)


class PasswordChange(BaseModel):
    old_password: str
    new_password: str = Field(min_length=6, max_length=128)


# ---- 项目 / 模块 / 里程碑 / 标签 ----

class ProjectIn(BaseModel):
    key: str = Field(min_length=1, max_length=32)
    name: str = Field(min_length=1, max_length=128)
    description: str = ""


class ProjectPatch(BaseModel):
    name: str | None = None
    description: str | None = None


class ProjectOut(BaseModel):
    id: int
    key: str
    name: str
    description: str
    created_at: datetime
    model_config = {"from_attributes": True}


class ModuleIn(BaseModel):
    name: str = Field(min_length=1, max_length=128)
    parent_id: int | None = None
    sort: int = 0


class ModuleOut(BaseModel):
    id: int
    project_id: int
    parent_id: int | None
    name: str
    sort: int
    model_config = {"from_attributes": True}


class MilestoneIn(BaseModel):
    name: str = Field(min_length=1, max_length=128)
    due_date: str = ""
    status: str = "open"


class MilestoneOut(BaseModel):
    id: int
    project_id: int
    name: str
    due_date: str
    status: str
    model_config = {"from_attributes": True}


class LabelIn(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    color: str = "#888888"


class LabelOut(BaseModel):
    id: int
    name: str
    color: str
    model_config = {"from_attributes": True}


# ---- Bug ----

class BugCreate(BaseModel):
    project_id: int
    title: str = Field(min_length=1, max_length=256)
    module_id: int | None = None
    milestone_id: int | None = None
    description: str = ""
    repro_steps: str = ""
    expected: str = ""
    actual: str = ""
    severity: str = "major"
    priority: str = "P2"
    environment: str = "debug"
    version_found: str = ""
    assignee_id: int | None = None
    label_ids: list[int] = []


class BugPatch(BaseModel):
    title: str | None = Field(default=None, min_length=1, max_length=256)
    module_id: int | None = None
    milestone_id: int | None = None
    description: str | None = None
    repro_steps: str | None = None
    expected: str | None = None
    actual: str | None = None
    severity: str | None = None
    priority: str | None = None
    environment: str | None = None
    version_found: str | None = None
    version_fixed: str | None = None
    assignee_id: int | None = None
    label_ids: list[int] | None = None


class StatusChange(BaseModel):
    to: str
    resolution: str | None = None
    comment: str = ""


# ---- 评论 / 过滤器 ----

class CommentIn(BaseModel):
    body: str = Field(min_length=1, max_length=20000)


class FilterIn(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    query_json: str = "{}"


# ---- 关联 ----

class CommitLinkIn(BaseModel):
    repo: str = ""
    commit_hash: str = Field(min_length=4, max_length=64)
    message: str = ""


class TestLinkIn(BaseModel):
    test_path: str = Field(min_length=1, max_length=512)
    note: str = ""


# ---- 集成 ----

class CommitIn(BaseModel):
    repo: str = ""
    hash: str = Field(min_length=4, max_length=64)
    message: str = ""
    author: str = ""


class CiFailureIn(BaseModel):
    stamp: str = ""
    summary: str = Field(min_length=1, max_length=256)
    project_key: str = ""
    failures: list[dict] = []
