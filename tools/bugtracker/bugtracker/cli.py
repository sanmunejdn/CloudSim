"""命令行通道：python -m bugtracker.cli <子命令>（默认 JSON 输出）。

退出码：0 成功 / 2 参数错误 / 3 业务错误（非法流转、资源不存在等）。
供脚本、CI 与 AI Agent（MCP 不可用时的降级通道）使用。
"""
from __future__ import annotations

import argparse
import json
import sys

from fastapi import HTTPException

from .config import load_settings
from .db import db
from .services import agent_ops


def _bootstrap() -> None:
    if db.engine is None:
        db.init(load_settings())


def _emit(data, pretty: bool) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2 if pretty else None,
                     default=str))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="bugtracker",
                                     description="BugTracker 命令行")
    parser.add_argument("--pretty", action="store_true", help="缩进 JSON 输出")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", help="多条件查询（默认排除 closed）")
    p.add_argument("--status", default="")
    p.add_argument("--severity", default="")
    p.add_argument("--priority", default="")
    p.add_argument("--project-key", default="")
    p.add_argument("--module", default="")
    p.add_argument("--assignee", default="")
    p.add_argument("--reporter", default="")
    p.add_argument("--keyword", default="")
    p.add_argument("--page", type=int, default=1)
    p.add_argument("--size", type=int, default=20)

    p = sub.add_parser("show", help="bug 完整详情")
    p.add_argument("id", type=int)

    p = sub.add_parser("search", help="关键词全文搜索")
    p.add_argument("keyword")
    p.add_argument("--size", type=int, default=20)

    p = sub.add_parser("create", help="新建 bug")
    p.add_argument("--title", required=True)
    p.add_argument("--project-key", default="")
    p.add_argument("--severity", default="major")
    p.add_argument("--priority", default="P2")
    p.add_argument("--description", default="")
    p.add_argument("--repro-steps", default="")
    p.add_argument("--expected", default="")
    p.add_argument("--actual", default="")
    p.add_argument("--environment", default="debug")
    p.add_argument("--version-found", default="")
    p.add_argument("--module", default="")
    p.add_argument("--assignee", default="")

    p = sub.add_parser("status", help="状态流转")
    p.add_argument("id", type=int)
    p.add_argument("to")
    p.add_argument("--resolution", default=None)
    p.add_argument("--comment", default="")

    p = sub.add_parser("update", help="修改字段")
    p.add_argument("id", type=int)
    p.add_argument("--title", default=None)
    p.add_argument("--severity", default=None)
    p.add_argument("--priority", default=None)
    p.add_argument("--environment", default=None)
    p.add_argument("--description", default=None)
    p.add_argument("--assignee", default=None, help='用户名；"none" 取消指派')
    p.add_argument("--module", default=None)
    p.add_argument("--version-fixed", default=None)

    p = sub.add_parser("comment", help="添加评论")
    p.add_argument("id", type=int)
    p.add_argument("body")

    sub.add_parser("stats", help="统计总览")
    sub.add_parser("projects", help="项目列表")

    p = sub.add_parser("modules", help="模块列表")
    p.add_argument("--project-key", default="")

    p = sub.add_parser("export", help="全量 JSON 导出到文件")
    p.add_argument("--output", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _bootstrap()

    def work():
        with db.session() as sess:
            actor = agent_ops.agent_user(sess, db.settings.agent_username)

            if args.cmd == "list":
                return agent_ops.list_bugs(
                    sess, actor, status=args.status, severity=args.severity,
                    priority=args.priority, project_key=args.project_key,
                    module=args.module, assignee=args.assignee,
                    reporter=args.reporter, keyword=args.keyword,
                    page=args.page, size=args.size)
            if args.cmd == "show":
                return agent_ops.get_bug(sess, actor, args.id)
            if args.cmd == "search":
                return agent_ops.search(sess, actor, args.keyword,
                                        size=args.size)
            if args.cmd == "create":
                return agent_ops.create_bug(
                    sess, actor, title=args.title,
                    project_key=args.project_key, severity=args.severity,
                    priority=args.priority, description=args.description,
                    repro_steps=args.repro_steps, expected=args.expected,
                    actual=args.actual, environment=args.environment,
                    version_found=args.version_found, module=args.module,
                    assignee=args.assignee)
            if args.cmd == "status":
                return agent_ops.update_status(
                    sess, actor, args.id, args.to,
                    resolution=args.resolution, comment=args.comment)
            if args.cmd == "update":
                return agent_ops.update_bug(
                    sess, actor, args.id, title=args.title,
                    severity=args.severity, priority=args.priority,
                    environment=args.environment, description=args.description,
                    assignee=None if args.assignee is None else
                    ("" if args.assignee == "none" else args.assignee),
                    module=args.module, version_fixed=args.version_fixed)
            if args.cmd == "comment":
                return agent_ops.add_comment(sess, actor, args.id, args.body)
            if args.cmd == "stats":
                return agent_ops.stats_overview(sess, actor)
            if args.cmd == "projects":
                return agent_ops.list_projects(sess, actor)
            if args.cmd == "modules":
                return agent_ops.list_modules(sess, actor,
                                              project_key=args.project_key)
            if args.cmd == "export":
                from .services.export import export_json
                payload = export_json(sess)
                with open(args.output, "w", encoding="utf-8") as f:
                    json.dump(payload, f, ensure_ascii=False, indent=1)
                return {"ok": True, "output": args.output}
            raise AssertionError(args.cmd)  # pragma: no cover

    try:
        result = db.run_with_retry(work)
        _emit(result, args.pretty)
        return 0
    except HTTPException as exc:
        _emit({"error": exc.detail, "status_code": exc.status_code},
              args.pretty)
        return 3


if __name__ == "__main__":
    sys.exit(main())
