/* BugTracker SPA：hash 路由 + 视图渲染（无构建，原生 JS） */
"use strict";

// ---------- 常量（与后端 models/workflow 一致） ----------
const STATUS_CN = {
  new: "新建", confirmed: "已确认", in_progress: "处理中", fixed: "已修复",
  verified: "回归验证", closed: "已关闭", reopened: "重新打开",
  rejected: "已拒绝", duplicate: "重复", suspended: "挂起",
};
const SEVERITY_CN = { fatal: "致命", major: "严重", minor: "一般", suggestion: "建议" };
const RESOLUTION_CN = {
  fixed: "已修复", wontfix: "不予修复", duplicate: "重复",
  bydesign: "设计如此", cannot_reproduce: "无法复现",
};
const ENV_CN = { debug: "Debug", release: "Release" };
const ROLE_CN = { admin: "管理员", dev: "开发", tester: "测试", viewer: "只读" };
// 前端流转提示（后端状态机为权威，非法操作会被 409 拒绝）
const TRANSITIONS = {
  new: ["confirmed", "rejected", "duplicate"],
  confirmed: ["in_progress", "suspended", "rejected", "duplicate"],
  in_progress: ["fixed", "suspended", "confirmed"],
  suspended: ["in_progress", "confirmed"],
  fixed: ["verified", "reopened"],
  verified: ["closed", "reopened"],
  reopened: ["in_progress", "confirmed"],
  rejected: ["reopened"],
  duplicate: ["reopened"],
  closed: [],
};

// ---------- 全局状态 ----------
const state = {
  user: null, projects: [], users: [], labels: [],
  listParams: { page: 1, size: 20, sort: "created_at", order: "desc" },
};

// ---------- 工具 ----------
const $ = (sel, root) => (root || document).querySelector(sel);
const $$ = (sel, root) => Array.from((root || document).querySelectorAll(sel));

function esc(s) {
  return String(s ?? "").replace(/[&<>"']/g,
    c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
function fmtTime(iso) { return iso ? iso.replace("T", " ").slice(0, 16) : "-"; }
function statusTag(s) { return `<span class="tag status-${s}">${STATUS_CN[s] || s}</span>`; }
function sevTag(s) { return `<span class="tag sev-${s}">${SEVERITY_CN[s] || s}</span>`; }

function toast(msg, isError) {
  const el = $("#toast");
  el.textContent = msg;
  el.className = isError ? "error" : "";
  el.style.display = "block";
  clearTimeout(el._timer);
  el._timer = setTimeout(() => { el.style.display = "none"; }, 3000);
}

async function guard(fn) {
  try { await fn(); } catch (e) { toast(e.message, true); }
}

// ---------- 数据预载 ----------
async function preload() {
  [state.projects, state.users, state.labels] = await Promise.all([
    api.get("/api/projects"), api.get("/api/users"), api.get("/api/labels"),
  ]);
}

function userOptions(selectedId, allowEmpty) {
  const opts = state.users.filter(u => u.active)
    .map(u => `<option value="${u.id}" ${u.id === selectedId ? "selected" : ""}>${esc(u.display_name || u.username)}</option>`);
  return (allowEmpty ? `<option value="">（未指派）</option>` : "") + opts.join("");
}
function projectOptions(selectedId) {
  return state.projects.map(p =>
    `<option value="${p.id}" ${p.id === selectedId ? "selected" : ""}>${esc(p.name)}</option>`).join("");
}
function flatModules(project) {
  const out = [];
  (function walk(nodes, prefix) {
    for (const n of nodes || []) {
      out.push({ id: n.id, name: prefix + n.name });
      walk(n.children, prefix + n.name + " / ");
    }
  })(project ? project.modules : [], "");
  return out;
}

// ---------- 布局 ----------
function renderShell() {
  document.body.innerHTML = `
  <div id="toast"></div>
  <div class="layout">
    <aside class="sidebar">
      <div class="brand">🐞 BugTracker</div>
      <nav>
        <a href="#/dashboard" data-nav="/dashboard">看板</a>
        <a href="#/bugs" data-nav="/bugs">Bug 列表</a>
        <a href="#/bugs/new" data-nav="/bugs/new">新建 Bug</a>
        <a href="#/projects" data-nav="/projects" data-admin>项目管理</a>
        <a href="#/users" data-nav="/users" data-admin>用户管理</a>
        <a href="#/notifications" data-nav="/notifications">通知 <span id="notif-badge"></span></a>
        <a href="#/settings" data-nav="/settings">设置</a>
      </nav>
      <div class="user-box">
        <div>${esc(state.user.display_name || state.user.username)}
          <span class="tag">${ROLE_CN[state.user.role]}</span></div>
        <a href="#" id="btn-logout">退出登录</a>
      </div>
    </aside>
    <main id="content"></main>
  </div>`;
  $$("[data-admin]").forEach(el => {
    if (state.user.role !== "admin") el.style.display = "none";
  });
  $("#btn-logout").onclick = () => guard(async () => {
    await api.post("/api/auth/logout");
    state.user = null;
    location.hash = "#/login";
  });
  refreshBadge();
}

async function refreshBadge() {
  try {
    const r = await api.get("/api/notifications/unread-count");
    $("#notif-badge").textContent = r.count || "";
  } catch { /* 忽略角标失败 */ }
}

function setActiveNav(path) {
  $$(".sidebar nav a").forEach(a =>
    a.classList.toggle("active", path.startsWith(a.dataset.nav)));
}

// ---------- 登录 ----------
function renderLogin() {
  document.body.innerHTML = `
  <div id="toast"></div>
  <div class="login-wrap"><div class="card login-card">
    <h2>🐞 BugTracker</h2>
    <input id="login-user" placeholder="用户名" autocomplete="username">
    <input id="login-pass" type="password" placeholder="密码" autocomplete="current-password">
    <button id="btn-login">登 录</button>
    <div class="muted mt" style="text-align:center">CloudSim 缺陷跟踪系统</div>
  </div></div>`;
  const submit = () => guard(async () => {
    const r = await api.post("/api/auth/login", {
      username: $("#login-user").value.trim(),
      password: $("#login-pass").value,
    });
    state.user = r.user;
    if (state.user.must_change_password) {
      return renderChangePassword(true);
    }
    location.hash = "#/dashboard";
  });
  $("#btn-login").onclick = submit;
  $("#login-pass").onkeydown = e => { if (e.key === "Enter") submit(); };
}

function renderChangePassword(forced) {
  document.body.innerHTML = `
  <div id="toast"></div>
  <div class="login-wrap"><div class="card login-card">
    <h2>修改密码</h2>
    ${forced ? '<div class="muted mb">首次登录，请先修改初始密码</div>' : ""}
    <input id="old-pass" type="password" placeholder="原密码">
    <input id="new-pass" type="password" placeholder="新密码（至少 6 位）">
    <button id="btn-chpass">确认修改</button>
  </div></div>`;
  $("#btn-chpass").onclick = () => guard(async () => {
    await api.post("/api/auth/password", {
      old_password: $("#old-pass").value, new_password: $("#new-pass").value,
    });
    toast("密码已修改");
    state.user.must_change_password = false;
    location.hash = "#/dashboard";
  });
}

// ---------- 看板 ----------
async function renderDashboard() {
  const c = $("#content");
  c.innerHTML = `<h2 class="mb">看板</h2>
    <div class="chart-grid">
      <div class="card"><h3>状态分布</h3><div id="ch-status" class="chart"></div></div>
      <div class="card"><h3>严重级别</h3><div id="ch-sev" class="chart"></div></div>
      <div class="card"><h3>近 30 天趋势</h3><div id="ch-trend" class="chart"></div></div>
      <div class="card"><h3>未决 bug 模块分布</h3><div id="ch-module" class="chart"></div></div>
    </div>
    <div class="chart-grid mt">
      <div class="card"><h3>指派给我的未决</h3><div id="my-todo"></div></div>
      <div class="card"><h3>人员工作量</h3><div id="by-user"></div></div>
    </div>`;

  const [overview, trend, byModule, byUser, myTodo] = await Promise.all([
    api.get("/api/stats/overview"), api.get("/api/stats/trend?days=30"),
    api.get("/api/stats/by-module"), api.get("/api/stats/by-user"),
    api.get(`/api/bugs?assignee_id=${state.user.id}&status=new,confirmed,in_progress,reopened,suspended&size=10`),
  ]);

  const dark = { textStyle: { color: "#9ba0ab" } };
  const pie = (id, dataMap, nameMap) => echarts.init($(id)).setOption({
    ...dark,
    tooltip: { trigger: "item" },
    series: [{
      type: "pie", radius: ["40%", "70%"],
      data: Object.entries(dataMap).map(([k, v]) => ({ name: (nameMap && nameMap[k]) || k, value: v })),
      label: { color: "#9ba0ab" },
    }],
  });
  pie("#ch-status", overview.by_status, STATUS_CN);
  pie("#ch-sev", overview.by_severity, SEVERITY_CN);

  echarts.init($("#ch-trend")).setOption({
    ...dark,
    tooltip: { trigger: "axis" },
    legend: { data: ["新建", "关闭"], textStyle: { color: "#9ba0ab" } },
    xAxis: { type: "category", data: trend.days.map(d => d.date.slice(5)) },
    yAxis: { type: "value", minInterval: 1 },
    series: [
      { name: "新建", type: "line", smooth: true, data: trend.days.map(d => d.created), itemStyle: { color: "#4f8cff" } },
      { name: "关闭", type: "line", smooth: true, data: trend.days.map(d => d.closed), itemStyle: { color: "#57ab5a" } },
    ],
  });

  echarts.init($("#ch-module")).setOption({
    ...dark,
    tooltip: {},
    xAxis: { type: "category", data: byModule.items.map(i => i.module_name) },
    yAxis: { type: "value", minInterval: 1 },
    series: [{ type: "bar", data: byModule.items.map(i => i.open_count), itemStyle: { color: "#4f8cff" } }],
  });

  $("#my-todo").innerHTML = myTodo.items.length
    ? `<table>${myTodo.items.map(b => `
        <tr><td><a href="#/bugs/${b.id}">#${b.id}</a></td>
        <td>${esc(b.title)}</td><td>${statusTag(b.status)}</td><td>${sevTag(b.severity)}</td></tr>`).join("")}</table>`
    : '<div class="muted">没有指派给你的未决 bug 🎉</div>';

  $("#by-user").innerHTML = `<table>
    <tr><th>用户</th><th>待处理</th><th>已修复</th><th>报告</th></tr>
    ${byUser.items.map(i => `<tr><td>${esc(i.username)}</td>
      <td>${i.assigned_open}</td><td>${i.fixed}</td><td>${i.reported}</td></tr>`).join("")}</table>`;
}

// ---------- Bug 列表 ----------
function currentFilters() {
  const p = {};
  const v = id => $(id) && $(id).value;
  if (v("#f-status")) p.status = v("#f-status");
  if (v("#f-severity")) p.severity = v("#f-severity");
  if (v("#f-priority")) p.priority = v("#f-priority");
  if (v("#f-project")) p.project_id = v("#f-project");
  if (v("#f-assignee")) p.assignee_id = v("#f-assignee");
  if (v("#f-keyword")) p.keyword = v("#f-keyword");
  return p;
}

function buildQueryString(params) {
  return Object.entries(params).filter(([, v]) => v !== "" && v != null)
    .map(([k, v]) => `${k}=${encodeURIComponent(v)}`).join("&");
}

async function renderBugList() {
  const c = $("#content");
  const filters = await api.get("/api/filters");
  c.innerHTML = `<h2 class="mb">Bug 列表</h2>
  <div class="card">
    <div class="filter-bar">
      <select id="f-status"><option value="">全部状态</option>
        ${Object.entries(STATUS_CN).map(([k, v]) => `<option value="${k}">${v}</option>`).join("")}</select>
      <select id="f-severity"><option value="">全部级别</option>
        ${Object.entries(SEVERITY_CN).map(([k, v]) => `<option value="${k}">${v}</option>`).join("")}</select>
      <select id="f-priority"><option value="">全部优先级</option>
        ${["P0", "P1", "P2", "P3"].map(p => `<option>${p}</option>`).join("")}</select>
      <select id="f-project"><option value="">全部项目</option>${projectOptions()}</select>
      <select id="f-assignee"><option value="">全部指派人</option>${userOptions()}</select>
      <input id="f-keyword" placeholder="关键词搜索" style="min-width:160px">
      <button id="btn-search">查询</button>
      <button class="secondary" id="btn-export">导出 CSV</button>
    </div>
    <div class="filter-bar">
      <select id="f-saved"><option value="">已存过滤器…</option>
        ${filters.map(f => `<option value="${f.id}">${esc(f.name)}</option>`).join("")}</select>
      <button class="secondary" id="btn-save-filter">保存当前过滤</button>
      <button class="secondary" id="btn-del-filter">删除过滤器</button>
    </div>
    <div id="bug-table"></div>
    <div class="pager">
      <button class="secondary" id="pg-prev">上一页</button>
      <span id="pg-info" class="muted"></span>
      <button class="secondary" id="pg-next">下一页</button>
    </div>
  </div>`;

  const load = async () => {
    const params = { ...state.listParams, ...currentFilters() };
    const data = await api.get("/api/bugs?" + buildQueryString(params));
    $("#bug-table").innerHTML = `<table>
      <tr><th>ID</th><th>标题</th><th>状态</th><th>级别</th><th>优先级</th>
      <th>模块</th><th>指派人</th><th>报告人</th><th>更新</th></tr>
      ${data.items.map(b => `<tr>
        <td><a href="#/bugs/${b.id}">#${b.id}</a></td>
        <td><a href="#/bugs/${b.id}">${esc(b.title)}</a></td>
        <td>${statusTag(b.status)}</td><td>${sevTag(b.severity)}</td>
        <td>${b.priority}</td><td>${esc(b.module_name || "-")}</td>
        <td>${esc(b.assignee_name || "-")}</td><td>${esc(b.reporter_name)}</td>
        <td class="muted">${fmtTime(b.updated_at)}</td></tr>`).join("")}
    </table>
    ${data.items.length ? "" : '<div class="muted" style="padding:16px">没有符合条件的 bug</div>'}`;
    const pages = Math.max(1, Math.ceil(data.total / state.listParams.size));
    $("#pg-info").textContent = `第 ${state.listParams.page} / ${pages} 页，共 ${data.total} 条`;
    $("#pg-prev").disabled = state.listParams.page <= 1;
    $("#pg-next").disabled = state.listParams.page >= pages;
  };

  $("#btn-search").onclick = () => guard(async () => { state.listParams.page = 1; await load(); });
  $("#f-keyword").onkeydown = e => { if (e.key === "Enter") $("#btn-search").click(); };
  $$(".filter-bar select").forEach(s => s.onchange = () => guard(async () => { state.listParams.page = 1; await load(); }));
  $("#pg-prev").onclick = () => guard(async () => { state.listParams.page--; await load(); });
  $("#pg-next").onclick = () => guard(async () => { state.listParams.page++; await load(); });
  $("#btn-export").onclick = () => {
    location.href = "/api/export/csv?" + buildQueryString(currentFilters());
  };
  $("#btn-save-filter").onclick = () => guard(async () => {
    const name = prompt("过滤器名称：");
    if (!name) return;
    await api.post("/api/filters", { name, query_json: JSON.stringify(currentFilters()) });
    toast("已保存");
    await renderBugList();
  });
  $("#f-saved").onchange = () => guard(async () => {
    const f = filters.find(x => x.id === +$("#f-saved").value);
    if (!f) return;
    const q = JSON.parse(f.query_json);
    $("#f-status").value = q.status || "";
    $("#f-severity").value = q.severity || "";
    $("#f-priority").value = q.priority || "";
    $("#f-project").value = q.project_id || "";
    $("#f-assignee").value = q.assignee_id || "";
    $("#f-keyword").value = q.keyword || "";
    state.listParams.page = 1;
    await load();
  });
  $("#btn-del-filter").onclick = () => guard(async () => {
    const id = $("#f-saved").value;
    if (!id) return;
    await api.del("/api/filters/" + id);
    toast("已删除");
    await renderBugList();
  });
  await load();
}

// ---------- Bug 新建 / 编辑 ----------
async function renderBugEdit(bugId) {
  const isEdit = bugId != null;
  const bug = isEdit ? await api.get("/api/bugs/" + bugId) : {
    title: "", description: "", repro_steps: "", expected: "", actual: "",
    severity: "major", priority: "P2", environment: "debug",
    project_id: state.projects[0] && state.projects[0].id,
    module_id: null, milestone_id: null, assignee_id: null,
    version_found: "", labels: [],
  };
  const c = $("#content");
  const project = state.projects.find(p => p.id === bug.project_id) || state.projects[0];

  c.innerHTML = `<h2 class="mb">${isEdit ? `编辑 #${bug.id}` : "新建 Bug"}</h2>
  <div class="card"><div class="form-grid">
    <div class="form-item full"><label>标题 *</label>
      <input id="e-title" value="${esc(bug.title)}"></div>
    <div class="form-item"><label>项目</label>
      <select id="e-project">${projectOptions(bug.project_id)}</select></div>
    <div class="form-item"><label>模块</label><select id="e-module"></select></div>
    <div class="form-item"><label>里程碑</label><select id="e-milestone"></select></div>
    <div class="form-item"><label>指派人</label>
      <select id="e-assignee">${userOptions(bug.assignee_id, true)}</select></div>
    <div class="form-item"><label>严重级别</label>
      <select id="e-severity">${Object.entries(SEVERITY_CN).map(([k, v]) =>
        `<option value="${k}" ${k === bug.severity ? "selected" : ""}>${v}</option>`).join("")}</select></div>
    <div class="form-item"><label>优先级</label>
      <select id="e-priority">${["P0", "P1", "P2", "P3"].map(p =>
        `<option ${p === bug.priority ? "selected" : ""}>${p}</option>`).join("")}</select></div>
    <div class="form-item"><label>环境</label>
      <select id="e-environment">${Object.entries(ENV_CN).map(([k, v]) =>
        `<option value="${k}" ${k === bug.environment ? "selected" : ""}>${v}</option>`).join("")}</select></div>
    <div class="form-item"><label>发现版本</label>
      <input id="e-version" value="${esc(bug.version_found)}"></div>
    <div class="form-item full"><label>标签</label><div id="e-labels" class="row">
      ${state.labels.map(l => `<label class="tag">
        <input type="checkbox" value="${l.id}" ${bug.labels.some(x => x.id === l.id) ? "checked" : ""}> ${esc(l.name)}
      </label>`).join("") || '<span class="muted">暂无标签（admin 可在设置页维护）</span>'}</div></div>
    <div class="form-item full"><label>问题描述</label>
      <textarea id="e-description">${esc(bug.description)}</textarea></div>
    <div class="form-item full"><label>复现步骤</label>
      <textarea id="e-repro">${esc(bug.repro_steps)}</textarea></div>
    <div class="form-item full"><label>期望结果</label>
      <textarea id="e-expected" style="min-height:50px">${esc(bug.expected)}</textarea></div>
    <div class="form-item full"><label>实际结果</label>
      <textarea id="e-actual" style="min-height:50px">${esc(bug.actual)}</textarea></div>
  </div>
  <div class="mt row">
    <button id="e-save">${isEdit ? "保存修改" : "提交 Bug"}</button>
    <button class="secondary" onclick="history.back()">取消</button>
  </div></div>`;

  const fillModules = () => {
    const p = state.projects.find(x => x.id === +$("#e-project").value);
    const mods = flatModules(p);
    $("#e-module").innerHTML = `<option value="">（未分类）</option>` +
      mods.map(m => `<option value="${m.id}" ${m.id === bug.module_id ? "selected" : ""}>${esc(m.name)}</option>`).join("");
    $("#e-milestone").innerHTML = `<option value="">（无）</option>` +
      (p ? p.milestones.filter(m => m.status === "open").map(m =>
        `<option value="${m.id}" ${m.id === bug.milestone_id ? "selected" : ""}>${esc(m.name)}</option>`).join("") : "");
  };
  $("#e-project").onchange = fillModules;
  fillModules();

  $("#e-save").onclick = () => guard(async () => {
    const payload = {
      title: $("#e-title").value.trim(),
      project_id: +$("#e-project").value,
      module_id: $("#e-module").value ? +$("#e-module").value : null,
      milestone_id: $("#e-milestone").value ? +$("#e-milestone").value : null,
      assignee_id: $("#e-assignee").value ? +$("#e-assignee").value : null,
      severity: $("#e-severity").value, priority: $("#e-priority").value,
      environment: $("#e-environment").value,
      version_found: $("#e-version").value.trim(),
      description: $("#e-description").value, repro_steps: $("#e-repro").value,
      expected: $("#e-expected").value, actual: $("#e-actual").value,
      label_ids: $$("#e-labels input:checked").map(x => +x.value),
    };
    if (!payload.title) { toast("标题不能为空", true); return; }
    if (isEdit) {
      delete payload.project_id;
      await api.patch("/api/bugs/" + bugId, payload);
      toast("已保存");
      location.hash = "#/bugs/" + bugId;
    } else {
      const r = await api.post("/api/bugs", payload);
      toast(`已创建 #${r.id}`);
      location.hash = "#/bugs/" + r.id;
    }
  });
}

// ---------- Bug 详情 ----------
async function renderBugDetail(bugId) {
  const [bug, comments, attachments, history] = await Promise.all([
    api.get("/api/bugs/" + bugId),
    api.get(`/api/bugs/${bugId}/comments`),
    api.get(`/api/bugs/${bugId}/attachments`),
    api.get(`/api/bugs/${bugId}/history`),
  ]);
  const c = $("#content");
  const transitions = TRANSITIONS[bug.status] || [];
  const canEdit = state.user.role !== "viewer";

  c.innerHTML = `
  <div class="row mb">
    <h2 class="grow">#${bug.id} ${esc(bug.title)}</h2>
    ${statusTag(bug.status)} ${sevTag(bug.severity)}
    <span class="tag">${bug.priority}</span>
    ${bug.resolution ? `<span class="tag">${RESOLUTION_CN[bug.resolution] || bug.resolution}</span>` : ""}
  </div>
  <div class="card">
    <div class="row mb">
      ${transitions.map(t =>
        `<button data-to="${t}" class="${["rejected", "duplicate"].includes(t) ? "danger" : ""}">${STATUS_CN[t]}</button>`).join("")}
      ${canEdit ? `<button class="secondary" onclick="location.hash='#/bugs/${bug.id}/edit'">编辑</button>` : ""}
      <button class="secondary" id="btn-watch">${bug.watching ? "取消关注" : "关注"}</button>
    </div>
    <div class="field-grid">
      <div><div class="k">项目 / 模块</div>${esc(bug.project_key)} / ${esc(bug.module_name || "未分类")}</div>
      <div><div class="k">里程碑</div>${esc(bug.milestone_name || "-")}</div>
      <div><div class="k">环境</div>${ENV_CN[bug.environment] || bug.environment}</div>
      <div><div class="k">报告人</div>${esc(bug.reporter_name)}</div>
      <div><div class="k">指派人</div>${esc(bug.assignee_name || "未指派")}</div>
      <div><div class="k">标签</div>${bug.labels.map(l => `<span class="tag" style="border-color:${esc(l.color)}">${esc(l.name)}</span>`).join(" ") || "-"}</div>
      <div><div class="k">发现版本</div>${esc(bug.version_found || "-")}</div>
      <div><div class="k">修复版本</div>${esc(bug.version_fixed || "-")}</div>
      <div><div class="k">创建 / 更新</div><span class="muted">${fmtTime(bug.created_at)} / ${fmtTime(bug.updated_at)}</span></div>
    </div>
    <div class="section-title">问题描述</div><div>${esc(bug.description) || '<span class="muted">（空）</span>'}</div>
    <div class="section-title">复现步骤</div><div style="white-space:pre-wrap">${esc(bug.repro_steps) || '<span class="muted">（空）</span>'}</div>
    <div class="section-title">期望 / 实际</div>
    <div class="row" style="align-items:flex-start">
      <div class="grow">${esc(bug.expected) || "-"}</div>
      <div class="grow">${esc(bug.actual) || "-"}</div>
    </div>
  </div>

  <div class="card"><h3>评论（${comments.length}）</h3>
    <div id="comment-list">${comments.map(cm => `
      <div class="comment"><div class="meta">${esc(cm.username)} · ${fmtTime(cm.created_at)}
        ${(cm.username === state.user.username || state.user.role === "admin") ?
          `<a href="#" data-del-comment="${cm.id}" style="float:right">删除</a>` : ""}</div>
      <div style="white-space:pre-wrap">${esc(cm.body)}</div></div>`).join("")}</div>
    ${canEdit ? `<textarea id="c-body" placeholder="发表评论，@用户名 可提醒对方"></textarea>
    <button class="mt" id="btn-comment">发表</button>` : ""}
  </div>

  <div class="card"><h3>附件（${attachments.length}）</h3>
    <div id="att-list">${attachments.map(a => `
      <div class="row" style="padding:4px 0">
        <a href="/api/attachments/${a.id}/download">${esc(a.filename)}</a>
        <span class="muted">${(a.size / 1024).toFixed(1)} KB</span>
        ${(a.uploaded_by === state.user.id || state.user.role === "admin") ?
          `<a href="#" data-del-att="${a.id}">删除</a>` : ""}
      </div>`).join("") || '<div class="muted">无附件</div>'}</div>
    ${canEdit ? `<div class="row mt"><input type="file" id="att-file">
      <button class="secondary" id="btn-upload">上传</button>
      <span class="muted">支持图片/日志/zip/dmp 等，≤50MB</span></div>` : ""}
  </div>

  <div class="card"><h3>关联</h3>
    <div class="k muted">Commit</div>
    <div>${bug.commits.map(x => `<div class="history-item">
      <code>${esc(x.commit_hash.slice(0, 12))}</code> ${esc(x.message)} <span class="muted">${esc(x.repo)}</span></div>`).join("")
      || '<div class="muted">无</div>'}</div>
    <div class="k muted mt">回归用例</div>
    <div>${bug.tests.map(t => `<div class="history-item">${esc(t.test_path)} ${esc(t.note)}
      ${state.user.role !== "viewer" ? `<a href="#" data-del-test="${t.id}">删除</a>` : ""}</div>`).join("")
      || '<div class="muted">无</div>'}</div>
    ${canEdit ? `<div class="row mt"><input id="test-path" placeholder="回归用例路径，如 tests/api/test_xxx.py::test_yyy" class="grow">
      <button class="secondary" id="btn-add-test">添加</button></div>` : ""}
  </div>

  <div class="card"><h3>变更历史（${history.length}）</h3>
    ${history.map(h => `<div class="history-item">
      ${fmtTime(h.created_at)} · ${esc(h.username)} ·
      ${h.field === "created" ? "创建" : `${esc(h.field)}: ${esc(h.old_value)} → ${esc(h.new_value)}`}
    </div>`).join("")}
  </div>`;

  // 状态流转
  $$("button[data-to]").forEach(btn => btn.onclick = () => guard(async () => {
    const to = btn.dataset.to;
    let resolution = null;
    if (to === "rejected") {
      resolution = prompt("拒绝原因（wontfix 不予修复 / bydesign 设计如此 / cannot_reproduce 无法复现）：", "wontfix");
      if (!resolution) return;
    }
    const comment = prompt(`流转为「${STATUS_CN[to]}」的备注（可空）：`) || "";
    await api.post(`/api/bugs/${bugId}/status`, { to, resolution, comment });
    toast("已流转");
    await renderBugDetail(bugId);
    refreshBadge();
  }));

  $("#btn-watch").onclick = () => guard(async () => {
    if (bug.watching) await api.del(`/api/bugs/${bugId}/watch`);
    else await api.post(`/api/bugs/${bugId}/watch`);
    await renderBugDetail(bugId);
  });

  const btnComment = $("#btn-comment");
  if (btnComment) btnComment.onclick = () => guard(async () => {
    const body = $("#c-body").value.trim();
    if (!body) return;
    await api.post(`/api/bugs/${bugId}/comments`, { body });
    await renderBugDetail(bugId);
  });

  $$("[data-del-comment]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      await api.del(`/api/bugs/${bugId}/comments/${a.dataset.delComment}`);
      await renderBugDetail(bugId);
    });
  });

  const btnUpload = $("#btn-upload");
  if (btnUpload) btnUpload.onclick = () => guard(async () => {
    const file = $("#att-file").files[0];
    if (!file) return;
    const fd = new FormData();
    fd.append("file", file);
    await api.upload(`/api/bugs/${bugId}/attachments`, fd);
    toast("已上传");
    await renderBugDetail(bugId);
  });

  $$("[data-del-att]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      await api.del("/api/attachments/" + a.dataset.delAtt);
      await renderBugDetail(bugId);
    });
  });

  const btnAddTest = $("#btn-add-test");
  if (btnAddTest) btnAddTest.onclick = () => guard(async () => {
    const path = $("#test-path").value.trim();
    if (!path) return;
    await api.post(`/api/bugs/${bugId}/links/test`, { test_path: path, note: "" });
    await renderBugDetail(bugId);
  });

  $$("[data-del-test]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      await api.del(`/api/bugs/${bugId}/links/test/${a.dataset.delTest}`);
      await renderBugDetail(bugId);
    });
  });
}

// ---------- 项目管理（admin） ----------
async function renderProjects() {
  await preload();
  const c = $("#content");
  const renderModuleTree = nodes => `<ul>${nodes.map(n => `
    <li class="module-node">📁 ${esc(n.name)}
      <span class="ops">
        <a href="#" data-add-child="${n.id}">加子模块</a>
        <a href="#" data-ren-module="${n.id}" data-name="${esc(n.name)}">改名</a>
        <a href="#" data-del-module="${n.id}">删除</a>
      </span>
      ${n.children.length ? renderModuleTree(n.children) : ""}
    </li>`).join("")}</ul>`;

  c.innerHTML = `<h2 class="mb">项目管理</h2>
  <div class="card"><div class="row">
    <input id="p-key" placeholder="项目 key（唯一英文标识）">
    <input id="p-name" placeholder="项目名称">
    <button id="btn-add-project">新建项目</button>
  </div></div>
  ${state.projects.map(p => `<div class="card">
    <h3>${esc(p.name)} <span class="muted">${esc(p.key)} · ${p.bug_count} 个 bug</span>
      <span style="float:right">
        <a href="#" data-add-module="${p.id}">加模块</a>
        <a href="#" data-add-milestone="${p.id}">加里程碑</a>
        ${p.bug_count === 0 ? `<a href="#" data-del-project="${p.id}">删除项目</a>` : ""}
      </span></h3>
    <div class="row" style="align-items:flex-start">
      <div class="grow module-tree">${p.modules.length ? renderModuleTree(p.modules) : '<span class="muted">无模块</span>'}</div>
      <div class="grow"><div class="k muted">里程碑</div>
        ${p.milestones.map(m => `<div class="row" style="padding:2px 0">
          <span class="grow">${esc(m.name)} <span class="muted">${esc(m.due_date)}</span></span>
          <span class="tag">${m.status === "open" ? "进行中" : "已关闭"}</span>
          <a href="#" data-toggle-ms="${m.id}" data-name="${esc(m.name)}" data-due="${esc(m.due_date)}" data-status="${m.status}">${m.status === "open" ? "关闭" : "重开"}</a>
          <a href="#" data-del-ms="${m.id}">删除</a>
        </div>`).join("") || '<span class="muted">无里程碑</span>'}</div>
    </div>
  </div>`).join("")}`;

  $("#btn-add-project").onclick = () => guard(async () => {
    const key = $("#p-key").value.trim(), name = $("#p-name").value.trim();
    if (!key || !name) { toast("key 与名称必填", true); return; }
    await api.post("/api/projects", { key, name });
    toast("已创建"); await renderProjects();
  });
  $$("[data-add-module], [data-add-child]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      const name = prompt("模块名称：");
      if (!name) return;
      const projectId = a.dataset.addModule ||
        state.projects.find(p => JSON.stringify(p.modules).includes(`"id":${a.dataset.addChild}`)).id;
      const body = { name };
      if (a.dataset.addChild) body.parent_id = +a.dataset.addChild;
      await api.post(`/api/projects/${projectId}/modules`, body);
      await renderProjects();
    });
  });
  $$("[data-ren-module]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      const name = prompt("新名称：", a.dataset.name);
      if (!name) return;
      await api.patch("/api/modules/" + a.dataset.renModule, { name, parent_id: null, sort: 0 });
      await renderProjects();
    });
  });
  $$("[data-del-module]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      if (!confirm("确认删除该模块？")) return;
      await api.del("/api/modules/" + a.dataset.delModule);
      await renderProjects();
    });
  });
  $$("[data-del-project]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      if (!confirm("确认删除该项目？")) return;
      await api.del("/api/projects/" + a.dataset.delProject);
      await renderProjects();
    });
  });
  $$("[data-add-milestone]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      const name = prompt("里程碑名称：");
      if (!name) return;
      const due = prompt("截止日期（YYYY-MM-DD，可空）：") || "";
      await api.post(`/api/projects/${a.dataset.addMilestone}/milestones`, { name, due_date: due });
      await renderProjects();
    });
  });
  $$("[data-toggle-ms]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      await api.patch("/api/milestones/" + a.dataset.toggleMs, {
        name: a.dataset.name, due_date: a.dataset.due,
        status: a.dataset.status === "open" ? "closed" : "open",
      });
      await renderProjects();
    });
  });
  $$("[data-del-ms]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      if (!confirm("确认删除该里程碑？")) return;
      await api.del("/api/milestones/" + a.dataset.delMs);
      await renderProjects();
    });
  });
}

// ---------- 用户管理（admin） ----------
async function renderUsers() {
  const users = await api.get("/api/users");
  const c = $("#content");
  c.innerHTML = `<h2 class="mb">用户管理</h2>
  <div class="card"><h3>新建用户</h3><div class="row">
    <input id="u-name" placeholder="用户名">
    <input id="u-display" placeholder="显示名">
    <input id="u-pass" placeholder="初始密码（≥6 位）">
    <select id="u-role">${Object.entries(ROLE_CN).map(([k, v]) =>
      `<option value="${k}">${v}</option>`).join("")}</select>
    <button id="btn-add-user">创建</button>
  </div></div>
  <div class="card"><table>
    <tr><th>ID</th><th>用户名</th><th>显示名</th><th>角色</th><th>状态</th><th>操作</th></tr>
    ${users.map(u => `<tr>
      <td>${u.id}</td><td>${esc(u.username)}</td><td>${esc(u.display_name)}</td>
      <td><select data-role-user="${u.id}" ${u.id === state.user.id ? "disabled" : ""}>
        ${Object.entries(ROLE_CN).map(([k, v]) =>
          `<option value="${k}" ${k === u.role ? "selected" : ""}>${v}</option>`).join("")}
      </select></td>
      <td>${u.active ? '<span class="tag status-verified">启用</span>' : '<span class="tag status-rejected">停用</span>'}
        ${u.must_change_password ? '<span class="muted">待改密</span>' : ""}</td>
      <td>
        ${u.id !== state.user.id ? `<a href="#" data-toggle-user="${u.id}" data-active="${u.active}">${u.active ? "停用" : "启用"}</a>` : ""}
        <a href="#" data-reset-pw="${u.id}">重置密码</a>
      </td></tr>`).join("")}
  </table></div>`;

  $("#btn-add-user").onclick = () => guard(async () => {
    await api.post("/api/users", {
      username: $("#u-name").value.trim(), display_name: $("#u-display").value.trim(),
      password: $("#u-pass").value, role: $("#u-role").value,
    });
    toast("已创建"); await renderUsers();
  });
  $$("[data-role-user]").forEach(sel => sel.onchange = () => guard(async () => {
    await api.patch("/api/users/" + sel.dataset.roleUser, { role: sel.value });
    toast("角色已更新");
  }));
  $$("[data-toggle-user]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      await api.patch("/api/users/" + a.dataset.toggleUser,
        { active: a.dataset.active !== "true" });
      await renderUsers();
    });
  });
  $$("[data-reset-pw]").forEach(a => a.onclick = e => {
    e.preventDefault();
    guard(async () => {
      const pw = prompt("新密码（≥6 位，用户首次登录需改密）：");
      if (!pw) return;
      await api.patch("/api/users/" + a.dataset.resetPw, { password: pw });
      toast("已重置");
    });
  });
}

// ---------- 通知中心 ----------
async function renderNotifications() {
  const data = await api.get("/api/notifications?size=100");
  const c = $("#content");
  c.innerHTML = `<div class="row mb"><h2 class="grow">通知中心</h2>
    <button class="secondary" id="btn-read-all">全部已读</button></div>
  <div class="card">
    ${data.items.map(n => `<div class="notif-item ${n.read ? "" : "unread"}" data-nid="${n.id}" data-bug="${n.bug_id || ""}">
      <div>${esc(n.message)}</div>
      <div class="muted">${fmtTime(n.created_at)} · ${n.type === "mention" ? "提及" : "变更"}</div>
    </div>`).join("") || '<div class="muted">暂无通知</div>'}
  </div>`;
  $("#btn-read-all").onclick = () => guard(async () => {
    await api.post("/api/notifications/read-all");
    await renderNotifications(); refreshBadge();
  });
  $$(".notif-item").forEach(el => el.onclick = () => guard(async () => {
    await api.post(`/api/notifications/${el.dataset.nid}/read`);
    if (el.dataset.bug) location.hash = "#/bugs/" + el.dataset.bug;
    else { await renderNotifications(); refreshBadge(); }
  }));
}

// ---------- 设置 ----------
async function renderSettings() {
  const c = $("#content");
  const isAdmin = state.user.role === "admin";
  c.innerHTML = `<h2 class="mb">设置</h2>
  <div class="card"><h3>修改我的密码</h3><div class="row">
    <input id="s-old" type="password" placeholder="原密码">
    <input id="s-new" type="password" placeholder="新密码（≥6 位）">
    <button id="btn-chpw">修改</button></div></div>
  ${isAdmin ? `
  <div class="card"><h3>标签管理</h3>
    <div class="row mb">${state.labels.map(l =>
      `<span class="tag" style="border-color:${esc(l.color)}">${esc(l.name)}
        <a href="#" data-del-label="${l.id}" style="margin-left:4px">×</a></span>`).join("")}</div>
    <div class="row"><input id="label-name" placeholder="新标签名">
      <input id="label-color" type="color" value="#4f8cff" style="width:40px;padding:2px">
      <button id="btn-add-label">添加</button></div></div>
  <div class="card"><h3>数据备份 / 迁移</h3>
    <div class="row">
      <button id="btn-backup">创建备份</button>
      <button class="secondary" id="btn-export-json">导出全量 JSON</button>
      <input type="file" id="import-json-file" accept=".json">
      <button class="secondary" id="btn-import-json">导入 JSON（清空重建）</button>
    </div>
    <p class="hint" style="margin-top:8px;color:var(--text-dim);font-size:12px">
      「创建备份」在线一致拷贝主库 + 附件到 data/backups/&lt;时间戳&gt;/（WAL 安全）。
    </p>
    <div class="row mt">
      <select id="csv-project">${projectOptions()}</select>
      <input type="file" id="import-csv-file" accept=".csv">
      <button class="secondary" id="btn-import-csv">导入 CSV 到该项目</button>
    </div>
    <div class="muted mt">⚠️ JSON 导入会清空现有全部数据；导出文件含密码哈希，注意保管。</div></div>
  <div class="card"><h3>集成</h3>
    <div class="muted">
      <p>git 联动：在目标仓库执行 <code>CloudSim/tools/bugtracker/scripts/install_hook.ps1 -RepoPath &lt;仓库路径&gt;</code>，
        之后 commit message 中 <code>fix #ID</code> 自动关闭对应 bug。</p>
      <p class="mt">CI 联动：<code>python tools/bugtracker/scripts/ci_failure_to_bug.py</code>
        解析最新 artifacts/checks 批次并生成 bug 草稿。</p>
      <p class="mt">集成令牌在 <code>tools/bugtracker/.env</code> 的 <code>BT_INTEGRATION_TOKEN</code> 配置。</p>
    </div></div>` : ""}`;

  $("#btn-chpw").onclick = () => guard(async () => {
    await api.post("/api/auth/password", {
      old_password: $("#s-old").value, new_password: $("#s-new").value });
    toast("密码已修改");
  });

  if (isAdmin) {
    $("#btn-add-label").onclick = () => guard(async () => {
      const name = $("#label-name").value.trim();
      if (!name) return;
      await api.post("/api/labels", { name, color: $("#label-color").value });
      await preload(); await renderSettings();
    });
    $$("[data-del-label]").forEach(a => a.onclick = e => {
      e.preventDefault();
      guard(async () => {
        await api.del("/api/labels/" + a.dataset.delLabel);
        await preload(); await renderSettings();
      });
    });
    $("#btn-backup").onclick = () => guard(async () => {
      const r = await api.post("/api/backup", {});
      toast("备份完成：" + r.path);
    });
    $("#btn-export-json").onclick = () => guard(async () => {
      const data = await api.get("/api/export/json");
      const blob = new Blob([JSON.stringify(data, null, 1)],
        { type: "application/json" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = `bugtracker-backup-${new Date().toISOString().slice(0, 10)}.json`;
      a.click();
    });
    $("#btn-import-json").onclick = () => guard(async () => {
      const file = $("#import-json-file").files[0];
      if (!file) { toast("请选择文件", true); return; }
      if (!confirm("⚠️ 导入将清空现有全部数据，确认继续？")) return;
      const text = await file.text();
      await api.post("/api/import/json?confirm=yes", JSON.parse(text));
      toast("导入完成，请重新登录");
      await api.post("/api/auth/logout").catch(() => {});
      state.user = null;
      location.hash = "#/login";
    });
    $("#btn-import-csv").onclick = () => guard(async () => {
      const file = $("#import-csv-file").files[0];
      if (!file) { toast("请选择 CSV 文件", true); return; }
      const fd = new FormData();
      fd.append("file", file);
      const r = await api.upload(
        `/api/import/csv?project_id=${$("#csv-project").value}`, fd);
      toast(`已导入 ${r.imported} 条`);
    });
  }
}

// ---------- 路由 ----------
async function route() {
  const hash = location.hash.slice(1) || "/dashboard";
  if (hash === "/login") { renderLogin(); return; }

  if (!state.user) {
    try { state.user = await api.get("/api/auth/me"); }
    catch { renderLogin(); return; }
  }
  if (state.user.must_change_password) { renderChangePassword(true); return; }

  renderShell();
  setActiveNav(hash);
  await preload();

  await guard(async () => {
    let m;
    if (hash === "/dashboard") await renderDashboard();
    else if (hash === "/bugs") await renderBugList();
    else if (hash === "/bugs/new") await renderBugEdit(null);
    else if ((m = hash.match(/^\/bugs\/(\d+)$/))) await renderBugDetail(+m[1]);
    else if ((m = hash.match(/^\/bugs\/(\d+)\/edit$/))) await renderBugEdit(+m[1]);
    else if (hash === "/projects") await renderProjects();
    else if (hash === "/users") await renderUsers();
    else if (hash === "/notifications") await renderNotifications();
    else if (hash === "/settings") await renderSettings();
    else location.hash = "#/dashboard";
  });
}

window.addEventListener("hashchange", route);
window.addEventListener("DOMContentLoaded", route);
