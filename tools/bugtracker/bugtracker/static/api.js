/* fetch 封装：统一错误处理，401 跳登录 */
const api = {
  async request(method, url, body, isForm) {
    const opts = { method, headers: {} };
    if (body !== undefined) {
      if (isForm) {
        opts.body = body;
      } else {
        opts.headers["Content-Type"] = "application/json";
        opts.body = JSON.stringify(body);
      }
    }
    const resp = await fetch(url, opts);
    if (resp.status === 401 && !url.includes("/auth/login")) {
      location.hash = "#/login";
      throw new Error("未登录或会话已过期");
    }
    const text = await resp.text();
    let data = null;
    try { data = text ? JSON.parse(text) : null; } catch { data = text; }
    if (!resp.ok) {
      let detail = resp.statusText;
      if (data && data.detail) {
        detail = typeof data.detail === "string"
          ? data.detail
          : (data.detail.detail || JSON.stringify(data.detail));
      }
      throw new Error(detail);
    }
    return data;
  },
  get(url) { return this.request("GET", url); },
  post(url, body) { return this.request("POST", url, body); },
  patch(url, body) { return this.request("PATCH", url, body); },
  del(url) { return this.request("DELETE", url); },
  upload(url, formData) { return this.request("POST", url, formData, true); },
};
