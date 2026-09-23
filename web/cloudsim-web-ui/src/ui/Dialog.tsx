/// @file Dialog.tsx
/// @brief 统一确认/输入对话框（替代 window.confirm / prompt）

import { useCallback, useEffect, useId, useRef, useState, type ReactNode } from "react";
import { createRoot } from "react-dom/client";

type ConfirmOpts = {
  title?: string;
  message: string;
  confirmText?: string;
  cancelText?: string;
  danger?: boolean;
};

type PromptOpts = {
  title?: string;
  message?: string;
  defaultValue?: string;
  placeholder?: string;
  confirmText?: string;
  cancelText?: string;
};

function DialogShell({
  title,
  children,
  onCancel,
}: {
  title: string;
  children: ReactNode;
  onCancel: () => void;
}) {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onCancel();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [onCancel]);

  return (
    <div className="dlg-mask" onClick={onCancel}>
      <div className="dlg" role="dialog" aria-modal="true" onClick={(e) => e.stopPropagation()}>
        <h3>{title}</h3>
        {children}
      </div>
    </div>
  );
}

function ConfirmBody({ opts, resolve }: { opts: ConfirmOpts; resolve: (v: boolean) => void }) {
  const cancel = useCallback(() => resolve(false), [resolve]);
  return (
    <DialogShell title={opts.title || "确认"} onCancel={cancel}>
      <p className="dlg-hint" style={{ whiteSpace: "pre-wrap" }}>
        {opts.message}
      </p>
      <div className="dlg-actions">
        <button type="button" onClick={cancel}>
          {opts.cancelText || "取消"}
        </button>
        <button type="button" className="primary" onClick={() => resolve(true)}>
          {opts.confirmText || "确定"}
        </button>
      </div>
    </DialogShell>
  );
}

function PromptBody({ opts, resolve }: { opts: PromptOpts; resolve: (v: string | null) => void }) {
  const [value, setValue] = useState(opts.defaultValue ?? "");
  const inputRef = useRef<HTMLInputElement>(null);
  const id = useId();
  const cancel = useCallback(() => resolve(null), [resolve]);

  useEffect(() => {
    inputRef.current?.focus();
    inputRef.current?.select();
  }, []);

  return (
    <DialogShell title={opts.title || "输入"} onCancel={cancel}>
      {opts.message ? <p className="dlg-hint">{opts.message}</p> : null}
      <label className="dlg-field" htmlFor={id}>
        <input
          id={id}
          ref={inputRef}
          value={value}
          placeholder={opts.placeholder}
          onChange={(e) => setValue(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") resolve(value);
          }}
        />
      </label>
      <div className="dlg-actions">
        <button type="button" onClick={cancel}>
          {opts.cancelText || "取消"}
        </button>
        <button type="button" className="primary" onClick={() => resolve(value)}>
          {opts.confirmText || "确定"}
        </button>
      </div>
    </DialogShell>
  );
}

/** 异步确认框 */
export function showConfirm(opts: ConfirmOpts): Promise<boolean> {
  return new Promise((resolve) => {
    const host = document.createElement("div");
    document.body.appendChild(host);
    const root = createRoot(host);
    let settled = false;
    const finish = (v: boolean) => {
      if (settled) return;
      settled = true;
      root.unmount();
      host.remove();
      resolve(v);
    };
    root.render(<ConfirmBody opts={opts} resolve={finish} />);
  });
}

/** 异步输入框；取消返回 null */
export function showPrompt(opts: PromptOpts): Promise<string | null> {
  return new Promise((resolve) => {
    const host = document.createElement("div");
    document.body.appendChild(host);
    const root = createRoot(host);
    let settled = false;
    const finish = (v: string | null) => {
      if (settled) return;
      settled = true;
      root.unmount();
      host.remove();
      resolve(v);
    };
    root.render(<PromptBody opts={opts} resolve={finish} />);
  });
}
