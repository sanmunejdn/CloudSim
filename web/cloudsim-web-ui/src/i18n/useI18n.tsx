import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { apiJson } from "../api/client";

type I18nPayload = { lang?: string; strings?: Record<string, string> };

type I18nCtx = {
  lang: string;
  setLang: (lang: string) => void;
  t: (key: string, fallback?: string) => string;
  theme: "light" | "dark";
  setTheme: (theme: "light" | "dark") => void;
  toggleTheme: () => void;
};

const LANG_KEY = "cloudsim.lang";
const THEME_KEY = "cloudsim.theme";

const Ctx = createContext<I18nCtx | null>(null);

function readTheme(): "light" | "dark" {
  const v = localStorage.getItem(THEME_KEY);
  return v === "dark" ? "dark" : "light";
}

function applyThemeToDom(theme: "light" | "dark") {
  document.documentElement.dataset.theme = theme;
}

export function I18nProvider({ children }: { children: ReactNode }) {
  const [lang, setLangState] = useState(() => localStorage.getItem(LANG_KEY) || "zh");
  const [strings, setStrings] = useState<Record<string, string>>({});
  const [theme, setThemeState] = useState<"light" | "dark">(() => readTheme());

  useEffect(() => {
    applyThemeToDom(theme);
    localStorage.setItem(THEME_KEY, theme);
  }, [theme]);

  useEffect(() => {
    let cancelled = false;
    void apiJson<I18nPayload>(`/api/i18n/${encodeURIComponent(lang)}`).then((r) => {
      if (!cancelled) setStrings(r.strings || {});
    });
    return () => {
      cancelled = true;
    };
  }, [lang]);

  const setLang = useCallback((l: string) => {
    localStorage.setItem(LANG_KEY, l);
    setLangState(l);
  }, []);

  const setTheme = useCallback((t: "light" | "dark") => setThemeState(t), []);

  const toggleTheme = useCallback(() => setThemeState((t) => (t === "light" ? "dark" : "light")), []);

  const t = useCallback(
    (key: string, fallback?: string) => strings[key] || fallback || key,
    [strings],
  );

  const value = useMemo(
    () => ({ lang, setLang, t, theme, setTheme, toggleTheme }),
    [lang, setLang, t, theme, setTheme, toggleTheme],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useI18n() {
  const v = useContext(Ctx);
  if (!v) throw new Error("I18nProvider missing");
  return v;
}
