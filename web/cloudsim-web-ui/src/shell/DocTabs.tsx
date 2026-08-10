import { useProject } from "../state/projectStore";

export default function DocTabs() {
  const { docTitle } = useProject();
  return (
    <div className="doc-tabs">
      <div className="doc-tab active">
        <span>{docTitle}</span>
        <button type="button" className="doc-close" title="关闭" disabled>
          ×
        </button>
      </div>
    </div>
  );
}
