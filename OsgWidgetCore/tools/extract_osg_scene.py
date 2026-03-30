"""Extract OsgScene.cpp line ranges from Widget OsgWidget.cpp (exclude Qt-only / controller glue)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "Widget" / "source" / "OsgWidget.cpp"
OUT = ROOT / "OsgWidgetCore" / "source" / "OsgScene.cpp"

# Inclusive 1-based line ranges to DROP (Qt file IO, UI init, eventFilter, capture delegates).
EXCLUDE = [
    (275, 291),  # ctor/dtor
    (1260, 1419),  # importModelFile, loadXyz, loadPly, importPointCloudFile
    (1421, 1486),  # initUi, initViewer
    (1777, 1790),  # pointCloudPluginReport (QString)
    (3196, 3265),  # eventFilter
    (3325, 3358),  # capture/load delegates
]


def in_exclude(n: int) -> bool:
    for a, b in EXCLUDE:
        if a <= n <= b:
            return True
    return False


def main():
    lines = SRC.read_text(encoding="utf-8").splitlines(keepends=True)
    kept = []
    for i, line in enumerate(lines, start=1):
        if not in_exclude(i):
            kept.append(line)
    text = "".join(kept)
    text = text.replace('#include "OsgWidget.h"', '#include "OsgScene.h"')
    text = text.replace("OsgWidget::", "OsgScene::")
    text = text.replace("OsgScene::DragAxis OsgScene::pickAxisAtScreenPos", "OsgScene::DragAxis OsgScene::pickAxisAtScreenPosFloat")
    # Fix accidental rename of AnnotationSnapshot
    text = text.replace("QList<OsgScene::AnnotationSnapshot>", "QList<OsgWidget::AnnotationSnapshot>")
    text = text.replace("OsgScene::AnnotationSnapshot", "OsgWidget::AnnotationSnapshot")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    print("Wrote", OUT, "lines", len(kept))


if __name__ == "__main__":
    main()
