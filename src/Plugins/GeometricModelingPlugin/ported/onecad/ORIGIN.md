# ORIGIN — OneCAD ported modules

- Upstream: https://github.com/andrejvysny/OneCAD
- License: MIT (see LICENSE)
- Copied trees: `src/core/sketch`, `src/app/commands`, `src/app/history`, `src/core/modeling`
- CloudSim uses adapted wrappers under plugin `inc/`/`source/` (FeatureDocument, CommandStack, SketchDocument)
  that follow OneCAD Command/Sketch patterns; full OneCAD app Document is not linked (Qt6/OCC coupling).
