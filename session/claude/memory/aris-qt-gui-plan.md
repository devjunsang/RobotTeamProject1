---
name: aris-qt-gui-plan
description: "Two GUI versions kept side by side — PyQt5 aris3_gui and new C++ Qt5 \"ARIS\" (aris3_gui_cpp), Iron Man suit color theme"
metadata:
  node_type: memory
  type: project
  originSessionId: 01245bf1-db1b-457a-9a32-b0b392adef89
  modified: 2026-11-10T11:06:13.117Z
---

Decided 2026-11-10: keep TWO GUI versions long-term in the same repo/branch as separate packages —
`aris3_gui` (Python/PyQt5, tag `v1-pyqt`) and `aris3_gui_cpp` (C++ Qt5 Widgets). Development on branch
`feat/qt-gui`, merged when working (user first thought branches can't be merged; explained packages-side-by-side).
Real-robot logic (e0509_manual RobotLink, real_pick) moves first into a shared ROS node `aris3_real_bridge`
so both GUIs use the same tested safety code.

The C++ app is named **ARIS**; theme should feel like an Iron Man suit (dark base, hot-rod red, gold, arc-reactor cyan).

**Why:** user wants both versions; real safety logic was validated on the real robot and must not be rewritten.
**How to apply:** don't propose a long-lived divergent branch; shared fixes go in shared packages. Keep stop/error
colors distinguishable from the red theme. Related: [[aris3-final-goal]]
