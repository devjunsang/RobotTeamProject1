---
name: e0509-manual-ws-copy
description: ~/ros2_ws/src/e0509_manual is a separate copy (not symlink) whose bringup.py has netcfg features missing from repo etc/e0509_manual
metadata:
  node_type: memory
  type: project
  originSessionId: 01245bf1-db1b-457a-9a32-b0b392adef89
  modified: 2026-11-10T12:05:20.756Z
---

As of 2026-11-10, `~/ros2_ws/src/e0509_manual` is a real directory, not a symlink to the repo's `etc/e0509_manual`.
robot_link.py and gripper_drl.py are identical, but the ws copy has extra `netcfg.py`, `settings_dialog.py`, and a
newer `bringup.py` (BringupManager gains `source=`, `set_target`, network path checks), plus differing gui.py/launch/setup.
The installed package (what aris3_gui / aris3_real_bridge import) comes from the ws copy.

**Why:** edits to the repo's etc/e0509_manual do NOT reach the running code; the netcfg work is not in git.
**How to apply:** when changing e0509_manual, check which copy is built; raise with the user whether to sync
the ws copy into the repo (or symlink it). Related: [[aris-qt-gui-plan]]
