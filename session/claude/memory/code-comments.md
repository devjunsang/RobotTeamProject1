---
name: code-comments
description: "Always put comments in every piece of code generated (Korean, explaining what each function/block does and why)"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 01245bf1-db1b-457a-9a32-b0b392adef89
  modified: 2026-11-10T12:07:15.981Z
---

When writing code, always add comments to each piece: every function/class/method gets a docstring or comment,
and non-obvious blocks get a line explaining what and why. Write them in Korean, matching the repo's style
(e.g. aris3_task_manager, e0509_manual robot_link.py).

**Why:** user asked explicitly on 2026-11-10 ("코드 생성시 각 코드마다 주석 꼭 넣어주고"); the team studies the
code to learn it (docs/*학습가이드.md), so commented code is part of the deliverable.
**How to apply:** applies to all languages (Python, C++/Qt for ARIS, launch files, tests, CMake). Includes test
functions (say what each test checks). Related: [[korean-progress-updates]], [[aris-qt-gui-plan]]
