---
name: korean-progress-updates
description: "ALL user-facing text must be Korean - final answers, progress notes, tables, reports. Drifted to English twice; never again"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 2638165a-38a6-4590-bc29-0c656e2593af
  modified: 2026-11-10T13:39:24.308Z
---

Write every user-visible message in Korean: final answers, intermediate status lines between tool calls,
findings, tables, summaries, and questions. Technical identifiers (file names, topics, commands) stay as-is.

**Why:** user asked "중간과정은 한글로 알려줘" (2026-11-09), and again "영어말고 한글로 좀 해줘" (2026-11-10) after
several long replies in a session drifted into English (status updates and final reports were written in English).
**How to apply:** before sending any text, check it is Korean - especially in long sessions, after many tool calls,
and when writing reports/progress tables. Code comments and commit messages are already Korean in this repo; keep that.
Related: [[code-comments]]
