---
name: github-account
description: User works only as devjunsang (MasterJun) on GitHub; ignore the demyank88 gh login on this PC
metadata:
  node_type: memory
  type: user
  originSessionId: 6a453bd9-8c50-42c9-a3d5-8a28afceaa94
  modified: 2026-11-08T13:07:44.899Z
---

The user is devjunsang on GitHub (git identity MasterJun <devjunsang@gmail.com>, set repo-locally in RobotTeamProject1). The `gh` CLI on this PC is logged in as demyank88 (a teammate's account); the user said to ignore it and work only as devjunsang.

**Why:** demyank88 belongs to someone else; pushes/PRs under it would be misattributed.
**How to apply:** Don't push, open PRs, or comment via gh as demyank88 without checking; suggest `! gh auth login` as devjunsang if a GitHub write is needed.
