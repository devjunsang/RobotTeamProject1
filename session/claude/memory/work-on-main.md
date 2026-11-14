---
name: work-on-main
description: "Since 2026-11-14 every task gets its own new branch + commit + push (never main); others' uncommitted edits go to separate wip/session-* branches"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 6d8a2b38-14a4-41ec-b12b-9fa90994d204
  modified: 2026-11-14T03:25:40.228Z
---

2026-11-14: the user made it policy to commit and push after every task, "바로 main에 push하지는 말고 branch에 push하자", then "다른 사람의 코드 수정도 매번 브랜치를 따서 커밋하고 푸쉬하도록 정책화해라." Replaces the 2026-11-13 rule of working directly on main. Written in AGENTS.md "Git Workflow".

**Why:** Everything should be saved remotely and traceable per task, while main stays clean (teammate rewq171717 also pushes to main; other agent sessions edit the same working tree).
**How to apply:**
- Each task: new branch (feat/fix/docs/<topic>) from up-to-date main or the unmerged branch it depends on; run focused tests; commit (Korean message); push. Never push to main; PR only when asked; no force-push without approval.
- Before starting a task and again right before my own commit: if `git status` shows edits I didn't make, create `wip/session-YYYYMMDD-HHMM` from where I am, commit them separately (say they came from another session, what changed, which tests ran), push, then branch my own work from there. Never mix them into my commit.
- Never switch to a branch that would revert someone's in-progress edits; branch from the current position (stacked branches).
- Push as devjunsang ([[github-account]]): `git -c credential.helper= -c credential.helper='!gh auth git-credential' push -u origin <branch>` with gh active account devjunsang.
