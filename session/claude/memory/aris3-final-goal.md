---
name: aris3-final-goal
description: "ARIS3 final goal is a gesture-interactive pick-and-place robot (MediaPipe hand pointing, required by user), decided 2026-11-09"
metadata:
  node_type: memory
  type: project
  originSessionId: 6a453bd9-8c50-42c9-a3d5-8a28afceaa94
  modified: 2026-11-09T11:51:41.466Z
---

Final goal (user, 2026-11-09): an interactive robot — the user points at an object with a hand gesture, the robot picks it, then places it at the coordinate the next gesture points to. User explicitly said gestures **must use MediaPipe** (2026-11-09).

Decisions:
- Gesture flow: point + hold ~1 s to select (highlight) -> confirm gesture (fist = pick, open palm = place). Two steps so misdetections never move the robot; on the real robot a confirm dialog is added on top.
- As of 2026-11-09 the user is working on the real E0509 with an overhead D455 (color via USB) and a pixel->robot homography calibration in aris3_gui real mode; gesture was implemented there first (aris3_gesture package, MediaPipe 1.x Tasks API in ~/venvs/aris3_gesture), not the earlier "laptop webcam over Gazebo view" stage.

**Why:** the masterplan's voice/color commands are now secondary; the project is reframed as human-robot interaction by gesture.
**How to apply:** prioritize work that serves the gesture loop; keep voice as optional complement. Related: [[github-account]]
