---
name: real-robot-schedule
description: No real E0509 until Friday 2026-11-13; build/verify the real path in a real-like simulation (DRCF emulator) meanwhile
metadata:
  node_type: memory
  type: project
  originSessionId: 01245bf1-db1b-457a-9a32-b0b392adef89
  modified: 2026-11-10T12:28:47.218Z
---

As of 2026-11-10 the real robot is unavailable; the next real retest is possible on Friday 2026-11-13.
Until then the user wants everything verified "as much as possible in simulation, in an environment similar to
the real one" — i.e. drive aris3_real_bridge (RobotLink) against the Doosan DRCF emulator (Docker
doosanrobot/dsr_emulator:3.0.1) with Gazebo, sim gripper and sim overhead camera, not just the MoveIt sim.

**Why:** the GUI's real mode was just moved onto aris3_real_bridge and has never run against a robot.
**How to apply:** prefer verifying through the real code path (Doosan services) in sim; keep a list of what the
emulator cannot reproduce (collision detection, safety stops, gripper DRL) for the Friday checklist.
Related: [[aris-qt-gui-plan]], [[e0509-manual-ws-copy]]
