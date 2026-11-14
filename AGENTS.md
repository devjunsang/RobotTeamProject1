# Repository Guidance

## Project Shape

- This repository contains ROS 2 packages for the ARIS3 pick-and-place robot project. Read the `ROLE.txt` in the package you are changing before editing it; it documents that package's responsibility, status, and constraints.
- Keep package boundaries clear. Shared ROS interfaces belong in `aris3_interfaces`; adapters belong in their bridge packages; orchestration belongs in `aris3_task_manager`.
- Treat existing message, service, and action definitions as cross-package contracts. If one changes, find and update all producers and consumers, then rebuild the affected packages.
- The simulation stack targets ROS 2 Jazzy and Gazebo Harmonic. Check a package's README and launch files for its actual runtime dependencies rather than assuming every package runs standalone.

## Build and Test

- Build from the ROS 2 workspace that contains these packages, with that workspace's setup sourced. For the task-manager stack, the documented build is:

  ```bash
  colcon build --packages-select aris3_interfaces aris3_sim_bridge aris3_task_manager
  ```

- Run focused package tests when possible. For the task manager:

  ```bash
  colcon test --packages-select aris3_task_manager
  colcon test-result --test-result-base build/aris3_task_manager --verbose
  ```

- The integration script `scripts/test_task_manager_sim.sh` requires the Gazebo workcell and Task Manager to be running. Do not treat it as a standalone unit test.
- After changing ROS interfaces, rebuild the interface package and every affected dependent package.

## Git Workflow

- Commit after every completed task (one logical change per commit), then push. Do not leave finished work uncommitted.
- Every task gets its own new branch: `feat/<topic>`, `fix/<topic>` or `docs/<topic>`. Create it from an up-to-date `main`,
  or from the unmerged branch the work depends on (stacked branches). Never push to `main` directly; merge into `main`
  only through a pull request when the user asks.
- Other people's changes (uncommitted edits in the working tree that you did not make - a teammate or another agent
  session): before starting your task, and again before your own commit, put them on their own new branch
  `wip/session-YYYYMMDD-HHMM`, commit them separately (message says they came from another session and lists what
  changed and which tests ran), and push. Never mix them into your own commit. Then branch your own task from there.
- Do not switch to a branch that would revert someone else's in-progress edits in the working tree; branch from where you are.
- Before committing, run the focused tests for the packages you changed and make sure they pass; say so in the commit body if a test could not run.
- Update the relevant README, `ROLE.txt`, and `docs/` together with the code in the same commit (or the commit right after),
  and regenerate `docs/*.html` with `scripts/docs_md_to_html.py` when a `.md` in `docs/` changes.
- Never force-push or rewrite pushed history without the user's explicit approval.

## Implementation Notes

- Preserve the existing C++17, `rclcpp`, and ROS 2 action conventions in `aris3_task_manager`; follow neighboring code and tests.
- Keep simulation-specific compatibility in `aris3_sim_bridge`. The Task Manager should depend on ARIS3 interfaces, not on simulator-specific detection or gripper APIs.
- Preserve existing public topic, service, action, and TF names unless the task explicitly requires a contract change.
- Before changing robot motion behavior, inspect the task-manager configuration and recovery/cancellation paths. Physical-robot support is not established; simulation success does not imply real-robot readiness.
- Avoid unrelated cleanup. Update the relevant package README or tests when behavior or usage changes.