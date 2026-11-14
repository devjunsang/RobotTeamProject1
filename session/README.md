# Claude Code 대화 기록·메모리 (session/)

이 저장소에서 Claude Code 로 작업한 대화 기록과 메모리를 다른 PC 에서 이어서 쓰려고 저장소에 넣어 둔다 (2026-11-14).

| 경로 | 내용 |
|---|---|
| `claude/*.jsonl` | 대화 기록 (한 파일 = 대화 하나). 이름 = 대화 ID |
| `claude/<대화 ID>/` | 그 대화의 하위 에이전트 기록 등 |
| `claude/memory/` | 메모리 (`MEMORY.md` 목록 + 항목 파일: GitHub 계정, 커밋 정책, 한국어 주석 규칙 …) |
| `sync.sh` | 이 PC 의 기록 -> `claude/` 로 복사 (비밀 가림) |
| `restore.sh` | `claude/` -> 이 PC 의 Claude Code 폴더로 복원 |

## 보안 - 공개 저장소

이 저장소는 공개다. `sync.sh` 는 복사본에서 다음을 가린다 (원본 `~/.claude/projects/…` 는 그대로).

- Anthropic API 키 `sk-ant-…` -> `sk-ant-REDACTED`
- GitHub 토큰 `ghp_…`·`gho_…`·`github_pat_…`
- 본인(devjunsang) 것이 아닌 gmail 주소 -> `REDACTED@…`

가리지 못한 키가 남으면 `sync.sh` 가 멈춘다. 그래도 대화 기록에는 로봇 컨트롤러 IP, PC 경로, 작업 내용이 그대로 있다.
비밀(키·비밀번호)을 채팅에 붙여 넣었다면 커밋 전에 `git diff --stat session` 과 `grep` 으로 한 번 더 본다.
**한 번 push 한 비밀은 히스토리에 남으므로, 새어 나갔으면 그 키를 바로 바꾼다(rotate).**

## 올리기 (이 PC -> 저장소)

```bash
cd ~/workspace/team/RobotTeamProject1
bash session/sync.sh
git switch -c docs/claude-sessions-$(date +%Y%m%d-%H%M)     # 정책: 작업마다 새 브랜치 (AGENTS.md Git Workflow)
git add session && git commit -m "docs(session): Claude 대화 기록 갱신" && git push -u origin HEAD
```

## 이어서 하기 (다른 PC)

```bash
git clone https://github.com/devjunsang/RobotTeamProject1.git ~/workspace/team/RobotTeamProject1
cd ~/workspace/team/RobotTeamProject1 && git switch <session/ 이 있는 브랜치>
bash session/restore.sh          # ~/.claude/projects/<이 저장소 경로 이름>/ 로 (기존 것은 .bak-시각)
claude --resume                  # 목록에서 대화를 골라 이어서 (또는 claude --continue)
```

- Claude Code 는 저장소 **절대 경로**로 폴더 이름을 정한다 (`/home/fc/workspace/team/RobotTeamProject1` ->
  `-home-fc-workspace-team-RobotTeamProject1`). `restore.sh` 가 그 PC 의 경로로 맞춘다.
- 기록 속 키는 가려져 있으니, 음성 LLM 등에 쓸 키는 새 PC 에서 `ANTHROPIC_API_KEY` 로 다시 넣는다.
- 저장소 밖 실행 환경(카메라 보정 `~/.config/aris3`, `~/ros2_ws/src/e0509_*`, 가상환경)은 여기 없다 -
  `~/aris3_migration_20261114.tar.gz` 묶음의 `README_옮기기.md` 참고.
