#!/usr/bin/env bash
# 저장소 session/claude/ -> 이 PC 의 Claude Code 대화 기록·메모리 폴더로 (2026-11-14)
# 사용: bash session/restore.sh   그다음 저장소 폴더에서 claude --resume (또는 --continue)
# 폴더 이름은 이 PC 의 저장소 절대 경로로 정한다 (경로가 원래와 달라도 됨). 기존 폴더는 .bak-시각 으로 옮김.
set -euo pipefail
REPO="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
ENC="$(printf '%s' "$REPO" | sed 's/[^A-Za-z0-9]/-/g')"
DST="$HOME/.claude/projects/$ENC"
[ -d "$REPO/session/claude" ] || { echo "session/claude 없음"; exit 1; }
if [ -d "$DST" ]; then mv "$DST" "$DST.bak-$(date +%Y%m%d-%H%M%S)"; echo "기존 폴더 백업: $DST.bak-*"; fi
mkdir -p "$HOME/.claude/projects"
cp -a "$REPO/session/claude" "$DST"
echo "복원함: $DST - 저장소 폴더에서 claude --resume"
