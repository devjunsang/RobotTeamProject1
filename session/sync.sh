#!/usr/bin/env bash
# Claude Code 대화 기록·메모리 -> 저장소 session/claude/ 로 복사 (2026-11-14)
# 사용: bash session/sync.sh      (저장소 안 어디서든). 복사한 뒤 git add session && git commit && git push 는 직접.
# 공개 저장소라 복사본에서 비밀을 가린다: Anthropic API 키(sk-ant-...), GitHub 토큰, 본인(devjunsang) 외 gmail 주소.
# 원본(~/.claude/projects/...)은 건드리지 않는다.
set -euo pipefail
REPO="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
ENC="$(printf '%s' "$REPO" | sed 's/[^A-Za-z0-9]/-/g')"       # Claude Code 가 저장소 경로로 정한 폴더 이름
SRC="$HOME/.claude/projects/$ENC"
DST="$REPO/session/claude"
[ -d "$SRC" ] || { echo "대화 기록 폴더 없음: $SRC"; exit 1; }
rm -rf "$DST"
mkdir -p "$DST"
cp -a "$SRC"/. "$DST"/
# 비밀 가리기 (텍스트 파일만): API 키·토큰, 그리고 본인(devjunsang) 것이 아닌 gmail 주소 (정규식으로 쓴 것 포함)
grep -rlIE 'sk-ant-|gh[pousr]_|github_pat_|@gmail' "$DST" | while read -r f; do
  perl -pi -e 's/sk-ant-[A-Za-z0-9_-]{20,}/sk-ant-REDACTED/g;
               s/gh[pousr]_[A-Za-z0-9]{30,}/gh_REDACTED/g;
               s/github_pat_[A-Za-z0-9_]{30,}/github_pat_REDACTED/g;
               s/(?<![A-Za-z0-9._%+-])(?!n?devjunsang\@)[A-Za-z0-9._%+-]+\@gmail(?:\\+)?\.com/REDACTED\@example.com/g;
               s/(?<![A-Za-z0-9._%+-])(?!n?devjunsang\@)(?!REDACTED\@)[A-Za-z0-9._%+-]+\@gmail/REDACTED\@gmail/g' "$f"
done
left="$(grep -rIcE 'sk-ant-[A-Za-z0-9_-]{20,}|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}' "$DST" | grep -vc ':0$' || true)"
if [ "$left" != "0" ]; then echo "가리지 못한 비밀이 남음 - 커밋하지 마세요"; exit 2; fi
echo "복사함: $SRC -> $DST ($(du -sh "$DST" | cut -f1), 대화 $(ls "$DST"/*.jsonl | wc -l)개, 메모리 $(ls "$DST"/memory | wc -l)개). 비밀 가림 확인."
