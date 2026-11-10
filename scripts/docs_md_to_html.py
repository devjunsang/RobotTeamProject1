#!/usr/bin/env python3
"""docs/*.md -> 읽기 쉬운 HTML (2026-11-11).

원문(.md)은 그대로 두고, 같은 이름의 .html 을 만든다 (내용을 손으로 옮기지 않아 원문과 어긋나지 않음).
.md 를 고친 뒤 다시 돌리면 된다.

  python3 scripts/docs_md_to_html.py            # docs/ 의 모든 .md -> .html, docs/index.html (문서 목록)

필요: Python markdown 패키지 (pip install markdown). 시스템에 없으면 가상환경에 설치해 그 Python 으로 실행.
디자인은 docs/ARIS3_학습가이드.html 과 같다: 목차, 표·코드 상자, 밝게/어둡게 자동.
"""
import html
import re
import sys
from pathlib import Path

import markdown

DOCS = Path(__file__).resolve().parent.parent / 'docs'

CSS = r"""
:root {
  --bg: #fbf8f6; --surface: #ffffff; --fg: #231a1a; --muted: #6b5e5c; --line: #e4d9d4;
  --red: #a3141c; --gold: #9a6f05; --cyan: #0b7c88; --code-bg: #f3ece8; --quote: #fbefe9;
  --font-body: "IBM Plex Sans KR", "Noto Sans KR", "Apple SD Gothic Neo", sans-serif;
  --font-mono: "JetBrains Mono", "DejaVu Sans Mono", ui-monospace, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #120b0c; --surface: #1b1214; --fg: #efe6e3; --muted: #b3a29e; --line: #3a2a2a;
  --red: #ff6b70; --gold: #e8b931; --cyan: #5ff4ff; --code-bg: #24181a; --quote: #2c1615; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #120b0c; --surface: #1b1214; --fg: #efe6e3; --muted: #b3a29e; --line: #3a2a2a;
  --red: #ff6b70; --gold: #e8b931; --cyan: #5ff4ff; --code-bg: #24181a; --quote: #2c1615; color-scheme: dark }
body { background: var(--bg); color: var(--fg); font-family: var(--font-body); font-size: 16px; line-height: 1.75; }
.layout { max-width: 1180px; margin: 0 auto; display: grid; grid-template-columns: 250px minmax(0, 1fr); gap: 40px;
  padding-block: 32px 80px; padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); }
aside.toc { position: sticky; top: calc(env(safe-area-inset-top, 0px) + 16px); align-self: start; max-height: calc(100vh - 32px);
  overflow-y: auto; font-size: 13.5px; line-height: 1.5; border-right: 1px solid var(--line); padding-right: 16px; }
aside.toc .home { display: inline-block; margin-bottom: 12px; color: var(--muted); text-decoration: none; font-size: 13px; }
aside.toc .toc-title { font-family: var(--font-mono); font-size: 11px; letter-spacing: .12em; color: var(--gold); text-transform: uppercase; margin: 6px 0 8px; }
aside.toc ul { list-style: none; padding: 0; margin: 0; }
aside.toc li { margin: 5px 0; }
aside.toc ul ul { padding-left: 12px; border-left: 1px solid var(--line); margin-left: 2px; }
aside.toc a { color: var(--fg); text-decoration: none; }
aside.toc a:hover { color: var(--cyan); }
main { min-width: 0; }
header.doc { border-bottom: 3px solid var(--red); margin-bottom: 8px; padding-bottom: 8px; }
.eyebrow { font-family: var(--font-mono); font-size: 12px; letter-spacing: .12em; color: var(--gold); text-transform: uppercase; }
main h1 { font-size: clamp(26px, 4.5vw, 36px); line-height: 1.25; margin: 4px 0 6px; text-wrap: balance; }
main h2 { font-size: 23px; margin: 48px 0 12px; padding-left: 12px; border-left: 5px solid var(--red); text-wrap: balance; }
main h3 { font-size: 18px; margin: 30px 0 8px; }
main h4 { font-size: 16px; margin: 22px 0 6px; color: var(--muted); }
main p, main li { max-width: 72ch; }
main a { color: var(--cyan); }
main a:focus-visible { outline: 2px solid var(--gold); outline-offset: 2px; }
main ul, main ol { padding-left: 1.4em; }
main li { margin: 3px 0; }
main hr { border: 0; border-top: 1px solid var(--line); margin: 36px 0; }
code { font-family: var(--font-mono); font-size: .86em; background: var(--code-bg); padding: 1px 5px; border-radius: 4px; }
pre { background: var(--code-bg); border: 1px solid var(--line); border-radius: 8px; padding: 14px 16px; overflow-x: auto; line-height: 1.5; }
pre code { background: none; padding: 0; font-size: 13px; }
blockquote { margin: 16px 0; padding: 10px 16px; background: var(--quote); border-left: 4px solid var(--gold); border-radius: 0 6px 6px 0; }
blockquote p { margin: 4px 0; }
.table-wrap { overflow-x: auto; border: 1px solid var(--line); border-radius: 8px; background: var(--surface); margin: 14px 0; }
table { border-collapse: collapse; width: 100%; font-size: 14.5px; }
th, td { text-align: left; padding: 8px 12px; border-bottom: 1px solid var(--line); vertical-align: top; }
th { font-size: 12.5px; letter-spacing: .04em; color: var(--muted); background: var(--code-bg); }
tr:last-child td { border-bottom: 0; }
.doclist { display: grid; gap: 12px; margin-top: 18px; }
.doclist a { display: block; padding: 14px 18px; border: 1px solid var(--line); border-radius: 8px; background: var(--surface);
  text-decoration: none; color: var(--fg); }
.doclist a:hover { border-color: var(--red); }
.doclist b { display: block; font-size: 17px; }
.doclist span { color: var(--muted); font-size: 14px; }
.foot { margin-top: 48px; color: var(--muted); font-size: 13px; }
@media (max-width: 860px) {
  .layout { grid-template-columns: minmax(0, 1fr); gap: 16px; }
  aside.toc { position: static; max-height: none; border-right: 0; border-bottom: 1px solid var(--line); padding: 0 0 12px; }
}
@media (prefers-reduced-motion: reduce) { * { scroll-behavior: auto !important; } }
"""

FONTS = ('<link rel="preconnect" href="https://fonts.googleapis.com">\n'
         '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+KR:wght@400;500;700'
         '&family=JetBrains+Mono:wght@400;600&display=swap">')


def page(title, body):
    """완성된 HTML 문서 (로컬에서 바로 열어도, 아티팩트로 올려도 보이게 doctype 포함)."""
    return (f'<!doctype html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
            f'<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            f'<meta name="claude:viewport-fit-sides" content="cover">\n<title>{html.escape(title)}</title>\n'
            f'{FONTS}\n<style>{CSS}</style>\n</head>\n<body>\n{body}\n</body>\n</html>\n')


def fix_lists(text):
    """한국어 문서에 흔한 「문단 바로 아래 목록」 을 목록으로 읽히게: 목록 앞에 빈 줄을 넣는다 (코드 블록 안은 그대로)."""
    out, in_code, prev = [], False, ''
    for line in text.splitlines():
        if line.strip().startswith('```'):
            in_code = not in_code
        is_item = bool(re.match(r'\s*([-*+]|\d+\.)\s', line))
        prev_item = bool(re.match(r'\s*([-*+]|\d+\.)\s', prev))
        if not in_code and is_item and prev.strip() and not prev_item and not prev.lstrip().startswith(('|', '>')):
            out.append('')
        out.append(line)
        prev = line
    return '\n'.join(out) + '\n'


def convert(md_path, names):
    """한 문서 변환: 본문 HTML, 목차, 제목."""
    src = md_path.read_text(encoding='utf-8')
    md = markdown.Markdown(extensions=['tables', 'fenced_code', 'toc', 'sane_lists', 'attr_list'],
                           extension_configs={'toc': {'toc_depth': '2-3'}})
    body = md.convert(fix_lists(src))
    # 표는 가로 스크롤 상자에 (휴대폰에서도 화면이 옆으로 밀리지 않게)
    body = body.replace('<table>', '<div class="table-wrap"><table>').replace('</table>', '</table></div>')
    # 다른 문서(.md) 링크 -> 만든 .html
    for n in names:
        body = body.replace(f'href="{n}.md', f'href="{n}.html').replace(f'href="./{n}.md', f'href="{n}.html')
    m = re.search(r'<h1[^>]*>(.*?)</h1>', body, re.S)
    title = re.sub('<[^>]+>', '', m.group(1)).strip() if m else md_path.stem
    if m:                                       # 첫 제목은 머리말로 옮긴다
        body = body.replace(m.group(0), '', 1)
    return body, md.toc, title


def main():
    """docs/*.md 를 모두 변환하고 문서 목록(index.html)을 만든다."""
    mds = sorted(DOCS.glob('*.md'))
    if not mds:
        sys.exit(f'변환할 .md 없음: {DOCS}')
    names = [p.stem for p in mds]
    entries = []
    for p in mds:
        body, toc, title = convert(p, names)
        first = re.search(r'<p>(.*?)</p>', body, re.S)
        summary = re.sub('<[^>]+>', '', first.group(1)).strip()[:120] if first else ''
        doc = (f'<div class="layout">\n<aside class="toc" aria-label="목차"><a class="home" href="index.html">← 문서 목록</a>'
               f'<div class="toc-title">목차</div>{toc}</aside>\n<main>\n<header class="doc"><div class="eyebrow">ARIS3 · docs/{p.name}</div>'
               f'<h1>{title}</h1></header>\n{body}\n<p class="foot">원문: <code>docs/{p.name}</code> · '
               f'<code>scripts/docs_md_to_html.py</code> 로 만든 페이지 (원문을 고친 뒤 다시 돌리면 갱신)</p>\n</main>\n</div>')
        out = p.with_suffix('.html')
        out.write_text(page(title, doc), encoding='utf-8')
        entries.append((out.name, title, summary))
        print('만듦:', out)
    extra = [('ARIS3_학습가이드.html', 'ARIS3 학습 가이드', '전체 통신 흐름도·실물 집기 순서·안전장치·버그 사례·직접 해 볼 과제')]
    links = '\n'.join(f'<a href="{html.escape(f)}"><b>{html.escape(t)}</b><span>{html.escape(s)}</span></a>'
                      for f, t, s in extra + entries if (DOCS / f).exists())
    index = (f'<div class="layout"><aside class="toc"><div class="toc-title">ARIS3</div></aside><main>'
             f'<header class="doc"><div class="eyebrow">ARIS3 · docs</div><h1>ARIS3 문서</h1></header>'
             f'<p>프로젝트 문서 목록입니다. 처음이면 「ARIS3 학습 가이드」 부터 보세요.</p><div class="doclist">{links}</div></main></div>')
    (DOCS / 'index.html').write_text(page('ARIS3 문서', index), encoding='utf-8')
    print('만듦:', DOCS / 'index.html')


if __name__ == '__main__':
    main()
