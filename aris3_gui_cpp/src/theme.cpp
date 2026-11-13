// ARIS 테마 구현 (theme.hpp 설명 참고). Qt 스타일시트는 그림자를 못 그리므로 패널 빛 번짐은
// widgets.cpp 의 Panel 이 QGraphicsDropShadowEffect 로 넣는다.
//
// 아이언맨 테마 6가지 (2026-11-12): 핫로드 레드 · 금색 · 아크 리액터 청록 중 어느 색이 주인공인지가 다르다.
//   실행할 때 ARIS_THEME=jarvis|classic|gold|hotrod|night|mark85 로 고른다 (없으면 jarvis - 사용자 선택).
//   스타일시트는 한 벌이고, 색 자리(@p1 패널 위, @t1 제목줄 위 ...)를 테마별 색으로 채운다.
// 정지 버튼 색은 어느 테마에서도 같다 (밝은 빨강 + 노란 굵은 테두리 - 가장 눈에 띄어야 함).
#include "aris3_gui_cpp/theme.hpp"

#include <QMap>
#include <QStringList>

#include <algorithm>
#include <QtGlobal>

namespace aris::theme
{

namespace
{
using Palette = QMap<QString, QString>;

// 색 자리 (테마마다 이 이름들의 색을 정한다)
//   win1 win2       창 바탕 (가운데 · 가장자리)
//   p1 p2 p3 pb     패널 판 (위 · 가운데 · 아래) · 테두리
//   t1 t2 t3 t4 tt tl   패널 제목줄 (위 → 아래) · 글자 · 아래 선
//   h1 h2 h3 hb hl  상단 줄 (위 → 아래) · 테두리 · 바닥 선
//   b1 b2 b3 bt bb  버튼 판 · 글자 · 테두리     bh1 bh2 bh3  마우스를 올렸을 때
//   g1 g2 g3 gt     금색 판 (금색 버튼 · 고른 탭 · 모드 표시) · 그 위 글자
//   v1 v2 v3 vt vb  음성 버튼 판 · 글자 · 테두리
//   glow            아크 리액터 빛 (마우스 올림 · 초점 · 켜짐 · 진행 막대)
//   i ib it         입력칸·로그 바탕 · 테두리 · 글자     hs  표 머리줄 바탕
//   d1 d2 dt db     못 누르는 버튼 판 · 글자 · 테두리
//   wm              강조 글자 (ARIS 글자·부제·표 머리줄 글자)   cb  체크박스 테두리·슬라이더 길·스크롤 손잡이
Palette classic()
{
  // 1. 클래식 마크 3: 붉은 장갑판 + 금색 페이스플레이트 제목줄 + 청록 선 (균형)
  return {
    {"win1", "#2a0b0e"}, {"win2", "#050203"},
    {"p1", "#3a0c11"}, {"p2", "#22070a"}, {"p3", "#120405"}, {"pb", "#c9a227"},
    {"t1", "#fff0b3"}, {"t2", "#f0c54a"}, {"t3", "#c9971a"}, {"t4", "#8a6512"}, {"tt", "#3a0508"}, {"tl", "#5ff4ff"},
    {"h1", "#e0262f"}, {"h2", "#a3141c"}, {"h3", "#5e0b11"}, {"hb", "#c9a227"}, {"hl", "#5ff4ff"},
    {"b1", "#7a151d"}, {"b2", "#4c0c12"}, {"b3", "#2a070a"}, {"bt", "#f5d27a"}, {"bb", "#c9a227"},
    {"bh1", "#9a1a23"}, {"bh2", "#5e0f16"}, {"bh3", "#33080c"},
    {"g1", "#fff0b3"}, {"g2", "#f0c54a"}, {"g3", "#a8801a"}, {"gt", "#3a0508"},
    {"v1", "#e0262f"}, {"v2", "#9e131b"}, {"v3", "#5e0b11"}, {"vt", "#ffd54a"}, {"vb", "#ffd54a"},
    {"glow", "#5ff4ff"}, {"i", "#05090b"}, {"ib", "#1f6f78"}, {"it", "#bff9ff"}, {"hs", "#6a0c12"},
    {"wm", "#ffe08a"}, {"cb", "#d4a017"},
    {"d1", "#2a1416"}, {"d2", "#170b0c"}, {"dt", "#6b5a4a"}, {"db", "#4a3a20"},
  };
}

Palette gold()
{
  // 2. 골드 페이스플레이트: 금색이 주인공 - 버튼·테두리·상단 줄 테두리까지 금속 금색, 빨강은 바탕으로
  Palette p = classic();
  p["pb"] = "#ffd54a";
  p["h1"] = "#b3161f"; p["h2"] = "#7a0d13"; p["h3"] = "#3a0508"; p["hb"] = "#ffd54a"; p["hl"] = "#ffd54a";
  p["b1"] = "#ffe9a0"; p["b2"] = "#e0b13a"; p["b3"] = "#9a7414"; p["bt"] = "#3a0508"; p["bb"] = "#6b5014";
  p["bh1"] = "#fff4c8"; p["bh2"] = "#f0c54a"; p["bh3"] = "#b3881c";
  p["v1"] = "#fff0b3"; p["v2"] = "#f0c54a"; p["v3"] = "#a8801a"; p["vt"] = "#5e0b11"; p["vb"] = "#a3141c";
  p["hs"] = "#8a6512";
  return p;
}

Palette hotrod()
{
  // 3. 핫로드 레드: 빨강이 주인공 - 제목줄도 붉은 장갑판 + 금색 글자, 아래 선도 금색 (청록은 빛으로만)
  Palette p = classic();
  p["p1"] = "#4a0d13"; p["p2"] = "#2a070b"; p["p3"] = "#160406";
  p["t1"] = "#ff3b44"; p["t2"] = "#c91c24"; p["t3"] = "#8f1219"; p["t4"] = "#5e0b11"; p["tt"] = "#ffe08a"; p["tl"] = "#ffd54a";
  p["b1"] = "#c91c24"; p["b2"] = "#8f1219"; p["b3"] = "#5e0b11"; p["bt"] = "#ffe08a"; p["bb"] = "#ffd54a";
  p["bh1"] = "#e0262f"; p["bh2"] = "#a3141c"; p["bh3"] = "#6a0c12";
  p["hs"] = "#8f1219";
  return p;
}

Palette night()
{
  // 4. 아크 리액터 나이트: 거의 검은 판 + 얇은 금색 + 청록 빛이 주인공 (밤에 빛나는 HUD, 눈이 덜 피곤)
  Palette p = classic();
  p["win1"] = "#140609"; p["win2"] = "#020101";
  p["p1"] = "#1a080a"; p["p2"] = "#0e0406"; p["p3"] = "#070203"; p["pb"] = "#8a6d22";
  p["t1"] = "#5e0b11"; p["t2"] = "#3a070b"; p["t3"] = "#240508"; p["t4"] = "#140305"; p["tt"] = "#ffd54a"; p["tl"] = "#5ff4ff";
  p["h1"] = "#5e0b11"; p["h2"] = "#2a0508"; p["h3"] = "#120203"; p["hb"] = "#8a6d22"; p["hl"] = "#5ff4ff";
  p["b1"] = "#2a080b"; p["b2"] = "#180507"; p["b3"] = "#0c0304"; p["bt"] = "#9ff7ff"; p["bb"] = "#1f8a96";
  p["bh1"] = "#3a0c10"; p["bh2"] = "#22070a"; p["bh3"] = "#120405";
  p["v1"] = "#3a0c10"; p["v2"] = "#22070a"; p["v3"] = "#120405"; p["vt"] = "#ffd54a"; p["vb"] = "#5ff4ff";
  p["hs"] = "#2a0508";
  return p;
}

Palette mark85()
{
  // 5. 마크 85 캔디 레드: 광택 나는 밝은 빨강 + 밝은 금색 + 은은한 청록 (영화 후반 수트처럼 화사하게)
  Palette p = classic();
  p["win1"] = "#3a0c11"; p["win2"] = "#0a0304";
  p["p1"] = "#6a0f17"; p["p2"] = "#3e080d"; p["p3"] = "#1e0407"; p["pb"] = "#ffd54a";
  p["t1"] = "#ffffff"; p["t2"] = "#ffe08a"; p["t3"] = "#e0b13a"; p["t4"] = "#a8801a"; p["tt"] = "#5e0b11"; p["tl"] = "#9ff7ff";
  p["h1"] = "#ff4a52"; p["h2"] = "#d41b22"; p["h3"] = "#8f1219"; p["hb"] = "#ffd54a"; p["hl"] = "#9ff7ff";
  p["b1"] = "#e0262f"; p["b2"] = "#a3141c"; p["b3"] = "#6a0c12"; p["bt"] = "#fff0b3"; p["bb"] = "#ffd54a";
  p["bh1"] = "#ff4a52"; p["bh2"] = "#c91c24"; p["bh3"] = "#7a0d13";
  p["v1"] = "#ff4a52"; p["v2"] = "#c91c24"; p["v3"] = "#7a0d13"; p["vt"] = "#fff0b3"; p["vb"] = "#ffe08a";
  p["hs"] = "#a3141c";
  return p;
}

Palette jarvis()
{
  // 6. 자비스 HUD: 청록이 주인공 - 남색 유리판 + 청록 페이스플레이트 제목줄 + 청록 글자 (헬멧 안 HUD 화면).
  //    빨강은 정지 버튼과 오류 표시에만 남겨 위험 신호가 더 눈에 띈다
  return {
    {"win1", "#0b2230"}, {"win2", "#020609"},
    {"p1", "#0e2836"}, {"p2", "#091a24"}, {"p3", "#040c12"}, {"pb", "#2fc8d8"},
    {"t1", "#e6fdff"}, {"t2", "#7ff6ff"}, {"t3", "#22b4c4"}, {"t4", "#0e6f7a"}, {"tt", "#03161c"}, {"tl", "#e6fdff"},
    {"h1", "#14485e"}, {"h2", "#0c2c3a"}, {"h3", "#05141c"}, {"hb", "#2fc8d8"}, {"hl", "#7ff6ff"},
    {"b1", "#14445a"}, {"b2", "#0c2a38"}, {"b3", "#06151d"}, {"bt", "#bff9ff"}, {"bb", "#2fc8d8"},
    {"bh1", "#1a5a74"}, {"bh2", "#103a4c"}, {"bh3", "#081c26"},
    {"g1", "#e6fdff"}, {"g2", "#7ff6ff"}, {"g3", "#1fa0b0"}, {"gt", "#03161c"},
    {"v1", "#14485e"}, {"v2", "#0c2c3a"}, {"v3", "#05141c"}, {"vt", "#7ff6ff"}, {"vb", "#7ff6ff"},
    {"glow", "#e6fdff"}, {"i", "#03090c"}, {"ib", "#1f6f78"}, {"it", "#bff9ff"}, {"hs", "#0c2a38"},
    {"wm", "#9ff7ff"}, {"cb", "#2fc8d8"},
    {"d1", "#0c1c24"}, {"d2", "#060e13"}, {"dt", "#3f6a76"}, {"db", "#1a3d48"},
  };
}

Palette chosen()
{
  const QByteArray name = qgetenv("ARIS_THEME");
  if (name == "classic") {return classic();}
  if (name == "gold") {return gold();}
  if (name == "night") {return night();}
  if (name == "hotrod") {return hotrod();}
  if (name == "mark85") {return mark85();}
  return jarvis();                           // 기본: 자비스 HUD 청록 (2026-11-12 사용자 선택)
}
}  // namespace

QString styleSheet()
{
  QString qss = QStringLiteral(R"QSS(
QWidget { background: #080304; color: #f1e6dc; font-family: "Noto Sans CJK KR", "NanumSquare", sans-serif; font-size: 10pt; }
QMainWindow { background: qradialgradient(cx:0.5, cy:0.45, radius:0.9, fx:0.5, fy:0.45, stop:0 @win1, stop:1 @win2); }
QToolTip { background: #12060a; color: @glow; border: 1px solid @glow; }

/* 패널: 장갑판 + 테두리 (바깥 청록 빛 번짐은 widgets.cpp Panel) */
QFrame#panel { border: 2px solid @pb; border-radius: 8px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @p1, stop:0.35 @p2, stop:1 @p3); }
QFrame#panel QWidget, QFrame#panelBody { background: transparent; }
/* 제목줄 (위 투명 규칙보다 구체적이어야 보임) */
QFrame#panel QLabel#panelTitle { color: @tt; font-weight: 800; letter-spacing: 2px; padding: 6px 12px;
  border-bottom: 2px solid @tl; border-top-left-radius: 6px; border-top-right-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @t1, stop:0.35 @t2, stop:0.7 @t3, stop:1 @t4); }

/* 상단 줄 */
QFrame#header { border: 2px solid @hb; border-bottom: 4px solid @hl; border-radius: 8px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @h1, stop:0.4 @h2, stop:1 @h3); }
QFrame#header QWidget { background: transparent; }
QLabel#subtitle { color: @wm; font-size: 9pt; }
QLabel#modeChip { color: @gt; font-weight: 800; padding: 2px 10px; border-radius: 3px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @g1, stop:1 @g3); }
QLabel#wordmark { color: @wm; font-size: 26pt; font-weight: 800; letter-spacing: 6px; }

/* 버튼 (패널 안 버튼도 - 위 투명 규칙에 덮이지 않게 QFrame#panel 도 적음): 금속 판, 누르면 들어감, 마우스를 올리면 아크 리액터 빛 테두리 */
QPushButton, QFrame#panel QPushButton { color: @bt; font-weight: 600; padding: 6px 12px; border: 1px solid @bb; border-radius: 4px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @b1, stop:0.5 @b2, stop:1 @b3); }
QPushButton:hover, QFrame#panel QPushButton:hover { border-color: @glow;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @bh1, stop:0.5 @bh2, stop:1 @bh3); }
QPushButton:pressed, QFrame#panel QPushButton:pressed { background: @b3; padding-top: 7px; padding-bottom: 5px; }
QPushButton:disabled, QFrame#panel QPushButton:disabled { color: @dt; border-color: @db;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @d1, stop:1 @d2); }
QPushButton#gold { color: @gt; font-weight: 800; border: 1px solid #6b5014;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @g1, stop:0.45 @g2, stop:1 @g3); }
QPushButton#gold:hover { border-color: @glow; }
QPushButton#gold:disabled { color: @dt; border-color: @db; background: @d1; }
QPushButton#gold:checked { border: 2px solid @glow; }
/* 음성 명령 버튼: 굵고 선명한 글자. 켜지면 아크 리액터 테두리 */
QPushButton#voice { color: @vt; font-size: 12pt; font-weight: 800; letter-spacing: 1px; border: 2px solid @vb;
  border-radius: 6px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @v1, stop:0.5 @v2, stop:1 @v3); }
QPushButton#voice:hover { border-color: @glow; }
QPushButton#voice:checked { border: 3px solid @glow; }
QPushButton#voice:disabled { color: @dt; border-color: @db; background: @d1; }
/* 정지: 밝은 빨강 + 노란 굵은 테두리 - 어느 테마에서도 같음 */
QPushButton#estop { color: white; font-weight: 800; font-size: 12pt; border: 3px solid #ffd23f; border-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff5a5f, stop:0.55 #d41b22, stop:1 #9e0f15); }
QPushButton#estop:pressed { background: #7a0a0f; }

/* 입력칸·목록·표·로그: HUD 유리 + 청록 글자, 초점이 오면 빛 테두리 */
QLineEdit, QComboBox, QPlainTextEdit, QListWidget, QTableWidget,
QFrame#panel QLineEdit, QFrame#panel QComboBox, QFrame#panel QPlainTextEdit, QFrame#panel QListWidget,
QFrame#panel QTableWidget { background: @i; border: 1px solid @ib;
  border-radius: 4px; color: @it; selection-background-color: #0f5560; selection-color: #ffffff; }
QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus { border: 1px solid @glow; }
QComboBox QAbstractItemView { background: #0a1114; color: @it; selection-background-color: #0f5560; }
QHeaderView::section { background: @hs; color: @wm; font-weight: 700; border: none; border-bottom: 2px solid @pb; padding: 3px; }
QTableWidget { gridline-color: #12343a; }

/* 탭 (TCP 이동 / Joint 이동): 고른 탭은 금색 */
QTabWidget::pane { border: 1px solid @pb; border-radius: 4px; top: -1px; }
QTabBar::tab { color: @bt; padding: 5px 14px; border: 1px solid @bb; border-bottom: none;
  border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @b1, stop:1 @b3); }
QTabBar::tab:selected { color: @gt; font-weight: 800;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 @g1, stop:1 @g3); }

/* 체크박스: 금색 테두리, 켜면 아크 리액터 */
QCheckBox { color: #f1e6dc; }
QCheckBox::indicator { width: 15px; height: 15px; border: 2px solid @cb; border-radius: 3px; background: @i; }
QCheckBox::indicator:checked { background: qradialgradient(cx:0.5, cy:0.5, radius:0.6, stop:0 #e6fdff, stop:0.5 @glow, stop:1 #0f8a96); }

/* 슬라이더 (속도): 지나온 길은 아크 리액터, 손잡이는 금색 */
QSlider::groove:horizontal { height: 6px; border-radius: 3px; background: @p2; border: 1px solid @cb; }
QSlider::sub-page:horizontal { border-radius: 3px; background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0f8a96, stop:1 @glow); }
QSlider::handle:horizontal { width: 14px; margin: -5px 0; border-radius: 7px; border: 1px solid #6b5014;
  background: qradialgradient(cx:0.4, cy:0.4, radius:0.7, stop:0 @g1, stop:1 @g3); }

/* 진행 막대: 아크 리액터 */
QProgressBar { border: 1px solid @pb; border-radius: 5px; background: @i; text-align: center; color: #f1e6dc; }
QProgressBar::chunk { border-radius: 4px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #bff9ff, stop:1 #13a7b5); }
QScrollBar:vertical { background: @i; width: 10px; }
QScrollBar::handle:vertical { background: @cb; border-radius: 4px; }
)QSS");
  // 색 자리 채우기: 긴 이름부터 (@bh1 이 @b 로 잘못 바뀌지 않게)
  const Palette p = chosen();
  QStringList keys = p.keys();
  std::sort(keys.begin(), keys.end(), [](const QString & a, const QString & b) {return a.size() > b.size();});
  for (const auto & k : keys) {qss.replace("@" + k, p[k]);}
  return qss;
}

QString dot(bool ok, const QString & text, bool warn)
{
  if (ok) {
    return QStringLiteral("<span style='color:#5ff4ff'>●</span> %1").arg(text);
  }
  if (warn) {
    return QStringLiteral("<span style='color:#f2a33a'>●</span> <span style='color:#f2a33a'>%1</span>").arg(text);
  }
  return QStringLiteral("<span style='color:#ff4d4f'>⚠</span> <span style='color:#ff4d4f'>%1</span>").arg(text);
}

}  // namespace aris::theme
