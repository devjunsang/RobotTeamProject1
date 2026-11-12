// ARIS 테마 구현 (theme.hpp 설명 참고). Qt 스타일시트는 그림자를 못 그리므로 패널 그림자는
// widgets.cpp 의 Panel 이 QGraphicsDropShadowEffect 로 넣는다.
#include "aris3_gui_cpp/theme.hpp"

namespace aris::theme
{

QString styleSheet()
{
  return QStringLiteral(R"QSS(
QWidget { background: #0c0607; color: #e6edf3; font-family: "Noto Sans CJK KR", "NanumSquare", sans-serif; font-size: 10pt; }
QToolTip { background: #160d0e; color: #f5c542; border: 1px solid #d4a017; }

/* 패널: 위는 따뜻하고 아래로 어두워지는 금속판, 금색 테두리 */
QFrame#panel { border: 1px solid #c9a227; border-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #24140f, stop:0.4 #160c0d, stop:1 #0f0809); }
QLabel#panelTitle { color: #f5c542; font-weight: 700; letter-spacing: 1px; padding: 6px 10px;
  border-bottom: 2px solid #d4a017; border-top-left-radius: 5px; border-top-right-radius: 5px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #c91c24, stop:0.45 #8f1219, stop:1 #5e0b11); }
QFrame#panel QWidget, QFrame#panelBody { background: transparent; }

/* 상단 줄: 붉은 금속판 */
QFrame#header { border: 1px solid #d4a017; border-bottom: 4px solid #d4a017; border-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d02028, stop:0.4 #9e131b, stop:1 #6a0c12); }
QFrame#header QWidget { background: transparent; }
QLabel#subtitle { color: #f3d9a4; font-size: 9pt; }
QLabel#modeChip { color: #2a0508; font-weight: 800; padding: 2px 10px; border-radius: 3px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffe08a, stop:1 #d4a017); }
QLabel#wordmark { color: #ffe08a; font-size: 26pt; font-weight: 800; letter-spacing: 6px; }

/* 버튼 (2026-11-12 아이언맨 색): 짙은 핫로드 레드 장갑판 + 금색 글자·테두리, 누르면 들어감 */
QPushButton { color: #f5d27a; font-weight: 600; padding: 6px 12px; border: 1px solid #c9a227; border-radius: 4px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #6e141b, stop:0.5 #470c11, stop:1 #2a070a); }
QPushButton:hover { border-color: #ffe08a; color: #ffe08a;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #8a1820, stop:0.5 #5a0f15, stop:1 #33080c); }
QPushButton:pressed { background: #24060a; padding-top: 7px; padding-bottom: 5px; }
QPushButton:disabled { color: #6b5a4a; border-color: #4a3a20;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a1416, stop:1 #170b0c); }
QPushButton#gold { color: #0b0f14; font-weight: 700; border: 1px solid #6b5014;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffe08a, stop:0.45 #e8b931, stop:1 #a8801a); }
QPushButton#gold:disabled { color: #4a3a20; background: #3a2e10; }
/* 음성 명령 모드 ON: 시안 빛 테두리로 켜져 있음을 분명히 */
QPushButton#gold:checked { border: 2px solid #5ff4ff; color: #0b0f14; }
/* 음성 명령 버튼: 붉은 장갑판 위 굵고 밝은 금색 글자 (선명하게). 켜지면 아크 리액터 시안 테두리 */
QPushButton#voice { color: #ffd54a; font-size: 12pt; font-weight: 800; letter-spacing: 1px; border: 2px solid #ffd54a;
  border-radius: 6px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d02028, stop:0.5 #8f1219, stop:1 #5e0b11); }
QPushButton#voice:hover { color: #fff0b3; border-color: #fff0b3; }
QPushButton#voice:checked { color: #ffe27a; border: 3px solid #5ff4ff;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0262f, stop:0.5 #a3141c, stop:1 #6a0c12); }
QPushButton#voice:disabled { color: #8a7040; border-color: #6b5014; background: #3a0d10; }
/* 정지: 밝은 빨강 + 노란 테두리 - 테마 빨강에 묻히지 않게 */
QPushButton#estop { color: white; font-weight: 800; font-size: 12pt; border: 3px solid #ffd23f; border-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff5a5f, stop:0.55 #d41b22, stop:1 #9e0f15); }
QPushButton#estop:pressed { background: #7a0a0f; }

/* 입력칸·목록·표: 패널 속에 파묻힌 느낌 */
QLineEdit, QComboBox, QPlainTextEdit, QListWidget, QTableWidget { background: #0b0708; border: 1px solid #4a3a20;
  border-radius: 4px; color: #e6edf3; selection-background-color: #8f1219; }
QComboBox QAbstractItemView { background: #160d0e; color: #e6edf3; selection-background-color: #8f1219; }
QHeaderView::section { background: #5e0b11; color: #f5c542; font-weight: 700; border: none; border-bottom: 1px solid #c9a227; padding: 3px; }
QTableWidget { gridline-color: #2a1d1d; }
QCheckBox { color: #e6edf3; }
QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #d4a017; border-radius: 3px; background: #0b0708; }
QCheckBox::indicator:checked { background: #d4a017; }
QProgressBar { border: 1px solid #4a3a20; border-radius: 5px; background: #0b0708; text-align: center; color: #e6edf3; }
QProgressBar::chunk { border-radius: 4px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffe08a, stop:1 #d4a017); }
QScrollBar:vertical { background: #0b0708; width: 10px; }
QScrollBar::handle:vertical { background: #8a6d22; border-radius: 4px; }
)QSS");
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
