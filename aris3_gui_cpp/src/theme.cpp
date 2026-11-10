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
QFrame#panel { border: 1px solid #8a6d22; border-radius: 6px;
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

/* 버튼: 볼록한 금속 단추, 누르면 들어감 */
QPushButton { color: #e6edf3; padding: 6px 12px; border: 1px solid #a8862a; border-radius: 4px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #3a2424, stop:0.55 #1e1313, stop:1 #140c0c); }
QPushButton:hover { border-color: #f5c542; color: #f5c542; }
QPushButton:pressed { background: #140c0c; padding-top: 7px; padding-bottom: 5px; }
QPushButton:disabled { color: #6b6060; border-color: #4a3a20; }
QPushButton#gold { color: #0b0f14; font-weight: 700; border: 1px solid #6b5014;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffe08a, stop:0.45 #e8b931, stop:1 #a8801a); }
QPushButton#gold:disabled { color: #4a3a20; background: #3a2e10; }
/* 정지: 밝은 빨강 + 노란 테두리 - 테마 빨강에 묻히지 않게 */
QPushButton#estop { color: white; font-weight: 800; font-size: 12pt; border: 3px solid #ffd23f; border-radius: 6px;
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff5a5f, stop:0.55 #d41b22, stop:1 #9e0f15); }
QPushButton#estop:pressed { background: #7a0a0f; }

/* 입력칸·목록·표: 패널 속에 파묻힌 느낌 */
QLineEdit, QComboBox, QPlainTextEdit, QListWidget, QTableWidget { background: #0b0708; border: 1px solid #4a3a20;
  border-radius: 4px; color: #e6edf3; selection-background-color: #8f1219; }
QComboBox QAbstractItemView { background: #160d0e; color: #e6edf3; selection-background-color: #8f1219; }
QHeaderView::section { background: #2a1416; color: #f5c542; border: none; border-bottom: 1px solid #8a6d22; padding: 3px; }
QTableWidget { gridline-color: #2a1d1d; }
QCheckBox { color: #e6edf3; }
QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #d4a017; border-radius: 3px; background: #0b0708; }
QCheckBox::indicator:checked { background: #d4a017; }
QProgressBar { border: 1px solid #4a3a20; border-radius: 5px; background: #0b0708; text-align: center; color: #e6edf3; }
QProgressBar::chunk { border-radius: 4px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffe08a, stop:1 #d4a017); }
QScrollBar:vertical { background: #0b0708; width: 10px; }
QScrollBar::handle:vertical { background: #4a3a20; border-radius: 4px; }
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
