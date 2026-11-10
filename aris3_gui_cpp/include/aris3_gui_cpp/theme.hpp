// ARIS 테마 (아이언맨 슈트 색 - 목업 https://claude.ai/artifact/JWd8zHNgw7hHZXVh9sKyZ8 과 같은 값).
//   바탕 짙은 검정·붉은 기, 주색 핫로드 레드, 보조 골드, 포인트 아크 시안.
// 안전 표시: 테마가 빨강이라 「오류」 는 밝은 빨강 + 아이콘, 정상은 시안, 경고는 앰버로 구분한다.
// 정지 버튼은 밝은 빨강 바탕 + 노란 테두리로 화면에서 가장 눈에 띄게.
#pragma once

#include <QColor>
#include <QString>

namespace aris::theme
{

inline const QColor kBg{"#0c0607"};          // 창 바탕
inline const QColor kPanel{"#160d0e"};       // 패널 바탕
inline const QColor kRed{"#a3141c"};         // 주색 (제목줄)
inline const QColor kGold{"#d4a017"};        // 보조 (테두리·강조)
inline const QColor kGoldText{"#f5c542"};    // 금색 글자
inline const QColor kCyan{"#5ff4ff"};        // 정상·포인트
inline const QColor kAmber{"#f2a33a"};       // 대기·경고
inline const QColor kError{"#ff4d4f"};       // 오류 (테마 빨강보다 밝게)
inline const QColor kText{"#e6edf3"};        // 본문 글자
inline const QColor kMuted{"#9aa7b4"};       // 흐린 글자

// 창 전체 스타일시트 (패널·제목줄·버튼·입력칸·표·진행 막대의 색과 입체감)
QString styleSheet();

// 상태 점 + 글자 (HTML). ok = 시안, warn = 앰버, 그 밖 = 밝은 빨강 + ⚠
QString dot(bool ok, const QString & text, bool warn = false);

}  // namespace aris::theme
