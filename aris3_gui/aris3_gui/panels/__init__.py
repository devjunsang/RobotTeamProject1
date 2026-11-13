"""ARIS3 Control Station 화면 패널 (main_window.ControlStation 을 패널별 파일로 나눈 것, 2026-11-10).

각 파일은 패널 하나의 mixin 클래스다: 화면 구성(_xxx_panel), 버튼 동작, 100 ms 주기 갱신(_refresh_xxx).
ControlStation 이 이 mixin 들을 모두 상속하므로 위젯·상태는 예전처럼 self 에 있고, 패널끼리는
self 를 통해 서로의 위젯·함수를 쓴다 (예: 음성 패널이 작업 큐에 넣고, 실물 집기가 「조작 허용」 을 본다).

  common.py          공용 상수·색, nofocus(), dot()
  header.py          상단 버튼 줄: Connect · Disconnect · Home · Start Task · Stop · 정지
  status.py          왼쪽: 시스템·로봇·그리퍼 상태, 카메라 정보
  camera.py          카메라 Top View, 작업 공간 2D
  task_queue.py      작업 큐·단계 진행, 카메라 화면 선택(이거/저기)으로 작업 만들기
  voice.py           음성 명령 (누르고 말하기 / 문장 입력 -> 해석 -> 실행)
  real_pick_panel.py 실물: 카메라 보정·집기, 손짓(MediaPipe), 실물 음성 명령
  jog.py             수동 조작 (TCP·관절), 조작 허용, 손으로 움직이기, 복구 후 홈
  log.py             로그
"""
