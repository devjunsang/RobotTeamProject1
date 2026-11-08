# 작업 관리자가 지금 무엇을 하는지 (토픽: /aris3/task_status, GUI·모니터링용)

uint8 STATE_IDLE=0                # 작업 없음, 새 작업을 받을 수 있음
uint8 STATE_BUSY=1                # 작업 실행 중
uint8 STATE_ERROR=2               # 마지막 작업이 실패함 (새 작업은 받을 수 있음)

builtin_interfaces/Time stamp
uint8 state
string task_id                    # 예: "red->A #3"
string stage                      # 현재 단계 이름 (VALIDATE, PERCEIVE, PICK ...)
string message                    # 사람이 읽는 설명
