# 실물 E0509 상태 한 장 (토픽: /aris3/real/state, aris3_real_bridge -> GUI, 10 Hz)  2026-11-10
# 두 GUI(aris3_gui PyQt, aris3_gui_cpp ARIS)가 화면에 보여 줄 것을 모두 담는다. GUI 는 로봇에 직접 붙지 않는다.
# 단위: 두산 컨트롤러와 같게 mm / deg, 로봇 베이스 좌표 (ARIS3 world [m] 가 아님).

builtin_interfaces/Time stamp

# ---------------- 접속
bool connected                    # 관절 상태가 들어오고 있음
bool connecting                   # 접속·해제 진행 중 (connect / disconnect 서비스)
string connection_text            # 사람이 읽는 접속 설명 ("이 화면이 실행한 접속 (pid ...)", "접속 안 됨" ...)

# ---------------- 로봇 상태 (두산 컨트롤러)
uint8 robot_state                 # 두산 상태 번호 (1 STANDBY, 2 MOVING, 3 SAFE_OFF, 5 SAFE_STOP, 6 EMERGENCY_STOP ...)
string robot_state_text           # 한국어 ("대기", "이동 중", "안전 정지" ...)
bool robot_state_ok               # 대기 또는 이동 중 (아니면 화면에 오류 색)

uint8 ROBOT_MODE_MANUAL=0         # 콕핏 직접 교시로 손으로 끌기 (손으로 움직이기)
uint8 ROBOT_MODE_AUTONOMOUS=1     # ROS/API 명령으로 움직임
uint8 ROBOT_MODE_UNKNOWN=255
uint8 robot_mode
bool mode_switching               # 로봇 모드 바꾸는 중
bool recovering                   # 복구 후 홈 진행 중
bool can_recover                  # 복구 서비스가 준비되어 「복구 후 홈」 을 누를 수 있음
bool busy                         # 이동·그리퍼 명령 실행 중
string why_not_ready              # 지금 움직일 수 없는 이유. "" = 움직일 수 있음 (armed 는 따로 본다)

# ---------------- 조작 허용 (브리지도 막는다: armed 가 아니면 이동·그리퍼·집기 요청을 거부)
bool armed
float32 armed_remaining_s         # 명령이 없으면 이 시간 뒤 자동으로 잠김 (마지막 명령부터 120 s)

# ---------------- 위치
bool joints_valid
float64[6] joints_deg
bool tcp_valid
float64[6] tcp                    # x, y, z [mm], rx, ry, rz [deg] (로봇 베이스)

# ---------------- 그리퍼 (RH-P12-RN-A, DRL 경로)
uint8 GRIPPER_RESULT_UNKNOWN=0    # 접속 뒤 아직 응답 확인 전
uint8 GRIPPER_RESULT_OK=1
uint8 GRIPPER_RESULT_FAILED=2     # 마지막 명령 실패 (토크 꺼짐 포함)
bool gripper_service_ready        # DRL 서비스가 있음 (접속됨)
bool gripper_running
uint8 gripper_result
bool gripper_valid                # 아래 두 값이 들어온 적 있음
uint16 gripper_position           # 0 열림 ~ 740 닫힘
bool gripper_torque

# ---------------- 집기 순서 (/aris3/real/pick_place)
uint8 HOLD_UNKNOWN=0
uint8 HOLD_YES=1                  # 마지막 닫기에서 물체에 막혀 멈춤 = 잡음
uint8 HOLD_NO=2                   # 끝까지 닫힘 = 빈손
bool sequence_running
string sequence_name              # "집기", "집어서 옮기기" ...
string sequence_step              # 지금 단계 ("접근", "하강" ...)
uint8 holding

# ---------------- 카메라 보정 (영상 픽셀 -> 로봇 XY, /aris3/real/calibration)
uint16 calib_points
bool calib_ready                  # 점 4개 이상, 아래 행렬이 유효
float64[9] calib_homography       # 행 우선 3x3: [x y w] = H [u v 1], 로봇 X = x/w, Y = y/w [mm]
float32 calib_grasp_z_mm          # 집는 높이 (기록한 z 의 중앙값)
float32 calib_error_mean_mm
float32 calib_error_max_mm
string calib_summary              # 사람이 읽는 요약
bool calib_pending                # ① 픽셀을 기억했고 ② 로봇 위치 기록을 기다리는 중
float32 calib_pending_u           # 기억한 픽셀 (화면에 표시)
float32 calib_pending_v
