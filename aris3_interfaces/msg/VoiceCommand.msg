# 음성 명령 하나의 인식·해석 결과 (토픽: /aris3/voice_command, aris3_voice -> GUI · Task Manager)
# 로봇을 움직이는 명령이 아니다. 실행은 GUI(또는 사용자)가 확인한 뒤 ExecuteTask 로 요청한다.

builtin_interfaces/Time stamp
string text                       # Whisper 가 인식한 문장 그대로
string normalized_text            # 오인식 보정·공백 제거 후
string action                     # "pick" | "place" | "home" (홈 자세로, 2026-11-09) | "find" (그 색 큐브 찾기, 2026-11-14) | "" (모름)
string color                      # "red" | "blue" | "yellow" | "green" | ""
string shape                      # "cube" | ... | "" (말하지 않음 - 모양이 모두 같으면 생략 가능)
bool refers_selected_object       # 「이거/그거/저거」: 화면(클릭·손짓)에서 고른 물체를 가리킴 (2026-11-09)
bool refers_selected_place        # 「저기/여기/거기」: 화면(클릭·손짓)에서 고른 자리를 가리킴
string target_color               # 쌓기 (2026-11-13): 「노란 큐브를 파란 큐브 위로」 의 받침 큐브 색 (blue). 없으면 ""
uint8 number                      # 큐브 번호 (2026-11-14): 「1번 파란색 큐브」 의 1 - 같은 색이 여럿일 때 색 검출 이름 blue_cube_1 의 번호. 0 = 말하지 않음
uint8 target_number               # 쌓기 받침 큐브의 번호 (「2번 파란 큐브 위로」 의 2). 0 = 말하지 않음
bool valid                        # 실행할 수 있는 명령인가 (아래 message 에 이유)
string message                    # 사람이 읽는 설명 ("PICK 명령 생성", "색이 지정되지 않음" ...)
float32 stt_time_s                # 음성 인식에 걸린 시간 [s]
