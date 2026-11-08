# 음성 명령 하나의 인식·해석 결과 (토픽: /aris3/voice_command, aris3_voice -> GUI · Task Manager)
# 로봇을 움직이는 명령이 아니다. 실행은 GUI(또는 사용자)가 확인한 뒤 ExecuteTask 로 요청한다.

builtin_interfaces/Time stamp
string text                       # Whisper 가 인식한 문장 그대로
string normalized_text            # 오인식 보정·공백 제거 후
string action                     # "pick" | "place" | "" (모름)
string color                      # "red" | "blue" | "yellow" | "green" | ""
string shape                      # "cube" | ... | "" (말하지 않음 - 모양이 모두 같으면 생략 가능)
bool valid                        # 실행할 수 있는 명령인가 (아래 message 에 이유)
string message                    # 사람이 읽는 설명 ("PICK 명령 생성", "색이 지정되지 않음" ...)
float32 stt_time_s                # 음성 인식에 걸린 시간 [s]
