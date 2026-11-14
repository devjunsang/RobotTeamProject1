// ARIS 순수 로직 단위 시험 (Qt·ROS 없이).
#include <gtest/gtest.h>

#include <cmath>

#include "aris3_gui_cpp/gui_logic.hpp"

using aris::Clock;
using aris::StageState;
using std::chrono::milliseconds;

// 10 Hz 로 들어오면 약 10 Hz, 2 초 넘게 끊기면 0 Hz 이고 fresh 가 아니다.
TEST(RateMeter, MeasuresRateAndStaleness)
{
  aris::RateMeter r;
  const auto t0 = Clock::now();
  for (int i = 0; i < 11; ++i) {r.tick(t0 + milliseconds(100 * i));}
  const auto last = t0 + milliseconds(1000);
  EXPECT_NEAR(r.hz(last), 10.0, 1e-6);
  EXPECT_TRUE(r.fresh(1.0, last + milliseconds(500)));
  EXPECT_FALSE(r.fresh(1.0, last + milliseconds(1500)));
  EXPECT_EQ(r.hz(last + milliseconds(2500)), 0.0);
}

// 받은 적이 없거나 한 번만 받으면 0 Hz.
TEST(RateMeter, ZeroWithTooFewSamples)
{
  aris::RateMeter r;
  EXPECT_EQ(r.hz(), 0.0);
  EXPECT_FALSE(r.fresh());
  r.tick();
  EXPECT_EQ(r.hz(), 0.0);
}

// ARIS 로그 거르기: ARIS3·Task Manager·RobotLink·GUI 는 INFO 도, 나머지는 WARN 이상만, MoveIt 잡음은 숨김.
TEST(LogFilter, ShowsArisNodesAndWarningsOnly)
{
  EXPECT_TRUE(aris::isArisLog("INFO", "aris3_real_bridge", "접속"));
  EXPECT_TRUE(aris::isArisLog("INFO", "task_manager", "수락"));
  EXPECT_TRUE(aris::isArisLog("INFO", "e0509_manual_controller", "홈 도착"));
  EXPECT_TRUE(aris::isArisLog("INFO", "gui", "작업 요청"));
  EXPECT_FALSE(aris::isArisLog("INFO", "move_group", "Planning request"));
  EXPECT_TRUE(aris::isArisLog("WARN", "move_group", "something"));
  EXPECT_FALSE(aris::isArisLog("WARN", "move_group", "No 3D sensor plugin(s) defined"));
}

// 단계 추적: 다음 단계 feedback 이 오면 앞 단계는 완료, 실패로 끝나면 실행 중 단계와 failed_stage 가 실패.
TEST(StageTracker, AdvancesAndFails)
{
  aris::StageTracker s;
  const auto t0 = Clock::now();
  s.feedback("VALIDATE", t0);
  s.feedback("OBSERVE", t0 + milliseconds(400));
  EXPECT_EQ(s.state("VALIDATE"), StageState::DONE);
  EXPECT_NEAR(s.seconds("VALIDATE"), 0.4, 1e-6);
  EXPECT_EQ(s.state("OBSERVE"), StageState::RUN);
  s.feedback("OBSERVE", t0 + milliseconds(500));       // 같은 단계 반복은 무시
  EXPECT_EQ(s.state("OBSERVE"), StageState::RUN);
  s.feedback("NOT_A_STAGE", t0 + milliseconds(600));   // 모르는 이름도 무시
  s.finish(false, "PERCEIVE", t0 + milliseconds(1000));
  EXPECT_EQ(s.state("OBSERVE"), StageState::FAIL);
  EXPECT_EQ(s.state("PERCEIVE"), StageState::FAIL);
  EXPECT_EQ(s.state("PICK"), StageState::WAIT);
  s.reset();
  EXPECT_EQ(s.state("OBSERVE"), StageState::WAIT);
}

// 성공으로 끝나면 실행 중이던 마지막 단계가 완료.
TEST(StageTracker, FinishOk)
{
  aris::StageTracker s;
  s.feedback("VERIFY");
  s.finish(true, "");
  EXPECT_EQ(s.state("VERIFY"), StageState::DONE);
}

// 아래를 보는 카메라 (world (0, 0, 2) 에서 -z 방향): 영상 가운데 = 바로 아래, project 와 unproject 가 서로 반대.
TEST(PinholeCamera, ProjectUnprojectRoundTrip)
{
  aris::PinholeCamera cam;
  // world -> 광학: x 축으로 180° 회전 (z 가 아래를 봄) 후 카메라 높이 2 m 만큼 앞(광축)으로
  cam.q = {1.0, 0.0, 0.0, 0.0};
  cam.t = {0.0, 0.0, 2.0};
  cam.width = 640;
  cam.height = 480;
  cam.hfov = 1.4;
  const auto c = cam.project({0.0, 0.0, 0.0});
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR((*c)[0], 320.0, 1e-9);
  EXPECT_NEAR((*c)[1], 240.0, 1e-9);
  EXPECT_NEAR((*c)[2], 2.0, 1e-9);
  const auto p = cam.project({0.3, -0.2, 0.75});
  ASSERT_TRUE(p.has_value());
  const auto back = cam.unproject((*p)[0], (*p)[1], 0.75);
  ASSERT_TRUE(back.has_value());
  EXPECT_NEAR((*back)[0], 0.3, 1e-9);
  EXPECT_NEAR((*back)[1], -0.2, 1e-9);
  EXPECT_FALSE(cam.project({0.0, 0.0, 3.0}).has_value());     // 카메라 뒤(위)는 없음
}

// 쿼터니언 회전: z 축 90° 회전이면 x 축이 y 축으로.
TEST(PinholeCamera, QuatApplyRotatesVector)
{
  const double s = std::sqrt(0.5);
  const auto r = aris::quatApply({0.0, 0.0, s, s}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(r[0], 0.0, 1e-12);
  EXPECT_NEAR(r[1], 1.0, 1e-12);
  EXPECT_NEAR(r[2], 0.0, 1e-12);
}

// 받침에 맞는 조사: 큐브를 / 삼각형을 / 원을 / 한글 아닌 말은 「를」.
TEST(VoiceReply, JosaFollowsFinalConsonant)
{
  EXPECT_EQ(aris::josa("큐브", "을", "를"), "를");
  EXPECT_EQ(aris::josa("삼각형", "을", "를"), "을");
  EXPECT_EQ(aris::josa("원", "이", "가"), "이");
  EXPECT_EQ(aris::josa("ABC", "을", "를"), "를");
  EXPECT_EQ(aris::josa("", "을", "를"), "를");
}

// 사용자가 원한 예: 「노란색 큐브 잡아」 -> 「네, 노란색 큐브를 잡습니다.」 그 밖의 동작도 한 문장씩.
TEST(VoiceReply, SentencesForEachAction)
{
  EXPECT_EQ(aris::voiceReply("pick", "yellow", "cube", false), "네, 노란색 큐브를 잡습니다.");
  EXPECT_EQ(aris::voiceReply("pick", "blue", "", false), "네, 파란색 큐브를 잡습니다.");
  EXPECT_EQ(aris::voiceReply("pick", "red", "triangle", false), "네, 빨간색 삼각형을 잡습니다.");
  EXPECT_EQ(aris::voiceReply("pick", "", "", false), "네, 고르신 물체를 잡습니다.");
  EXPECT_EQ(aris::voiceReply("pick", "red", "", true), "네, 빨간색 큐브를 집어서 고르신 자리로 옮깁니다.");
  EXPECT_EQ(aris::voiceReply("home", "", "", false), "네, 홈 자세로 돌아갑니다.");
  EXPECT_EQ(aris::voiceReply("stop", "", "", false), "네, 멈춥니다.");
  EXPECT_EQ(aris::voiceReply("gripper_open", "", "", false), "네, 그리퍼를 엽니다.");
  EXPECT_EQ(aris::voiceReply("", "", "", false), "잘 못 알아들었어요. 다시 말씀해 주세요.");
}

// 색 탐색 (2026-11-13): 좌우로 넓혀 가다 시작 자세로 돌아오고, 안 보이는 색만 고른다.
TEST(ColorSearch, SweepEndsAtStartAndMissingColors)
{
  const auto sw = aris::searchSweepDeg();
  ASSERT_FALSE(sw.empty());
  EXPECT_DOUBLE_EQ(sw.back(), 0.0);
  bool left = false, right = false;
  for (double d : sw) {
    EXPECT_LE(std::abs(d), 45.0);
    left |= d < 0;
    right |= d > 0;
  }
  EXPECT_TRUE(left && right);
  EXPECT_EQ(aris::missingColors({"yellow", "blue"}, {"blue", "red"}), std::vector<std::string>{"yellow"});
  EXPECT_EQ(aris::missingColors({"yellow", "", "yellow"}, {}), std::vector<std::string>{"yellow"});
  EXPECT_TRUE(aris::missingColors({"red"}, {"red"}).empty());
}

// 쌓기 대답 (2026-11-13): 집을 큐브와 받침 큐브를 둘 다 말한다.
TEST(VoiceReply, StackSentence)
{
  EXPECT_EQ(aris::voiceReply("pick", "yellow", "cube", false, "blue"), "네, 노란색 큐브를 파란색 큐브 위에 올립니다.");
  EXPECT_EQ(aris::voiceReply("pick", "", "", false, "green"), "네, 고르신 물체를 초록색 큐브 위에 올립니다.");
}

// 어떤 대답에도 호출어 「아리스」 가 없다 (마이크가 다시 들어도 명령이 되지 않게).
TEST(VoiceReply, NeverContainsWakeWord)
{
  for (const char * a : {"pick", "place", "home", "stop", "resume", "power_on", "disconnect", ""}) {
    for (bool rp : {false, true}) {
      EXPECT_EQ(aris::voiceReply(a, "yellow", "cube", rp).find("아리스"), std::string::npos) << a;
    }
  }
}

// 큐브 번호 (2026-11-14): 「1번 파란색 큐브」 - 이름의 번호 읽기, 말한 번호로 큐브 고르기, 대답 문장.
TEST(CubeNumber, IdNumberPickAndReply)
{
  EXPECT_EQ(aris::cubeNumberFromId("blue_cube_2"), 2);
  EXPECT_EQ(aris::cubeNumberFromId("blue_cube"), 0);        // 그 색이 하나뿐이면 번호 없음
  EXPECT_EQ(aris::cubeNumberFromId("blue"), 0);
  EXPECT_EQ(aris::cubeNumberFromId(""), 0);
  EXPECT_EQ(aris::cubeNumberFromId("cube_"), 0);

  const std::vector<std::string> two{"blue_cube_1", "blue_cube_2"};
  EXPECT_EQ(aris::pickCubeIndex(two, 2), 1);                // 말한 번호의 큐브
  EXPECT_EQ(aris::pickCubeIndex(two, 1), 0);
  EXPECT_EQ(aris::pickCubeIndex(two, 3), -1);               // 없는 번호
  EXPECT_EQ(aris::pickCubeIndex(two, 0), -2);               // 번호를 말하지 않았는데 여럿
  EXPECT_EQ(aris::pickCubeIndex({"blue_cube"}, 0), 0);      // 하나뿐이면 번호 없이도
  EXPECT_EQ(aris::pickCubeIndex({"blue_cube"}, 1), 0);      // 하나뿐인 큐브 = 1번
  EXPECT_EQ(aris::pickCubeIndex({"blue_cube"}, 2), -1);
  EXPECT_EQ(aris::pickCubeIndex({}, 1), -1);

  EXPECT_EQ(aris::objectKo("blue", "cube", 1), "1번 파란색 큐브");
  EXPECT_EQ(aris::objectKo("", "", 1), "고르신 물체");      // 색이 없으면 번호를 붙이지 않는다
  EXPECT_EQ(aris::voiceReply("pick", "blue", "cube", false, "", 1), "네, 1번 파란색 큐브를 잡습니다.");
  EXPECT_EQ(aris::voiceReply("pick", "blue", "cube", false, "blue", 1, 2),
    "네, 1번 파란색 큐브를 2번 파란색 큐브 위에 올립니다.");
}

// 올려놓기 (2026-11-14): 「빨간 큐브 위로 올려놔」 - 들고 있는 큐브를 받침 위에. 받침 번호도 말한다.
TEST(VoiceReply, PlaceOnBaseCube)
{
  EXPECT_EQ(aris::voiceReply("place", "", "", false, "red"), "네, 들고 있는 큐브를 빨간색 큐브 위에 올려놓겠습니다.");
  EXPECT_EQ(aris::voiceReply("place", "", "", false, "blue", 0, 2),
    "네, 들고 있는 큐브를 2번 파란색 큐브 위에 올려놓겠습니다.");
  EXPECT_EQ(aris::voiceReply("place", "", "", false), "네, 여기에 내려놓겠습니다.");   // 받침이 없으면 예전 그대로
}

// 찾기 (2026-11-14): 「파란색 큐브 찾아」 의 대답. 번호를 말했으면 번호도.
TEST(VoiceReply, FindSentence)
{
  EXPECT_EQ(aris::voiceReply("find", "blue", "cube", false), "네, 파란색 큐브를 찾아볼게요.");
  EXPECT_EQ(aris::voiceReply("find", "red", "", false, "", 2), "네, 2번 빨간색 큐브를 찾아볼게요.");
  EXPECT_EQ(aris::voiceReply("scan", "", "", false), "네, 큐브를 찾아볼게요.");   // 탐색 (2026-11-14): 색을 말하지 않음
}

// 닿지 않는 위치 (2026-11-14): 베이스에서 700 mm 넘으면 멀다고 보고, 그 이유를 말한다.
TEST(Reach, TooFarAndReply)
{
  EXPECT_FALSE(aris::tooFar(685.0, 29.0));                  // 실물에서 간 곳
  EXPECT_TRUE(aris::tooFar(714.0, 53.0));                   // 컨트롤러가 NOT REACHABLE 로 거부한 곳
  EXPECT_FALSE(aris::tooFar(0.0, -700.0));
  EXPECT_TRUE(aris::tooFar(500.0, 500.0));
  EXPECT_EQ(aris::tooFarReply("노란색 큐브"), "노란색 큐브는 너무 멀어서 팔이 닿지 않아요.");
  EXPECT_EQ(aris::tooFarReply("고르신 자리"), "고르신 자리는 너무 멀어서 팔이 닿지 않아요.");
}

// 로그 창 (2026-11-14): 「아리스」 없이 들린 말(무시한 말)은 넣지 않고, 「아리스」 로 부른 명령·ARIS 의 대답은 넣는다.
TEST(LogFilter, HidesSpeechWithoutWakeWord)
{
  EXPECT_TRUE(aris::isIgnoredSpeechLog("aris3_voice", "「오늘 점심 뭐 먹지」 - 호출어(아리스) 없음, 무시 (0.22s)"));
  EXPECT_TRUE(aris::isIgnoredSpeechLog("aris3_voice", "「네, 말씀하세요.」 - ARIS 가 말하는 중에 들린 말이라 무시, 명령을 계속 기다림"));
  EXPECT_FALSE(aris::isIgnoredSpeechLog("aris3_voice", "「아리스, 노란색 큐브 잡아.」 -> action=pick color=yellow valid=True"));
  EXPECT_FALSE(aris::isIgnoredSpeechLog("aris3_voice", "「아리스」 - 호출어만 들림: 8 초 안의 다음 말을 명령으로 받음"));
  EXPECT_FALSE(aris::isIgnoredSpeechLog("aris3_voice", "말함: 「네, 노란색 큐브를 잡습니다.」 (합성 0.0s)"));
  EXPECT_FALSE(aris::isIgnoredSpeechLog("aris3_real_bridge", "호출어(아리스) 없음, 무시"));   // 다른 노드의 글은 그대로
}

// 닿는 범위 경계선 (2026-11-14): 보정 행렬의 역으로 옮긴 점들을 다시 로봇 좌표로 바꾸면 모두 반지름 위에 있다.
TEST(Reach, BoundaryPixelsLieOnCircle)
{
  // 픽셀 -> 로봇: x = 900 - 1.2 v, y = 780 - 1.2 u (위쪽 카메라처럼 축이 바뀌고 뒤집힌 닮음 변환)
  const std::array<double, 9> H{0.0, -1.2, 900.0, -1.2, 0.0, 780.0, 0.0, 0.0, 1.0};
  const auto lines = aris::reachBoundaryPx(H, 700.0, 72);
  ASSERT_EQ(lines.size(), 1u);                              // 원근이 없으면 원 전체가 한 조각
  ASSERT_EQ(lines[0].size(), 73u);                          // 72 점 + 닫는 점
  EXPECT_DOUBLE_EQ(lines[0].front()[0], lines[0].back()[0]);
  for (const auto & q : lines[0]) {
    const double x = 900.0 - 1.2 * q[1], y = 780.0 - 1.2 * q[0];
    EXPECT_NEAR(std::hypot(x, y), 700.0, 1e-6);
  }
  EXPECT_TRUE(aris::reachBoundaryPx({0, 0, 0, 0, 0, 0, 0, 0, 0}, 700.0).empty());   // 뒤집을 수 없는 행렬
  EXPECT_TRUE(aris::reachBoundaryPx(H, 0.0).empty());
}

// 가까운 쪽 한계 (2026-11-14): 베이스에서 300 mm 안쪽은 너무 가깝다고 보고, 그 이유를 말한다.
TEST(Reach, TooCloseAndReply)
{
  EXPECT_TRUE(aris::tooClose(211.0, -6.0));                 // 실물에서 하강 중 3번 관절 한계로 멈춘 곳
  EXPECT_FALSE(aris::tooClose(402.0, 148.0));               // 실물에서 집은 곳
  EXPECT_FALSE(aris::tooClose(300.0, 0.0));
  EXPECT_EQ(aris::tooCloseReply("파란색 큐브"), "파란색 큐브는 로봇에 너무 가까워서 팔이 닿지 않아요.");
}

