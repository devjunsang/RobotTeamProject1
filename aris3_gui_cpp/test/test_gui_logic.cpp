// ARIS 순수 로직 단위 시험 (Qt·ROS 없이).
#include <gtest/gtest.h>

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
