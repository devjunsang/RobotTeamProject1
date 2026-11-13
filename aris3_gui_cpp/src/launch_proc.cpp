// LaunchProc 구현 (backend.hpp 설명 참고). SimBackend 의 시작·종료 방식을 실물 모드 보조 노드에도 쓰려고 뺐다.
#include "aris3_gui_cpp/backend.hpp"

#include <QProcess>
#include <QProcessEnvironment>

#include <signal.h>
#include <sys/wait.h>

#include <chrono>
#include <thread>

namespace aris
{

namespace
{
// pid 가 살아 있는가 (우리가 거둘 수 있는 자식이면 거둔다)
bool alivePid(qint64 pid)
{
  if (pid <= 0) {return false;}
  int status = 0;
  if (::waitpid(static_cast<pid_t>(pid), &status, WNOHANG) == static_cast<pid_t>(pid)) {return false;}
  return ::kill(static_cast<pid_t>(pid), 0) == 0;
}
}  // namespace

bool LaunchProc::externalRunning() const
{
  if (pattern_.isEmpty()) {return false;}
  QProcess p;
  p.start("pgrep", {"-f", pattern_});
  p.waitForFinished(3000);
  return p.exitCode() == 0;
}

bool LaunchProc::running() const
{
  return alivePid(pid_);
}

QString LaunchProc::start()
{
  if (running()) {return QString();}
  if (externalRunning()) {return name_ + QStringLiteral(" 노드가 이미 실행 중 - 그것을 사용");}
  QProcess p;
  // 이 화면의 Qt 표시 설정(시험용 offscreen 등)을 물려주지 않는다
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.remove("QT_QPA_PLATFORM");
  p.setProcessEnvironment(env);
  p.setProgram("setsid");
  p.setArguments(QStringList{"ros2", "launch"} + args_);
  p.setStandardOutputFile(log_);
  p.setStandardErrorFile(log_, QIODevice::Append);
  qint64 pid = 0;
  if (!p.startDetached(&pid)) {return name_ + QStringLiteral(" 을(를) 시작하지 못함 (ros2 환경 확인)");}
  pid_ = pid;
  return name_ + QStringLiteral(" 노드 시작 (기록 ") + log_ + ")";
}

void LaunchProc::stop(double timeout_s)
{
  if (!running()) {
    pid_ = 0;
    return;
  }
  ::kill(-static_cast<pid_t>(pid_), SIGINT);       // 프로세스 그룹 전체에
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (alivePid(pid_) && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (alivePid(pid_)) {::kill(-static_cast<pid_t>(pid_), SIGKILL);}
  pid_ = 0;
}

}  // namespace aris
