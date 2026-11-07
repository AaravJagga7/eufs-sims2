#include "eufs_sim2/time/time.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace eufs::sim2::time {

rosgraph_msgs::msg::Clock TimeToClockMsg(Time time) {
  rosgraph_msgs::msg::Clock clock;
  clock.clock = TimeToTimeMsg(time);
  return clock;
}

builtin_interfaces::msg::Time TimeToTimeMsg(Time time) {
  builtin_interfaces::msg::Time msg;
  Time ns = Time(1s);  // 1e9 nanoseconds
  msg.sec = std::chrono::duration_cast<std::chrono::seconds>(time).count();
  msg.nanosec = time.count() % ns.count();
  return msg;
}

bool HasTimeElapsed(Time now, Time past, Duration period) { return now - past >= period; }

bool HasTimeElapsed(Time now, Time past, double frequency) {
  return HasTimeElapsed(now, past, Duration(static_cast<uint64_t>(1e9 / frequency)));
}

}  // namespace eufs::sim2::time
