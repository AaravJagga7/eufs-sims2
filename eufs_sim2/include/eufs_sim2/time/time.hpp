#ifndef INCLUDE_EUFS_SIM2_TIME_TIME_HPP_
#define INCLUDE_EUFS_SIM2_TIME_TIME_HPP_

#include <chrono>  // NOLINT
#include <rosgraph_msgs/msg/clock.hpp>

namespace eufs::sim2::time {

using Time = std::chrono::duration<size_t, std::nano>;
using Duration = std::chrono::duration<size_t, std::nano>;

/**
 * @brief convert time object to rosgraph_msgs::msg::Clock message
 * @param time object to convert
 * @return msg
 */
rosgraph_msgs::msg::Clock TimeToClockMsg(Time time);

/**
 * @brief convert time object to builtin_interfaces::msg::Time message
 * @param time object to convert
 * @return msg
 */
builtin_interfaces::msg::Time TimeToTimeMsg(Time time);

/**
 * @brief check if a period of time has passed
 * @param now current time
 * @param past past time
 * @param period period of time to check
 */
bool HasTimeElapsed(Time now, Time past, Duration period);

/**
 * @brief check if a period of time determined by a frequency has passed
 * @param now current time
 * @param past past time
 * @param frequency  to check
 */
bool HasTimeElapsed(Time now, Time past, double frequency);

}  // namespace eufs::sim2::time

#endif  // INCLUDE_EUFS_SIM2_TIME_TIME_HPP_
