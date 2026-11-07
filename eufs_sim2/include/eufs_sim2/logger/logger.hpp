#ifndef INCLUDE_EUFS_SIM2_LOGGER_LOGGER_HPP_
#define INCLUDE_EUFS_SIM2_LOGGER_LOGGER_HPP_

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace eufs::sim2::logging {

/**
 * @brief Initialise and register the logger
 * @param log_path Path to the log file
 * @param logger_name Name of logger
 * @return Pointer to the logger
 */
inline auto CreateLogger(const std::string &log_path, const std::string &logger_name = "eufs_sim2",
                         bool register_logger = false) {
  // Create and configure logger sinks
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::info);
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, true);
  file_sink->set_level(spdlog::level::trace);
  spdlog::sinks_init_list sink_list = {file_sink, console_sink};

  // Create and configure logger
  auto logger = std::make_shared<spdlog::logger>(logger_name, sink_list.begin(), sink_list.end());
  logger->set_level(spdlog::level::trace);
  logger->flush_on(spdlog::level::trace);

  // Register logger so its available when using `GetLogger`.
  if (register_logger) spdlog::register_logger(logger);

  return logger;
}

/**
 * @brief Get the logger instance
 * @param logger_name Name of the logger
 * @return Pointer to the logger
 */
inline auto GetLogger(const std::string &logger_name = "eufs_sim2") {
  return spdlog::get(logger_name);
}

}  // namespace eufs::sim2::logging

#endif  // INCLUDE_EUFS_SIM2_LOGGER_LOGGER_HPP_
