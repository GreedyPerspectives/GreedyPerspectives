/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

// Wrapper for the boost logging library
#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_LOGGING_H
#define VISUALGROUPCOVERAGEPLANNER_LOGGING_H

#define AI_LOGGING_ENABLED 1
#include <AIToolbox/Logging.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <fmt/ranges.h>
#include <iomanip>
#include <memory>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>

static void aiToolBoxLogger(int severity, const char * message)
{
    switch(severity)
    {
    case 0:
        spdlog::debug("AI-TOOLBOX: [{}]", message);
        break;
    case 1:
        spdlog::info("AI-TOOLBOX: [{}]", message);
        break;
    case 2:
        spdlog::warn("AI-TOOLBOX: [{}]", message);
        break;
    case 3:
        spdlog::error("AI-TOOLBOX: [{}]", message);
        break;
    }
}
// Define formatter<Eigen::Matrix<T, Rows, Cols>>
template<typename T>
struct fmt::formatter<T,
                      std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
  : ostream_formatter
{};

namespace logging
{
    inline void init()
    {
        auto time = std::time(nullptr);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time),
                            "%F_%T"); // ISO 8601 without timezone information.
        auto s = ss.str();
        std::replace(s.begin(), s.end(), ':', '-');

        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(
            std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/" + s + ".log"));
        sinks.push_back(std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>());
        auto combined_logger =
            std::make_shared<spdlog::logger>("name", begin(sinks), end(sinks));
        spdlog::set_default_logger(combined_logger);

        spdlog::set_pattern("[%H:%M:%S.%f] [%^%L%$] %v");
        spdlog::set_level(spdlog::level::info); // Set global log level
        spdlog::info("Starting to log...");

        AIToolbox::AILogger = aiToolBoxLogger;
    }
} // namespace logging

#endif // VISUALGROUPCOVERAGEPLANNER_LOGGING_H
