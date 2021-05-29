#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>

/**
 * Does vanilla initialization of glog and gflags.
 * Usage:
 * RPG_COMMON_MAIN
 * {
 *   // same body as usually, with argc and argv
 * }
 * You will need to add gflags_catkin and glog_catkin
 * as dependency to package.xml .
 */
#define RPG_COMMON_MAIN                                    \
    int rpg_common_main(int argc, char** argv);            \
    int main(int argc, char** argv) {                      \
        google::InitGoogleLogging(argv[0]);                \
        google::ParseCommandLineFlags(&argc, &argv, true); \
        google::InstallFailureSignalHandler();             \
        FLAGS_alsologtostderr = true;                      \
        FLAGS_colorlogtostderr = true;                     \
        return rpg_common_main(argc, argv);                \
    }                                                      \
    int rpg_common_main(int argc, char** argv)

/**
 * "Starts" chrono timer (high resolution clock)
 * Usage:
 * TIMER_START(t1);
 * _t1 is the name of the first time taken.
 * Remark: use the ";" at the end!
 */
#define TIMER_START(t1) auto t1 = std::chrono::high_resolution_clock::now()
/**
 * "Stops" the chrono timer and compute time
 * Usage:
 * TIMER_STOP(t1,t2,duration);
 * Remark: use the ";" at the end!
 * @see TIMER_START
 */
#define TIMER_STOP(t1, t2, duration)                     \
    auto t2 = std::chrono::high_resolution_clock::now(); \
    auto duration =                                      \
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()