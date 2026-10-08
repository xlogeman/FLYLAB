/**
 * 1Hz 节流打印宏: 无论所在循环/回调被调多频繁, 每个调用点(按代码行区分)
 * 最多每 1 秒输出一次, 用于避免规划失败时 15~100Hz 级联刷屏.
 *
 * 用法:
 *   LOG_THROTTLE([&] { cout << "msg" << endl; });
 *   LOG_THROTTLE([&] { ros_ptr_->warn("msg {}", x); });
 */
#pragma once

#include <chrono>

#define LOG_THROTTLE(print_expr)                                                          \
    do {                                                                                  \
        static double _log_throttle_last_t = -1e18;                                       \
        double _log_throttle_now = std::chrono::duration<double>(                         \
            std::chrono::steady_clock::now().time_since_epoch()).count();                 \
        if (_log_throttle_now - _log_throttle_last_t > 1.0) {                             \
            _log_throttle_last_t = _log_throttle_now;                                     \
            print_expr;                                                                   \
        }                                                                                 \
    } while (0)
