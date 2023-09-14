#pragma once
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace lh {
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& function_name, double time_useage) {
            function_name_ = function_name;
            time_usage_in_ms_.push_back(time_useage);
        }
        std::string function_name_;
        std::vector<double> time_usage_in_ms_;
    };

    template <typename F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto t1 = std::chrono::steady_clock::now();
        // 完美转发并调用函数
        std::forward<F>(func)();
        auto t2 = std::chrono::steady_clock::now();
        double time_useage = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        // 记录函数调用时间
        if (records_.find(func_name) == records_.end()) {
            // auto record = TimerRecord(func_name, time_useage);
            // records_[func_name] = record;
            records_.insert({func_name, TimerRecord(func_name, time_useage)});
        } else {
            records_[func_name].time_usage_in_ms_.emplace_back(time_useage);
        }
    }

    static void PrintAll();
    static void DumpIntoFile(const std::string& file_name);
    static double GetMeanTime(const std::string& function_name);

    static void Clear() { records_.clear(); }

   private:
    static std::map<std::string, TimerRecord> records_;
};
}  // namespace lh