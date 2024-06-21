#pragma once

#include <chrono>

namespace Timer {
class TicToc {
   public:
    TicToc() {
        // tic();
    }
    void tic() {
        start = std::chrono::system_clock::now();
    }
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        return time_used.count() * 1000;
    }

   private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
}  // namespace Timer