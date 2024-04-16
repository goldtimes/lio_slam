#pragma once

namespace ctlio::slam {
struct Odom {
    Odom() = default;
    Odom(double time, double left_pluse, double right_pluse)
        : timestamped_(time), left_pluse_(left_pluse), right_pluse_(right_pluse) {
    }
    double timestamped_ = 0.0;
    double left_pluse_ = 0.0;
    double right_pluse_ = 0.0;
};

}  // namespace ctlio::slam