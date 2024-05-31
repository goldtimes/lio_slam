#pragma once

namespace sensors{
    struct Odom{
        Odom() {}
        Odom(double t, double left_pulse, double right_pulse ) : timestamp_(t), left_pulse_(left_pulse), right_pulse_(right_pulse){}

        double timestamp_ = 0.0;
        // 脉冲
        double left_pulse_ = 0.0;
        double right_pulse_ = 0.0;
    };
}