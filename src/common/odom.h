#ifndef ODOM_H
#define ODOM_H

namespace lh {
struct Odom {
    Odom() {}
    Odom(double timestamp, double left_pulse, double right_pulse)
        : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) {}

    double timestamp_ = 0.0;
    double left_pulse_ = 0.0;
    double right_pulse_ = 0.0;
};
}  // namespace lh

#endif