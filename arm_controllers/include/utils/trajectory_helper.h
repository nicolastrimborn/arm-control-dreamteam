
#ifndef TRAJECTORY_HELPER_H
#define TRAJECTORY_HELPER_H

inline double trajectory_generator_pos(double dStart, double dEnd, double dDuration)
{
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
}

inline double trajectory_generator_vel(double dStart, double dEnd, double dDuration)
{
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
}

inline double trajectory_generator_acc(double dStart, double dEnd, double dDuration)
{
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
}		

#endif