/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef KF_ORIENTATION_TASK_TASK_HPP
#define KF_ORIENTATION_TASK_TASK_HPP

#include <base/Angle.hpp>
#include "kf_orientation/TaskBase.hpp"

namespace kf_orientation {

class KalmanFilter {

public:
    KalmanFilter(base::Vector3d cov, base::Angle angle = base::Angle::fromDeg(0), double bias = 0) :
        cov(cov), //[0] - cov_gyro, [1] - cov_process,[2]- cov_acc
        angle(angle),
        bias(bias),
        P(base::Matrix2d::Zero()){}

    base::Angle getAngle() const {
        return angle;
    }

    void setAngle(base::Angle angle) {
        this->angle = angle;
    }

    base::Angle computeAngle(const base::Angle& new_acc_angle, double gyro_rate, double dt);

private:
    base::Angle angle;
    base::Vector3d cov;
    base::Matrix2d P;

    double bias;
    double gyro_rate;
};

class Task: public TaskBase {
    friend class TaskBase;
protected:

    base::Time previousTime;

    boost::shared_ptr<KalmanFilter> kalmanFilterPitch;
    boost::shared_ptr<KalmanFilter> kalmanFilterRoll;

    base::Orientation anglesFromEuler(base::Angle roll, base::Angle pitch, base::Angle yaw) {
        Eigen::AngleAxisd roll_angle  (roll.getRad(),  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle (pitch.getRad(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle   (yaw.getRad(),   Eigen::Vector3d::UnitZ());
        return roll_angle * pitch_angle * yaw_angle;
    }

public:

    Task(std::string const& name = "kf_orientation::Task");

    Task(std::string const& name, RTT::ExecutionEngine* engine);

    ~Task();

    bool configureHook();

    bool startHook();

    void updateHook();

    void errorHook();

    void stopHook();

    void cleanupHook();
};
}

#endif

