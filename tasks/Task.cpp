/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace kf_orientation;

base::Angle KalmanFilter::computeAngle(const base::Angle& new_acc_angle, double new_gyro_rate, double dt) {

    double cov_gyro = cov[0];
    double cov_process = cov[1];
    double cov_acc = cov[2];

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    gyro_rate = new_gyro_rate - bias;
    angle = base::Angle::fromDeg(angle.getDeg() + dt * gyro_rate);
    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P(0, 0) += dt * (dt * P(1, 1) - P(0, 1) - P(1, 0) + cov_gyro);
    P(0, 1) -= dt * P(1, 1);
    P(1, 0) -= dt * P(1, 1);
    P(1, 1) += cov_process * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P(0, 0) + cov_acc; // Estimate error

    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P(0, 0) / S;
    K[1] = P(1, 0) / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = (new_acc_angle - angle).getDeg(); // Angle difference

    /* Step 6 */
    angle = base::Angle::fromDeg(angle.getDeg() + K[0] * y);
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P(0, 0);
    float P01_temp = P(0, 1);

    P(0, 0) -= K[0] * P00_temp;
    P(0, 1) -= K[0] * P01_temp;
    P(1, 0) -= K[1] * P00_temp;
    P(1, 1) -= K[1] * P01_temp;

    return angle;
}

/**
 * The Kf Orientation orogen component
 */
Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.
bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    kalmanFilterPitch.reset(new KalmanFilter(_filter_tunning_pitch.get()));
    kalmanFilterRoll.reset(new KalmanFilter(_filter_tunning_roll.get()));

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if (previousTime.isNull()) {
        previousTime = base::Time::now();
        return;
    }

    double pressureBar;
    if (_pressure_bar_cmd.read(pressureBar) == RTT::NewData) {
        _pressure_bar_cmd.read(pressureBar);
    }

    base::samples::IMUSensors imuSensors;
    if (_imu_sensors_cmd.read(imuSensors) == RTT::NewData) {
        _imu_sensors_cmd.read(imuSensors);
    }

    double dt = (previousTime - base::Time::now()).toSeconds();
    double acc_x = imuSensors.acc[0];
    double acc_y = imuSensors.acc[1];
    double acc_z = imuSensors.acc[2];

    double gyro_x = imuSensors.gyro[0];
    double gyro_y = imuSensors.gyro[1];

    double mag_x = imuSensors.mag[0];
    double mag_y = imuSensors.mag[1];
    double mag_z = imuSensors.mag[2];

    base::Angle roll = base::Angle::fromRad(atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)));
    base::Angle pitch = base::Angle::fromRad(atan2(-acc_x, acc_z));

    base::Angle kalman_pitch = kalmanFilterPitch->getAngle();
    base::Angle kalman_roll = kalmanFilterRoll->getAngle();

    if (pitch.getDeg() < -90 && kalman_pitch.getDeg() > 90 || pitch.getDeg() > 90 && kalman_pitch.getDeg() < -90) {
        kalmanFilterPitch->setAngle(pitch);
        kalman_pitch = pitch;
    }
    else {
        kalman_pitch = kalmanFilterPitch->computeAngle(pitch, gyro_y, dt);
    }

    if (abs(kalman_pitch.getDeg()) > 90) {
        gyro_x = -gyro_x; // Invert rate, so it fits the restriced accelerometer reading
    }

    kalman_roll = kalmanFilterRoll->computeAngle(roll, gyro_x, dt); // Calculate the angle using a Kalman filter

    double xh = mag_x * cos(kalman_pitch.getRad()) +
                mag_y * sin(kalman_roll.getRad()) * sin(kalman_pitch.getRad()) -
                mag_z * cos(kalman_roll.getRad()) * sin(kalman_pitch.getRad());

    double yh = mag_y * cos(kalman_roll.getRad()) +
                mag_z * sin(kalman_roll.getRad());

    double heading = atan2(-yh, xh);

    if(heading < 0) heading += 2 * M_PI;
    if(heading > 2 * M_PI) heading -= 2 * M_PI;

    base::Angle yaw = base::Angle::fromRad(heading);

    std::cout << "roll: "  << kalman_roll.getDeg() << std::endl;
    std::cout << "pitch: " << kalman_pitch.getDeg() << std::endl;
    std::cout << "yaw: "   << yaw.getDeg() << std::endl;

    double depth = pressureBar * 1.019716;

    base::samples::RigidBodyState pose;
    pose.time = base::Time::now();
    pose.orientation = anglesFromEuler(kalman_roll, kalman_pitch, yaw);
    pose.position = base::Vector3d(0, 0, depth);
    _orientation_samples.write(pose);
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
