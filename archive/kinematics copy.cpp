#include "kinematics.h"

// Constructor to initialize wheel radius and wheel separation
Kinematics::Kinematics(float wheel_radius, float wheel_separation)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {}

// Function to calculate wheel speeds (m/s) based on input linear and angular velocities
Kinematics::WheelSpeeds Kinematics::computeWheelSpeeds(float linear_velocity_x, float angular_velocity_z) {
    WheelSpeeds speeds;

    // Calculate the individual wheel speeds based on the robot's linear and angular velocity
    speeds.motor1 = linear_velocity_x - (angular_velocity_z * wheel_separation_ / 2.0);
    speeds.motor2 = linear_velocity_x + (angular_velocity_z * wheel_separation_ / 2.0);
    speeds.motor3 = speeds.motor1;
    speeds.motor4 = speeds.motor2;

    return speeds;
}

// Function to calculate wheel RPMs based on input linear and angular velocities
Kinematics::MotorRPM Kinematics::calculateRPM(float linear_velocity_x, float angular_velocity_z) {
    MotorRPM rpm;

    // First calculate the wheel speeds using the computeWheelSpeeds function
    WheelSpeeds speeds = computeWheelSpeeds(linear_velocity_x, angular_velocity_z);

    // Convert the wheel speeds (m/s) to RPM
    rpm.motor1 = (speeds.motor1 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor2 = (speeds.motor2 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor3 = (speeds.motor3 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor4 = (speeds.motor4 / (2.0 * M_PI * wheel_radius_)) * 60.0;

    return rpm;
}

// Function to calculate robot velocities based on wheel RPMs
Kinematics::Velocities Kinematics::getVelocities(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm) {
    // Convert RPM to linear velocity (m/s) for each wheel
    float front_left_velocity = (front_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float front_right_velocity = (front_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_left_velocity = (back_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_right_velocity = (back_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;

    // Calculate the average linear velocity (assuming differential drive, so left and right side)
    float linear_velocity_x = (front_left_velocity + back_left_velocity + front_right_velocity + back_right_velocity) / 4.0f;

    // Calculate the angular velocity around the z-axis (yaw)
    // (left wheels velocity - right wheels velocity) / wheel separation gives angular velocity in rad/s
    float angular_velocity_z = ((back_right_velocity + front_right_velocity) - (back_left_velocity + front_left_velocity)) / wheel_separation_;

    // Return the computed velocities
    Velocities velocities;
    velocities.linear_x = linear_velocity_x;
    velocities.linear_y = 0.0f;  // Assuming no lateral movement (sideways velocity)
    velocities.angular_z = angular_velocity_z;
    
    return velocities;
}
