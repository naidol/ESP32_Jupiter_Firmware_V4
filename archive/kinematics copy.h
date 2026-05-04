#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>

class Kinematics {
public:
    struct Velocities {
        float linear_x;  // Linear velocity in the x direction (forward)
        float linear_y;  // Linear velocity in the y direction (sideways, usually zero for diff-drive)
        float angular_z; // Angular velocity around the z axis (yaw)
    };

    struct WheelSpeeds {
        float motor1;   // Linear velocity of the front-left wheel (m/s)
        float motor2;  // Linear velocity of the front-right wheel (m/s)
        float motor3;    // Linear velocity of the back-left wheel (m/s)
        float motor4;   // Linear velocity of the back-right wheel (m/s)
    };

    struct MotorRPM {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    // Constructor to initialize wheel radius and wheel separation
    Kinematics(float wheel_radius, float wheel_separation);

    // Function to calculate wheel speeds (m/s) based on input linear and angular velocities
    WheelSpeeds computeWheelSpeeds(float linear_velocity_x, float angular_velocity_z);

    // Function to calculate wheel RPMs based on input linear and angular velocities
    MotorRPM calculateRPM(float linear_velocity_x, float angular_velocity_z);

    // Function to calculate robot velocities based on wheel RPMs
    Velocities getVelocities(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm);

private:
    float wheel_radius_;      // Radius of the wheel (meters)
    float wheel_separation_;  // Distance between the left and right wheels (meters)
};

#endif // KINEMATICS_H
