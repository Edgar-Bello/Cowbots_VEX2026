#include "main.h"
//#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "liblvgl/llemu.hpp"

using namespace pros;

constexpr double TRACK_WIDTH = 12.0; // distance between vertical wheels
constexpr double WHEEL_DIAMETER = 2.125;
constexpr double WHEEL_CIRC = M_PI * WHEEL_DIAMETER;
constexpr double TICKS_PER_REV = 36000.0;
constexpr double INCHES_PER_TICK = WHEEL_CIRC / TICKS_PER_REV;

constexpr double H_OFFSET = 2.5; // horizontal wheel offset (south)

double odomX = 0;
double odomY = 0;
double odomTheta = 0; // radians

Motor frontLeft(2, MotorGearset::green);
Motor frontRight(-1, MotorGearset::green);
Motor backLeft(4, MotorGearset::green);  
Motor backRight(-3, MotorGearset::green);

IMU imu_sensor(14);

Rotation verticalRotation(11); 
Rotation verticalRotation2(12);
Rotation horizontalRotation(13);

void odomTask(void*) {
    int prevVL = verticalRotation.get_position();
    int prevVR = verticalRotation2.get_position();
    int prevH  = horizontalRotation.get_position();
    double prevIMU = imu_sensor.get_rotation();

    while (true) {
        int currVL = verticalRotation.get_position();
        int currVR = verticalRotation2.get_position();
        int currH  = horizontalRotation.get_position();
        double currIMU = imu_sensor.get_rotation();

        // Convert ticks → inches
        double dVL = (currVL - prevVL) * INCHES_PER_TICK;
        double dVR = (currVR - prevVR) * INCHES_PER_TICK;
        double dH  = (currH  - prevH ) * INCHES_PER_TICK;

        // Heading change (deg → rad)
        double dTheta = (currIMU - prevIMU) * M_PI / 180.0;

        // Forward movement
        double dForward = (dVL + dVR) / 2.0;

        // Correct horizontal wheel for rotation
        double dStrafe = dH - dTheta * H_OFFSET;

        // Midpoint integration (more accurate)
        double avgTheta = odomTheta + dTheta / 2.0;

        // Convert to field coordinates
        odomX += dStrafe * cos(avgTheta) - dForward * sin(avgTheta);
        odomY += dStrafe * sin(avgTheta) + dForward * cos(avgTheta);
        odomTheta += dTheta;

        // Wrap angle
        while (odomTheta > M_PI) odomTheta -= 2 * M_PI;
        while (odomTheta < -M_PI) odomTheta += 2 * M_PI;

        // Save previous
        prevVL = currVL;
        prevVR = currVR;
        prevH  = currH;
        prevIMU = currIMU;

        delay(10);
    }
}

void moveToPose(double targetX, double targetY, double targetTheta) {

    double kP_xy = 1;
    double kD_xy = 1;

    double kP_turn = 1;
    double kD_turn = 1;

    double prevXErr = 0;
    double prevYErr = 0;
    double prevTurnErr = 0;

    while (true) {

        double x = odomX;
        double y = odomY;
        double theta = odomTheta;

        double dx = targetX - x;
        double dy = targetY - y;

        double dist = sqrt(dx*dx + dy*dy);

        double pathAngle = atan2(dy, dx);

        double blend = std::min(dist / 24.0, 1.0);
        double desiredTheta = blend * pathAngle + (1 - blend) * targetTheta;

        double dtheta = desiredTheta - theta;
        while (dtheta > M_PI) dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;

        if (dist < 1.0 && fabs(targetTheta - theta) < 0.05) break;

        // Field → robot frame
        double robotX =  dx * cos(theta) + dy * sin(theta);
        double robotY = -dx * sin(theta) + dy * cos(theta);

        double dX = robotX - prevXErr;
        double dY = robotY - prevYErr;
        double dTurn = dtheta - prevTurnErr;

        prevXErr = robotX;
        prevYErr = robotY;
        prevTurnErr = dtheta;

        double vx = kP_xy * robotX + kD_xy * dX;
        double vy = kP_xy * robotY + kD_xy * dY;
        double omega = kP_turn * dtheta + kD_turn * dTurn;

        // X-drive kinematics
        double fl = vy + vx + omega;
        double fr = vy - vx - omega;
        double bl = vy - vx + omega;
        double br = vy + vx - omega;

        double maxVal = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
        if (maxVal > 1) {
            fl /= maxVal; fr /= maxVal; bl /= maxVal; br /= maxVal;
        }

        frontLeft.move(fl * 127);
        frontRight.move(fr * 127);
        backLeft.move(bl * 127);
        backRight.move(br * 127);

        delay(10);
    }

    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void encoderTask(void*) {
    while (true) {
        lcd::print(0, "X: %.2f", odomX);
        lcd::print(1, "Y: %.2f", odomY);
        lcd::print(2, "T: %.2f", odomTheta * 180 / M_PI);

        lcd::print(3, "VL: %d", verticalRotation.get_position());
        lcd::print(4, "VR: %d", verticalRotation2.get_position());
        lcd::print(5, "H : %d", horizontalRotation.get_position());

        delay(100);
    }
}

void initialize() {
    lcd::initialize();
    lcd::set_text(1, "Custom Odom Ready");

    imu_sensor.reset();
    verticalRotation.reset();
    verticalRotation2.reset();
    horizontalRotation.reset();

    verticalRotation.reverse(); // keep if needed

    delay(2000); // allow IMU to calibrate

    Task odom_task(odomTask);
    Task screen_task(encoderTask);
}

void autonomous() {
    moveToPose(-2, 2, 0);
}

void opcontrol() {

	pros::Controller master(pros::E_CONTROLLER_MASTER);

    imu_sensor.tare(); // Set current heading as 0°

    while (true) {

        // Debug: LCD button states
        pros::lcd::print(
            0, "%d %d %d",
            (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
            (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
            (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0
        );

        // Controller inputs
        int forward  = master.get_analog(ANALOG_LEFT_Y);   // Forward / backward
        int strafe   = master.get_analog(ANALOG_LEFT_X);   // Left / right
        int rotation = master.get_analog(ANALOG_RIGHT_X);  // Rotation

        // IMU heading (degrees → radians)
        double direction = imu_sensor.get_rotation();
        double rad = direction * M_PI / 180.0;

        // Field-centric transform
        double temp = forward * cos(rad) + strafe * sin(rad);
        strafe  = -forward * sin(rad) + strafe * cos(rad);
        forward = temp;

        // X-Drive motor calculations
        int fl = forward + strafe + rotation;
        int fr = forward - strafe - rotation;
        int bl = forward - strafe + rotation;
        int br = forward + strafe - rotation;

        // Normalize motor values
        double maxVal = std::max({abs(fl), abs(fr), abs(bl), abs(br)});
        if (maxVal > 127) {
            fl = fl * 127 / maxVal;
            fr = fr * 127 / maxVal;
            bl = bl * 127 / maxVal;
            br = br * 127 / maxVal;
        }

        // Send power to motors
        frontLeft.move(fl);
        frontRight.move(fr);
        backLeft.move(bl);
        backRight.move(br);

        pros::delay(20);
    }

	/*

    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int strafe  = master.get_analog(ANALOG_LEFT_X);
        int rotation = master.get_analog(ANALOG_RIGHT_X);

        int fl = forward + strafe + rotation;
        int fr = forward - strafe - rotation;
        int bl = forward - strafe + rotation;
        int br = forward + strafe - rotation;

        double maxVal = std::max({abs(fl), abs(fr), abs(bl), abs(br)});
        if (maxVal > 127) {
            fl = fl * 127 / maxVal;
            fr = fr * 127 / maxVal;
            bl = bl * 127 / maxVal;
            br = br * 127 / maxVal;
        }

        frontLeft.move(fl);
        frontRight.move(fr);
        backLeft.move(bl);
        backRight.move(br);

        delay(20);
    }

	*/
}