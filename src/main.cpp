#include "main.h"

using namespace pros;

MotorGroup frontLeft({3, -4}, MotorGearset::green);
MotorGroup frontRight({-1, 2}, MotorGearset::green);
MotorGroup backLeft({5, -6}, MotorGearset::green);
MotorGroup backRight({-7, 8}, MotorGearset::green);

Motor upperIntake(9, MotorGearset::green);
Motor lowerIntake(10, MotorGearset::green);

Controller master(E_CONTROLLER_MASTER);

IMU imu_sensor(14);

Rotation verticalRotation(11);
Rotation verticalRotation2(12);
Rotation horizontalRotation(13);

bool buttonPressedOnce = false;
bool buttonPressedTwice = false;
double limitspeed = 1.0;

void setBrakeMode(motor_brake_mode_e mode){
    frontLeft.set_brake_mode(mode); //Might have to add the _all to these functions
    frontRight.set_brake_mode(mode);
    backLeft.set_brake_mode(mode);
    backRight.set_brake_mode(mode);
}


void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello Cowbots!");
    pros::lcd::register_btn1_cb(on_center_button);

    imu_sensor.reset();
    verticalRotation.reset();
    verticalRotation2.reset();
    horizontalRotation.reset();
    verticalRotation.reverse();
    setBrakeMode(MOTOR_BRAKE_BRAKE);

    while (imu_sensor.is_calibrating()) {
        pros::delay(10);
    }
    pros::delay(200);
}

void disabled() {
    setBrakeMode(MOTOR_BRAKE_COAST);
}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    while (true) {
        pros::lcd::print(
            0, "%d %d %d",
            (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
            (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
            (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0
        );

        double forward  = -master.get_analog(ANALOG_LEFT_Y);
        double strafe   = -master.get_analog(ANALOG_LEFT_X);
        double rotation = -master.get_analog(ANALOG_RIGHT_X);

        //double rad = imu_sensor.get_rotation() * M_PI / 180.0;

        //double temp = forward * cos(rad) + strafe * sin(rad);
        //strafe = -forward * sin(rad) + strafe * cos(rad);
        //forward = temp;

        double fl = forward + strafe + rotation;
        double fr = forward - strafe - rotation;
        double bl = forward - strafe + rotation;
        double br = forward + strafe - rotation;

        double maxVal = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});
        if (maxVal > 127) {
            fl = fl * 127.0 / maxVal;
            fr = fr * 127.0 / maxVal;
            bl = bl * 127.0 / maxVal;
            br = br * 127.0 / maxVal;
        }

        if(master.get_digital(DIGITAL_A)) {
            upperIntake.move(100); //Max 127
            lowerIntake.move(100);
        } else if(master.get_digital(DIGITAL_B)) {
            upperIntake.move(-100);
            lowerIntake.move(-100);
        } else {
            upperIntake.move(0);
            lowerIntake.move(0);
        }

        if(master.get_digital_new_press(DIGITAL_R1)){
            if(buttonPressedTwice){
                limitspeed = 0.75;
                buttonPressedTwice = false;
            } else if(buttonPressedOnce){
                limitspeed = 0.50;
                buttonPressedOnce = false;
                buttonPressedTwice = true;
            } else {
                limitspeed = 1.0;
                buttonPressedOnce = true;
            }
        }

        frontLeft.move((int)(fl * limitspeed));
        frontRight.move((int)(fr * limitspeed));
        backLeft.move((int)(bl * limitspeed));
        backRight.move((int)(br * limitspeed));

        pros::delay(10);
    }
}