#include "main.h"

using namespace pros;

constexpr double WHEEL_DIAMETER = 2.125;
constexpr double WHEEL_CIRC = M_PI * WHEEL_DIAMETER;
constexpr double TICKS_PER_REV = 36000.0;
constexpr double INCHES_PER_TICK = WHEEL_CIRC / TICKS_PER_REV;
constexpr double H_OFFSET = 5.625; // horizontal wheel offset (south), will probably change

MotorGroup frontLeft({-3, 4}, MotorGearset::blue);
MotorGroup frontRight({1, -2}, MotorGearset::blue);
MotorGroup backLeft({-5, 6}, MotorGearset::blue);
MotorGroup backRight({7, -8}, MotorGearset::blue);

Motor upperIntake(9, MotorGearset::green);
Motor lowerIntake(10, MotorGearset::green);

Controller master(E_CONTROLLER_MASTER);

IMU imu_sensor(14);

Rotation verticalRotation(11);
Rotation verticalRotation2(13); //Might be 12
Rotation horizontalRotation(12); //might be 13

ADIDigitalOut pistonIntake('B'); 
ADIDigitalOut pistonScore('A');
ADIDigitalOut pistonDeScore('D');
ADIDigitalOut pistonCage('C');

double odomX = 0;
double odomY = 0;
double odomTheta = 0; // radians

bool buttonPressedOnce = false;
bool buttonPressedTwice = false;
double limitspeed = 1.0;

bool isScoringPiston = false;
bool isIntakePiston = false;
bool isIntakeCage = false;
bool isDeScorePiston = false;

enum class IntakeState { IDLE, INTAKING, RECOVERING };
IntakeState intakeState = IntakeState::IDLE;

uint32_t recoverStart    = 0;
uint32_t posCheckStart   = 0;
double   posAtCheckStart = 0;

constexpr double   STALL_POS_THRESHOLD  = 50.0;  // ticks moved minimum — tune if needed
constexpr uint32_t STALL_CHECK_WINDOW   = 300;   // ms window to measure movement
constexpr uint32_t RECOVER_DURATION_MS  = 500;  // ms to outtake

void setBrakeMode(motor_brake_mode_e mode){
    frontLeft.set_brake_mode(mode);
    frontRight.set_brake_mode(mode);
    backLeft.set_brake_mode(mode);
    backRight.set_brake_mode(mode);
}

void setOdometry(int& prevVL, int& prevVR, int& prevH, double& prevIMU) {

        lcd::print(0, "X: %.2f", odomX);
        lcd::print(1, "Y: %.2f", odomY);
        lcd::print(2, "T: %.2f", odomTheta * 180 / M_PI);

    //X and Y are switched, change that
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

void turnToHeading(double targetHeadingDeg) {

    constexpr double kP_rot     = 1.8;
    constexpr double MAX_ROT    = 80.0;
    constexpr double STOP_ANGLE = 2.0;
    constexpr uint32_t SETTLE_MS = 150;

    double targetHeadingRad = targetHeadingDeg * M_PI / 180.0;

    int prevVL     = verticalRotation.get_position();
    int prevVR     = verticalRotation2.get_position();
    int prevH      = horizontalRotation.get_position();
    double prevIMU = imu_sensor.get_rotation();

    uint32_t settleStart = 0;
    bool settling = false;

    while (true) {

        setOdometry(prevVL, prevVR, prevH, prevIMU);

        double rotErr = targetHeadingRad - odomTheta;
        while (rotErr >  M_PI) rotErr -= 2 * M_PI;
        while (rotErr < -M_PI) rotErr += 2 * M_PI;
        double rotErrDeg = rotErr * 180.0 / M_PI;

        if (fabs(rotErrDeg) < STOP_ANGLE) {
            if (!settling) {
                settling    = true;
                settleStart = pros::millis();
            } else if (pros::millis() - settleStart >= SETTLE_MS) {
                break;
            }
        } else {
            settling = false;
        }

        double rot = kP_rot * rotErrDeg;
        rot = std::max(-MAX_ROT, std::min(MAX_ROT, rot));

        frontLeft.move( (int) rot);
        frontRight.move((int)-rot);
        backLeft.move(  (int) rot);
        backRight.move( (int)-rot);

        delay(10);
    }

    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void translateTo(double targetX, double targetY) {

    constexpr double kP_xy       = 40.0;
    constexpr double kD_xy       = 15.0;
    constexpr double MAX_POWER   = 127.0;
    constexpr double MIN_POWER   = 20.0;
    constexpr double SLOW_RADIUS = 8.0;
    constexpr double STOP_DIST   = 0.75;
    constexpr uint32_t SETTLE_MS = 150;

    double prevXErr = 0;
    double prevYErr = 0;

    int prevVL     = verticalRotation.get_position();
    int prevVR     = verticalRotation2.get_position();
    int prevH      = horizontalRotation.get_position();
    double prevIMU = imu_sensor.get_rotation();

    uint32_t settleStart = 0;
    bool settling = false;

    while (true) {

        setOdometry(prevVL, prevVR, prevH, prevIMU);

        double dx   = targetX - odomX;
        double dy   = targetY - odomY;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist < STOP_DIST) {
            if (!settling) {
                settling    = true;
                settleStart = pros::millis();
            } else if (pros::millis() - settleStart >= SETTLE_MS) {
                break;
            }
        } else {
            settling = false;
        }

        double robotX = dx * cos(odomTheta) + dy * sin(odomTheta);
        double robotY = -dx * sin(odomTheta) + dy * cos(odomTheta);

        double dX = robotX - prevXErr;
        double dY = robotY - prevYErr;
        prevXErr  = robotX;
        prevYErr  = robotY;

        double vx = kP_xy * robotX + kD_xy * dX;
        double vy = kP_xy * robotY + kD_xy * dY;

        double speedScale = 1.0;
        if (dist < SLOW_RADIUS) {
            speedScale = std::max(dist / SLOW_RADIUS, MIN_POWER / MAX_POWER);
        }

        double fl = vy + vx;
        double fr = vy - vx;
        double bl = vy - vx;
        double br = vy + vx;

        double maxTrans = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
        if (maxTrans > 0) {
            fl = (fl / maxTrans) * MAX_POWER * speedScale;
            fr = (fr / maxTrans) * MAX_POWER * speedScale;
            bl = (bl / maxTrans) * MAX_POWER * speedScale;
            br = (br / maxTrans) * MAX_POWER * speedScale;
        }

        frontLeft.move((int)fl);
        frontRight.move((int)fr);
        backLeft.move((int)bl);
        backRight.move((int)br);

        delay(10);
    }

    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void moveToPose(double targetX, double targetY, double targetHeadingDeg) {

    // Compute halfway heading between current and target
    // keeping it in [-180, 180] to always take the short arc
    double currentDeg = odomTheta * 180.0 / M_PI;

    double diff = targetHeadingDeg - currentDeg;
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    double midHeadingDeg = currentDeg + diff * 0.5;

    // Step 1 — rotate halfway
    turnToHeading(midHeadingDeg);

    // Step 2 — translate to XY holding the mid heading (odomTheta is now midHeading)
    translateTo(targetX, targetY);

    // Step 3 — rotate the rest to final heading
    turnToHeading(targetHeadingDeg);
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
    pros::lcd::set_text(1, "Hello UTRGV!");
    pros::lcd::register_btn1_cb(on_center_button);

    imu_sensor.reset();
    verticalRotation.reset();
    verticalRotation2.reset();
    horizontalRotation.reset();
    verticalRotation.reverse();
    horizontalRotation.reverse();

    lowerIntake.set_reversed(true);
    upperIntake.set_reversed(true);

    setBrakeMode(MOTOR_BRAKE_COAST);

    pistonIntake.set_value(false);
    pistonCage.set_value(false);
    pistonDeScore.set_value(false);
    pistonScore.set_value(false);

    while (imu_sensor.is_calibrating()) {
        pros::delay(10);
    }

    pros::delay(200);
}

void disabled() {
    setBrakeMode(MOTOR_BRAKE_COAST);
}

void competition_initialize() {

}

void autonomous() {
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveToPose(0, 20, 90); 
}

void opcontrol() {

    int prevVL = verticalRotation.get_position();
    int prevVR = verticalRotation2.get_position();
    int prevH  = horizontalRotation.get_position();
    double prevIMU = imu_sensor.get_rotation();

    while (true) {
        lcd::print(
            0, "%d %d %d",
            (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
            (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
            (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0
        );

        lcd::print(0, "X: %.2f", odomX);
        lcd::print(1, "Y: %.2f", odomY);
        lcd::print(2, "T: %.2f", odomTheta * 180 / M_PI);

        lcd::print(3, "VL: %d", verticalRotation.get_position());
        lcd::print(4, "VR: %d", verticalRotation2.get_position());
        lcd::print(5, "H : %d", horizontalRotation.get_position());

        delay(100);

        setOdometry(prevVL, prevVR, prevH, prevIMU);

        double forward  = master.get_analog(ANALOG_LEFT_Y);
        double strafe   = master.get_analog(ANALOG_LEFT_X);
        double rotation = master.get_analog(ANALOG_RIGHT_X);

        double rad = imu_sensor.get_rotation() * M_PI / 180.0;

        double temp = forward * cos(rad) + strafe * sin(rad);
        strafe = -forward * sin(rad) + strafe * cos(rad);
        forward = temp;

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

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
            if (buttonPressedTwice) {
                limitspeed = 0.50;
                buttonPressedTwice = false;
            } else if (buttonPressedOnce) {
                limitspeed = 0.75;
                buttonPressedOnce = false;
                buttonPressedTwice = true;
            } else {
                limitspeed = 1.0;
                buttonPressedOnce = true;
            }
        }

        int intakeSpeed = 0;

if (master.get_digital(DIGITAL_R1)) {
    intakeSpeed = 12000;
} else if (master.get_digital(DIGITAL_R2)) {
    intakeSpeed = -12000;
} else if (master.get_digital(DIGITAL_X) || master.get_digital(DIGITAL_A)) {
    intakeSpeed = 12000;
}


//Add this to the autonomous part
/*
uint32_t now = pros::millis();

if (intakeState == IntakeState::RECOVERING) {
    upperIntake.move_voltage(-12000);
    lowerIntake.move_voltage(-12000);

    if (now - recoverStart >= RECOVER_DURATION_MS) {
        intakeState    = IntakeState::INTAKING;
        posCheckStart  = now;
        posAtCheckStart = upperIntake.get_position();
    }

} else if (intakeSpeed > 0) {
    intakeState = IntakeState::INTAKING;
    upperIntake.move_voltage(intakeSpeed);
    lowerIntake.move_voltage(intakeSpeed);

    if (now - posCheckStart >= STALL_CHECK_WINDOW) {
        double currentPos = upperIntake.get_position();
        double delta      = std::abs(currentPos - posAtCheckStart);

        if (delta < STALL_POS_THRESHOLD) {
            intakeState  = IntakeState::RECOVERING;
            recoverStart = now;
        }

        // Reset window regardless
        posCheckStart   = now;
        posAtCheckStart = currentPos;
    }

} else {
    // Idle or outtaking — pass through, reset stall tracking
    intakeState     = IntakeState::IDLE;
    posCheckStart   = now;
    posAtCheckStart = upperIntake.get_position();
    upperIntake.move_voltage(intakeSpeed);
    lowerIntake.move_voltage(intakeSpeed);
}  

*/


    //If using the unstuck method for teleop comment this part
        upperIntake.move_voltage(intakeSpeed);
        lowerIntake.move_voltage(intakeSpeed);

        if (master.get_digital_new_press(DIGITAL_L1)) {
            pistonIntake.set_value(true);
        }
        if (master.get_digital_new_release(DIGITAL_L1)) {
            pistonIntake.set_value(false);
        }

        if (master.get_digital_new_press(DIGITAL_X)) {
            pistonScore.set_value(true);
            pistonCage.set_value(true);
            isIntakeCage = true;
        }
        if (master.get_digital_new_release(DIGITAL_X)) {
            pistonScore.set_value(false);        }

        if (master.get_digital_new_press(DIGITAL_A)) {
            pistonScore.set_value(true);
        }
        if (master.get_digital_new_release(DIGITAL_A)) {
            pistonScore.set_value(false);
        }

        if (master.get_digital_new_press(DIGITAL_L2)) {
            isDeScorePiston = !isDeScorePiston;
            pistonDeScore.set_value(isDeScorePiston);
        }

        if (master.get_digital_new_press(DIGITAL_B)) {
            isIntakeCage = !isIntakeCage;
            pistonCage.set_value(isIntakeCage);
        }

        if(master.get_digital_new_press(DIGITAL_UP)){
            imu_sensor.reset();
            while (imu_sensor.is_calibrating()) {
            pros::delay(10);
            }
        pros::delay(200);
        }

        frontLeft.move((int)(fl * limitspeed));
        frontRight.move((int)(fr * limitspeed));
        backLeft.move((int)(bl * limitspeed));
        backRight.move((int)(br * limitspeed));

        pros::delay(10);
    }

}
