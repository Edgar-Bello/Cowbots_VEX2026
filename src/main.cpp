#include "main.h"

using namespace pros;

constexpr double WHEEL_DIAMETER = 2.125;
constexpr double WHEEL_CIRC = M_PI * WHEEL_DIAMETER;
constexpr double TICKS_PER_REV = 36000.0;
constexpr double INCHES_PER_TICK = WHEEL_CIRC / TICKS_PER_REV;
constexpr double H_OFFSET = 5.81;

MotorGroup frontLeft({-3, 4}, MotorGearset::blue);
MotorGroup frontRight({1, -2}, MotorGearset::blue);
MotorGroup backLeft({-5, 6}, MotorGearset::blue);
MotorGroup backRight({7, -8}, MotorGearset::blue);

Motor upperIntake(9, MotorGearset::green);
Motor lowerIntake(10, MotorGearset::green);

Controller master(E_CONTROLLER_MASTER);

IMU imu_sensor(14);

Rotation verticalRotation(11);
Rotation verticalRotation2(13);
Rotation horizontalRotation(12);

ADIDigitalOut pistonIntake('B');
ADIDigitalOut pistonScore('A');
ADIDigitalOut pistonDeScore('D');
ADIDigitalOut pistonCage('C');

double odomX = 0;
double odomY = 0;
double odomTheta = 0; // radians, CCW positive (internal)

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

constexpr double   STALL_POS_THRESHOLD  = 50.0;
constexpr uint32_t STALL_CHECK_WINDOW   = 300;
constexpr uint32_t RECOVER_DURATION_MS  = 500;

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

    int currVL = verticalRotation.get_position();
    int currVR = verticalRotation2.get_position();
    int currH  = horizontalRotation.get_position();

    double currIMU = -imu_sensor.get_rotation(); // sensor CW+ -> internal CCW+

    double dVL = (currVL - prevVL) * INCHES_PER_TICK;
    double dVR = (currVR - prevVR) * INCHES_PER_TICK;
    double dH  = (currH  - prevH ) * INCHES_PER_TICK;

    double dTheta = (currIMU - prevIMU) * M_PI / 180.0;

    double dForward = (dVL + dVR) / 2.0;

    // If your strafe drifts when turning, flip the sign on the dTheta*H_OFFSET term.
    double dStrafe = dH - dTheta * H_OFFSET;

    double avgTheta = odomTheta + dTheta / 2.0;

    odomX += dStrafe * cos(avgTheta) - dForward * sin(avgTheta);
    odomY += dStrafe * sin(avgTheta) + dForward * cos(avgTheta);
    odomTheta += dTheta;

    while (odomTheta > M_PI) odomTheta -= 2 * M_PI;
    while (odomTheta < -M_PI) odomTheta += 2 * M_PI;

    prevVL = currVL;
    prevVR = currVR;
    prevH  = currH;
    prevIMU = currIMU;
}

void moveToPose(double targetX, double targetY, double targetHeadingDeg) {

    // --- TUNING ---
    constexpr double kP_xy      = 5.0;
    constexpr double kD_xy      = 18.0;

    constexpr double kD_rot     = 10.0;   // Might change to 5.0
    double kP_rot     = 0.7; // Will be dynamically adjusted based on error

    constexpr double MAX_POWER  = 127.0;
    constexpr double MIN_POWER  = 10.0;

    constexpr double SLOW_RADIUS = 10.0;
    constexpr double STOP_DIST   = 0.75;
    constexpr double STOP_ANGLE  = 5.0;

    constexpr double ROT_MAX  = 40.0;
    constexpr double TRANS_MAX = MAX_POWER - ROT_MAX;

    constexpr uint32_t XY_SETTLE_MS  = 250;
    constexpr uint32_t ROT_SETTLE_MS = 300;

    double targetHeadingRad = (-targetHeadingDeg) * M_PI / 180.0;

    double prevXErr = 0;
    double prevYErr = 0;
    double prevRotErr = 0;

    int prevVL     = verticalRotation.get_position();
    int prevVR     = verticalRotation2.get_position();
    int prevH      = horizontalRotation.get_position();
    double prevIMU = -imu_sensor.get_rotation();

    // --- INITIAL ROTATION ERROR ---
    double initialRotErr = targetHeadingRad - odomTheta;
    while (initialRotErr >  M_PI) initialRotErr -= 2 * M_PI;
    while (initialRotErr < -M_PI) initialRotErr += 2 * M_PI;

    double initialRotErrDeg = fabs(initialRotErr * 180.0 / M_PI);

    uint32_t xySettleStart  = 0;
    uint32_t rotSettleStart = 0;
    bool xyFirstHit  = false;
    bool rotFirstHit = false;

    while (true) {

        setOdometry(prevVL, prevVR, prevH, prevIMU);

        // --- POSITION ERROR ---
        double dx   = targetX - odomX;
        double dy   = targetY - odomY;
        double dist = sqrt(dx * dx + dy * dy);

        // --- ROTATION ERROR ---
        double rotErr = targetHeadingRad - odomTheta;
        while (rotErr >  M_PI) rotErr -= 2 * M_PI;
        while (rotErr < -M_PI) rotErr += 2 * M_PI;
        double rotErrDeg = rotErr * 180.0 / M_PI;

        double currentAbsErr = fabs(rotErrDeg);

        // --- SETTLING LOGIC ---
        if (dist < STOP_DIST && !xyFirstHit) {
            xyFirstHit = true;
            xySettleStart = pros::millis();
        }

        if (currentAbsErr < STOP_ANGLE && !rotFirstHit) {
            rotFirstHit = true;
            rotSettleStart = pros::millis();
        }

        bool xyDone  = xyFirstHit  && (pros::millis() - xySettleStart  >= XY_SETTLE_MS);
        bool rotDone = rotFirstHit && (pros::millis() - rotSettleStart >= ROT_SETTLE_MS);

        if (xyDone && rotDone) break;

        // --- FIELD -> ROBOT FRAME ---
        double robotX =  dx * cos(odomTheta) + dy * sin(odomTheta);
        double robotY = -dx * sin(odomTheta) + dy * cos(odomTheta);

        // --- TRANSLATION PD ---
        double dX = robotX - prevXErr;
        double dY = robotY - prevYErr;
        prevXErr = robotX;
        prevYErr = robotY;

        double vx = kP_xy * robotX + kD_xy * dX;
        double vy = kP_xy * robotY + kD_xy * dY;

        // --- SLOW DOWN NEAR TARGET ---
        double speedScale = 1.0;
        if (dist < SLOW_RADIUS) {
            speedScale = std::max(dist / SLOW_RADIUS, MIN_POWER / MAX_POWER);
            kP_rot = 0.045;
        }

        // --- MECANUM MIX ---
        double fl_t = vy + vx;
        double fr_t = vy - vx;
        double bl_t = vy - vx;
        double br_t = vy + vx;

        double maxTrans = std::max({fabs(fl_t), fabs(fr_t), fabs(bl_t), fabs(br_t)});
        if (maxTrans > 0) {
            fl_t = (fl_t / maxTrans) * TRANS_MAX * speedScale;
            fr_t = (fr_t / maxTrans) * TRANS_MAX * speedScale;
            bl_t = (bl_t / maxTrans) * TRANS_MAX * speedScale;
            br_t = (br_t / maxTrans) * TRANS_MAX * speedScale;
        }

        // --- ROTATION PD WITH DYNAMIC kP ---
        double dRot = rotErrDeg - prevRotErr;
        prevRotErr = rotErrDeg;

        double rot = 0;

        // kP switching
        if (currentAbsErr > 5.0) {
            rot = kP_rot * rotErrDeg + kD_rot * dRot;

            rot = std::max(-ROT_MAX, std::min(ROT_MAX, rot));

            // taper near goal
            if (currentAbsErr < 20.0) {
                rot *= (currentAbsErr / 20.0);
            }
        }

        // --- FINAL MIX ---
        double fl = fl_t + rot;
        double fr = fr_t - rot;
        double bl = bl_t + rot;
        double br = br_t - rot;

        double maxFinal = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
        if (maxFinal > MAX_POWER) {
            fl = fl / maxFinal * MAX_POWER;
            fr = fr / maxFinal * MAX_POWER;
            bl = bl / maxFinal * MAX_POWER;
            br = br / maxFinal * MAX_POWER;
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
    pros::lcd::set_text(1, "Hello COWBOTS!");
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

void competition_initialize() {}

void autonomous() {
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveToPose(0, 20, 90);
}

void opcontrol() {

    int prevVL = verticalRotation.get_position();
    int prevVR = verticalRotation2.get_position();
    int prevH  = horizontalRotation.get_position();
    double prevIMU = -imu_sensor.get_rotation();

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

        double rad = (-imu_sensor.get_rotation()) * M_PI / 180.0;

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

        upperIntake.move_voltage(intakeSpeed);
        lowerIntake.move_voltage(intakeSpeed);

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            pistonIntake.set_value(true);
        }
        if (master.get_digital_new_release(E_CONTROLLER_DIGITAL_L1)) {
            pistonIntake.set_value(false);
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
            pistonScore.set_value(true);
            pistonCage.set_value(true);
            isIntakeCage = true;
        }
        if (master.get_digital_new_release(E_CONTROLLER_DIGITAL_X)) {
            pistonScore.set_value(false);
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            pistonScore.set_value(true);
        }
        if (master.get_digital_new_release(E_CONTROLLER_DIGITAL_A)) {
            pistonScore.set_value(false);
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
            isDeScorePiston = !isDeScorePiston;
            pistonDeScore.set_value(isDeScorePiston);
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
            isIntakeCage = !isIntakeCage;
            pistonCage.set_value(isIntakeCage);
        }

        if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
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