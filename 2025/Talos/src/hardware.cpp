#include "vex.h"
#include "hardware.h"
#include <algorithm>
#include <cmath>

using namespace vex;

brain Brain;
controller Controller;

motor LeftFront(LEFT_FRONT_PORT, true);
motor LeftMiddle(LEFT_MIDDLE_PORT, true);
motor LeftBack(LEFT_BACK_PORT, true);

motor RightFront(RIGHT_FRONT_PORT);
motor RightMiddle(RIGHT_MIDDLE_PORT);
motor RightBack(RIGHT_BACK_PORT);

motor IntakeFrontMiddle(INTAKE_FRONT_MIDDLE_PORT);
motor IntakeFrontTop(INTAKE_FRONT_TOP_PORT);
motor IntakeBack(INTAKE_BACK_PORT);

motor_group Left(LeftFront, LeftMiddle, LeftBack);
motor_group Right(RightFront, RightMiddle, RightBack);

pneumatics Pneumatics(Brain.ThreeWirePort.A);

namespace {

double applyDeadband(double value) {
    return (std::fabs(value) < kDriveDeadband) ? 0.0 : value;
}

double clampPercent(double value) {
    return std::clamp(value, -100.0, 100.0);
}

void commandDriveSide(motor_group &group, double speed) {
    if (std::fabs(speed) <= kDriveStopThreshold) {
        group.stop(brakeType::brake);
        return;
    }

    group.spin(directionType::fwd, speed, velocityUnits::pct);
}

const char *intakeStatusMessage(IntakeState state) {
    switch (state) {
        case INTAKE:
            return "Intaking";
        case OUTTAKE_TO_TOP:
            return "Outtaking to top";
        case OUTTAKE_TO_BOTTOM:
            return "Outtaking to bottom";
        case NEUTRAL:
            return "Neutral";
        default:
            return "Unknown state";
    }
}

void updateControllerStatus(IntakeState state) {
    static bool firstUpdate = true;
    static IntakeState lastState = NEUTRAL;

    if (!firstUpdate && state == lastState) {
        return;
    }

    firstUpdate = false;
    lastState = state;

    Controller.Screen.clearLine(1);
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(intakeStatusMessage(state));
}

void spinIntake(vex::motor &device, directionType direction) {
    device.spin(direction, kIntakeMotorSpeed, velocityUnits::pct);
}

void stopIntake(vex::motor &device) {
    device.stop(brakeType::brake);
}

void stopAllIntake() {
    stopIntake(IntakeFrontMiddle);
    stopIntake(IntakeFrontTop);
    stopIntake(IntakeBack);
}

} // namespace

void robotDrive(double frontBackSpeed, double turnSpeed) {
    const double processedForward = applyDeadband(frontBackSpeed);
    const double processedTurn = applyDeadband(turnSpeed);

    const double leftSideSpeed = clampPercent((processedForward + processedTurn) * 0.98);
    const double rightSideSpeed = clampPercent(processedForward - processedTurn);

    commandDriveSide(Left, leftSideSpeed);
    commandDriveSide(Right, rightSideSpeed);
}

void intakeMechanism(IntakeState intakeState) {
    static bool extended = false;

    auto setRampExtended = [&](bool shouldExtend) {
        if (!kUsePneumatics) {
            extended = shouldExtend;
            return;
        }

        if (extended != shouldExtend) {
            Pneumatics.set(shouldExtend);
            extended = shouldExtend;
        }
    };

    switch (intakeState) {
        case INTAKE:
            spinIntake(IntakeBack, directionType::fwd);
            spinIntake(IntakeFrontMiddle, directionType::fwd);
            stopIntake(IntakeFrontTop);
            setRampExtended(true);
            break;

        case OUTTAKE_TO_TOP:
            spinIntake(IntakeFrontMiddle, directionType::fwd);
            spinIntake(IntakeBack, directionType::rev);
            spinIntake(IntakeFrontTop, directionType::fwd);
            setRampExtended(false);
            break;

        case OUTTAKE_TO_BOTTOM:
            spinIntake(IntakeFrontMiddle, directionType::rev);
            spinIntake(IntakeBack, directionType::rev);
            stopIntake(IntakeFrontTop);
            setRampExtended(false);
            break;

        case NEUTRAL:
            stopAllIntake();
            break;

        default:
            stopAllIntake();
            setRampExtended(false);
            break;
    }

    updateControllerStatus(intakeState);
}
