#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController(int8_t& motorPowerTL, int8_t& motorPowerTR, int8_t& motorPowerBR, int8_t& motorPowerBL)
 : motorPowerTL(motorPowerTL), motorPowerTR(motorPowerTR), motorPowerBR(motorPowerBR), motorPowerBL(motorPowerBL) {}

void MotorController::setTargetValues(float *targetVelocity, float *targetPitch, float *targetRoll) {
    velocityController.setTarget(targetVelocity);
    pitchController.setTarget(targetPitch);
    rollController.setTarget(targetRoll);
}

void MotorController::setVelocityConstants(const PID_Instructions &values) {
    velocityController.setConstants(values);
}

void MotorController::setPitchConstants(const PID_Instructions &values) {
    pitchController.setConstants(values);
}

void MotorController::setRollConstants(const PID_Instructions &values) {
    rollController.setConstants(values);
}

void MotorController::calculatePower(float velocity, float pitch, float roll, float deltaTime) {
    float basePower = velocityController.calculate(velocity, deltaTime);
    basePower = constrain(basePower, -127, 127);
    float pitchShift = pitchController.calculate(pitch, deltaTime);
    float rollShift = rollController.calculate(roll, deltaTime);

    motorPowerTL = int8_t(constrain((basePower + pitchShift + rollShift), -127, 127));
    motorPowerTR = int8_t(constrain((basePower + pitchShift - rollShift), -127, 127));
    motorPowerBR = int8_t(constrain((basePower - pitchShift - rollShift), -127, 127));
    motorPowerBL = int8_t(constrain((basePower - pitchShift + rollShift), -127, 127));
}