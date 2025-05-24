#pragma once

// Declarations for motor simulation functions will go here.

void initializeMotorSimulation();
void updateMotorState(double inputVoltage, double deltaTime);
double getMotorSpeed();
double getMotorPosition();
void resetMotorSimulation();