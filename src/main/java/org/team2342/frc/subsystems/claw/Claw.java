// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.claw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ClawConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;
import org.team2342.lib.sensors.distance.DistanceSensorIO;
import org.team2342.lib.sensors.distance.DistanceSensorIOInputsAutoLogged;

public class Claw extends SubsystemBase {

  private final DumbMotorIO motor;
  private final DumbMotorIOInputsAutoLogged motorInputs = new DumbMotorIOInputsAutoLogged();

  private final DistanceSensorIO distanceSensor;
  private final DistanceSensorIOInputsAutoLogged distanceSensorInputs =
      new DistanceSensorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Claw motor is disconnected!", AlertType.kError);
  private final Alert sensorAlert = new Alert("Claw sensor is disconnected!", AlertType.kError);

  public Claw(DumbMotorIO motor, DistanceSensorIO distanceSensor) {
    this.motor = motor;
    this.distanceSensor = distanceSensor;
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Claw/Motor", motorInputs);
    distanceSensor.updateInputs(distanceSensorInputs);
    Logger.processInputs("Claw/Sensor", distanceSensorInputs);

    motorAlert.set(!motorInputs.connected);
    sensorAlert.set(!distanceSensorInputs.connected);

    ExecutionLogger.log("Claw");
  }

  public Command intake() {
    return Commands.run(() -> motor.runVoltage(3.0), this);
  }

  public Command outtake() {
    return Commands.run(() -> motor.runVoltage(-3.0), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> motor.runVoltage(0.0), this);
  }

  public Command intakeUntilCoral() {
    return Commands.sequence(
        intake().until(() -> distanceSensorInputs.distanceMeters < ClawConstants.SENSOR_THRESHOLD),
        stop());
  }
}
