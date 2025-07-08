// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ElevatorConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Elevator extends SubsystemBase {

  private final SmartMotorIO motors;
  private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();

  private final Alert motor1Alert =
      new Alert("Elevator lead motor is disconnected!", AlertType.kError);
  private final Alert motor2Alert =
      new Alert("Elevator follower motor is disconnected!", AlertType.kError);

  public Elevator(SmartMotorIO motors) {
    this.motors = motors;

    setName("Elevator");
    setDefaultCommand(run(() -> motors.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motors.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motor1Alert.set(!inputs.motorsConnected[0]);
    motor2Alert.set(!inputs.motorsConnected[1]);

    ExecutionLogger.log("Elevator");
  }

  @AutoLogOutput(key = "Elevator/HeightMeters")
  public double getHeightMeters() {
    return inputs.positionRad / ElevatorConstants.M_TO_RAD;
  }

  public Command goToHeight(double targetHeightMeters) {
    return run(() -> motors.runPosition(targetHeightMeters * ElevatorConstants.M_TO_RAD))
        .until(
            () ->
                Math.abs(targetHeightMeters - getHeightMeters())
                    <= ElevatorConstants.AT_GOAL_TOLERANCE)
        .withName("Elevator Go To Height");
  }

  public Command holdHeight(double heightMeters) {
    return run(() -> motors.runPosition(heightMeters * ElevatorConstants.M_TO_RAD))
        .withName("Elevator Hold Height");
  }

  public void zeroElevator() {
    // motors.setPosition(0);
  }

  public Command stop() {
    return runOnce(() -> motors.runVoltage(0.0));
  }
}
