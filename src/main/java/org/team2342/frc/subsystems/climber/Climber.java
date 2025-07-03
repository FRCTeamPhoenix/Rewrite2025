package org.team2342.frc.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private final DumbMotorIO motor;
  private final DumbMotorIOInputsAutoLogged inputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert =
      new Alert("Climber Motor is diconnected", AlertType.kError);

  public Climber(DumbMotorIO motor) {
    this.motor = motor;
    setName("Climber");

    setDefaultCommand(Commands.run(() -> motor.runVoltage(0.0), this));
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    

    motorAlert.set(!inputs.connected);

    ExecutionLogger.log("Climber");
  }

  public Command out() {
    return Commands.run(() -> motor.runVoltage(10.0), this).withName("Climber Out");
  }

  public Command in() {
    return Commands.run(() -> motor.runVoltage(-10.0), this).withName("Climber In");
  }

  public Command stop() {
    return Commands.runOnce(() -> motor.runVoltage(0.0), this).withName("Climber Stop");
  }
}
