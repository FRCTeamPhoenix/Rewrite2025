package org.team2342.frc.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;
import org.team2342.lib.motors.dumb.DumbMotorIOTalonFX;

public class Climber extends SubsystemBase {
  private final DumbMotorIOTalonFX climberTalon;
  private final DumbMotorIOInputsAutoLogged inputs = new DumbMotorIOInputsAutoLogged();

  private double volts = 0.0;

  public Climber(DumbMotorIOTalonFX climberTalon) {
    this.climberTalon = climberTalon;
  }

  @Override
  public void periodic() {
    climberTalon.updateInputs(inputs);
    climberTalon.runVoltage(volts);
    Logger.processInputs("Climber", inputs);

    ExecutionLogger.log("Climber/Periodic");
  }
}
