// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.WristConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIO;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIOInputsAutoLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Wrist extends SubsystemBase{
    private final SmartMotorIO motor; 
    private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();

    private final AbsoluteEncoderIO encoder; 
    private final AbsoluteEncoderIOInputsAutoLogged encoderInputs = new AbsoluteEncoderIOInputsAutoLogged();

    private final Alert motorAlert = new Alert("Wrist motor is disconnected!", AlertType.kError);
    private final Alert encoderAlert = new Alert("Wrist Encoder is disconnected!", AlertType.kError);

    private boolean reset = false;

    public Wrist(SmartMotorIO motor, AbsoluteEncoderIO encoder){
        this.motor = motor;
        this.encoder = encoder;

        setName("Wrist");
        setDefaultCommand(run(() -> motor.runVoltage(0.0)));
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
        encoder.updateInputs(encoderInputs);

        Logger.processInputs("Wrist/Motor", inputs);
        Logger.processInputs("Wrist/Encoder", encoderInputs);

        motorAlert.set(!inputs.motorsConnected[0]);
        encoderAlert.set(!encoderInputs.connected);

        if (!reset && encoderInputs.connected) {
            //get rotations from encoder, then convert to radians, and use gear ratio
            double absolutePosition = encoderInputs.angle.getRotations() *2.0 * Math.PI * WristConstants.GEAR_RATIO;
            motor.setPosition(absolutePosition);
            reset = true;
        }

        ExecutionLogger.log("Wrist");
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.positionRad / WristConstants.GEAR_RATIO);
    }

    public Command goToAngle(Rotation2d targetAngle) {
        return run(() -> motor.runPosition(targetAngle.getRadians() * WristConstants.GEAR_RATIO))
        .until(() -> Math.abs(targetAngle.getRadians() - getAngle().getRadians()) < 0.01)
        .withName("Wrist Go To Angle");
    }

    public Command holdAngle(Rotation2d targetAngle) {
        return run(() -> motor.runPosition(targetAngle.getRadians() * WristConstants.GEAR_RATIO))
            .withName("Wrist Hold Angle");
    }

    public Command stop() {
        return runOnce(() -> motor.runVoltage(0.0)).withName("Wrist Stop");
    }
    
}
