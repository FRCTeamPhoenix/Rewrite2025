// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team2342.frc.Constants.CANConstants;
import org.team2342.frc.Constants.ClawConstants;
import org.team2342.frc.Constants.ClimberConstants;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.Constants.ElevatorConstants;
import org.team2342.frc.Constants.VisionConstants;
import org.team2342.frc.commands.DriveCommands;
import org.team2342.frc.commands.DriveToPose;
import org.team2342.frc.commands.RotationLockedDrive;
import org.team2342.frc.subsystems.claw.Claw;
import org.team2342.frc.subsystems.climber.Climber;
import org.team2342.frc.subsystems.drive.Drive;
import org.team2342.frc.subsystems.drive.GyroIO;
import org.team2342.frc.subsystems.drive.GyroIOPigeon2;
import org.team2342.frc.subsystems.drive.ModuleIO;
import org.team2342.frc.subsystems.drive.ModuleIOSim;
import org.team2342.frc.subsystems.drive.ModuleIOTalonFX;
import org.team2342.frc.subsystems.superstructure.elevator.Elevator;
import org.team2342.frc.subsystems.vision.Vision;
import org.team2342.frc.subsystems.vision.VisionIO;
import org.team2342.frc.subsystems.vision.VisionIOConstrainedSim;
import org.team2342.frc.subsystems.vision.VisionIOPhoton;
import org.team2342.frc.subsystems.vision.VisionIOSim;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOSim;
import org.team2342.lib.motors.dumb.DumbMotorIOTalonFX;
import org.team2342.lib.motors.smart.SmartMotorConfig.FollowerConfig;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOTalonFX;
import org.team2342.lib.pidff.PIDFFConfigs;
import org.team2342.lib.pidff.PIDFFConfigs.GravityType;
import org.team2342.lib.sensors.distance.DistanceSensorIO;
import org.team2342.lib.sensors.distance.DistanceSensorIOLaserCAN;
import org.team2342.lib.sensors.distance.DistanceSensorIOSim;

public class RobotContainer {
  @Getter private final Drive drive;
  @Getter private final Vision vision;
  @Getter private final Claw claw;
  @Getter private final Climber climber;
  @Getter private final Elevator elevator;

  private final LoggedDashboardChooser<Command> autoChooser;

  @Getter private final CommandXboxController driverController = new CommandXboxController(0);
  private final Alert driverControllerAlert =
      new Alert("Driver controller is disconnected!", AlertType.kError);

  @Getter private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Alert operatorControllerAlert =
      new Alert("Operator controller is disconnected!", AlertType.kError);

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(CANConstants.PIGEON_ID),
                new ModuleIOTalonFX(CANConstants.FL_IDS, DriveConstants.COMP_ENCODER_OFFSETS[0]),
                new ModuleIOTalonFX(CANConstants.FR_IDS, DriveConstants.COMP_ENCODER_OFFSETS[1]),
                new ModuleIOTalonFX(CANConstants.BL_IDS, DriveConstants.COMP_ENCODER_OFFSETS[2]),
                new ModuleIOTalonFX(CANConstants.BR_IDS, DriveConstants.COMP_ENCODER_OFFSETS[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhoton(
                    VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM),
                new VisionIOPhoton(
                    VisionConstants.LEFT_CAMERA_NAME, VisionConstants.FRONT_LEFT_TRANSFORM));
        claw =
            new Claw(
                new DumbMotorIOTalonFX(CANConstants.CLAW_ID, ClawConstants.CLAW_CONFIG),
                new DistanceSensorIOLaserCAN(
                    CANConstants.CLAW_LASERCAN_ID,
                    RangingMode.SHORT,
                    TimingBudget.TIMING_BUDGET_33MS,
                    new RegionOfInterest(0, 0, 6, 6)));
        climber =
            new Climber(
                new DumbMotorIOTalonFX(CANConstants.CLIMBER_ID, ClimberConstants.CLIMBER_CONFIG));
        elevator =
            new Elevator(
                new SmartMotorIOTalonFX(
                    CANConstants.ELEVATOR_ID,
                    ElevatorConstants.WRIST_CONFIG.withPIDFFConfigs(
                        new PIDFFConfigs()
                            .withKP(2.5)
                            .withKI(1)
                            .withKG(0.58)
                            .withGravityType(GravityType.STATIC)),
                    new FollowerConfig[] {
                      new FollowerConfig(CANConstants.ELEVATOR_FOLLOWER_ID, false)
                    }));

        LoggedPowerDistribution.getInstance(CANConstants.PDH_ID, ModuleType.kRev);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOConstrainedSim(
                    VisionConstants.RIGHT_CAMERA_NAME,
                    VisionConstants.FRONT_RIGHT_TRANSFORM,
                    drive::getVisionGyroHeading,
                    drive::getRawOdometryPose,
                    VisionConstants.cameraMatrix,
                    VisionConstants.distCoeffs),
                new VisionIOSim(
                    VisionConstants.LEFT_CAMERA_NAME,
                    VisionConstants.FRONT_LEFT_TRANSFORM,
                    drive::getRawOdometryPose));
        claw =
            new Claw(
                new DumbMotorIOSim(ClawConstants.CLAW_SIM_MOTOR, ClawConstants.CLAW_SIM),
                new DistanceSensorIOSim("ClawSimSensor", 1));
        climber =
            new Climber(
                new DumbMotorIOSim(
                    ClimberConstants.CLIMBER_SIM_MOTOR, ClimberConstants.CLIMBER_SIM));
        elevator = new Elevator(new SmartMotorIO() {});

        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        claw = new Claw(new DumbMotorIO() {}, new DistanceSensorIO() {});
        climber = new Climber(new DumbMotorIO() {});
        elevator = new Elevator(new SmartMotorIO() {});

        break;
    }

    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.get();

    if (Constants.TUNING) setupDevelopmentRoutines();

    configureBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Named Command Test", Commands.print("Named Command Test"));
  }

  private void configureBindings() {
    // Basic drive controls
    drive.setDefaultCommand(
        new RotationLockedDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController
        .a()
        .whileTrue(
            new DriveToPose(
                drive,
                VisionConstants.TAG_LAYOUT
                    .getTagPose(7)
                    .orElse(new Pose3d())
                    .toPose2d()
                    .plus(
                        new Transform2d(
                            DriveConstants.DRIVE_BASE_RADIUS + 0.45, 0, Rotation2d.k180deg)),
                drive::getPose,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX()));
    operatorController.povLeft().whileTrue(climber.out()).onFalse(climber.stop());
    operatorController.povRight().whileTrue(climber.in()).onFalse(climber.stop());

    driverController.y().whileTrue(claw.intakeUntilCoral()).onFalse(claw.stop());
    driverController.x().whileTrue(claw.outtake()).onFalse(claw.stop());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void setupDevelopmentRoutines() {
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData(
        "Print Encoder Zeros",
        Commands.runOnce(() -> drive.printModuleAbsoluteAngles()).ignoringDisable(true));
    SmartDashboard.putData(
        "Set Vision Gyro Offset",
        Commands.runOnce(() -> drive.setVisionGyroOffset()).ignoringDisable(true));
    SmartDashboard.putData(
        "Toggle Constrained PhotonVision",
        Commands.runOnce(() -> vision.toggleHeadingsFree()).ignoringDisable(true));
  }

  public void updateAlerts() {
    driverControllerAlert.set(!driverController.isConnected());
    operatorControllerAlert.set(!operatorController.isConnected());
  }
}
