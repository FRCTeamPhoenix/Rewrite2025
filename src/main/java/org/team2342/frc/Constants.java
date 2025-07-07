// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team2342.lib.motors.MotorConfig;
import org.team2342.lib.motors.MotorConfig.IdleMode;
import org.team2342.lib.motors.smart.SmartMotorConfig;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIO.AbsoluteEncoderConfig;

public final class Constants {
  public static final Mode CURRENT_MODE = Mode.REAL;
  public static final boolean TUNING = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static final String LOW_BACK_CAMERA_NAME = "back_arducam";
    public static final String LEFT_CAMERA_NAME = "left_arducam";
    public static final String RIGHT_CAMERA_NAME = "right_arducam";

    public static final String LIMELIGHT_NAME = "limelight";

    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(-35)));

    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(-8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(35)));

    public static final Transform3d LOW_BACK_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.73),
                Units.inchesToMeters(11.286),
                Units.inchesToMeters(7.89)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            680.3518437477294,
            0.0,
            393.34429560711095,
            0.0,
            681.5148816063638,
            304.5111454902841,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.04913279370181987,
            -0.08080811604393605,
            0.0012713783068216294,
            -9.086414571538155E-4,
            0.03813939624862079,
            -0.002083234186226857,
            0.003667258530403619,
            -0.0014957440403602612);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.1;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.06; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0 // Camera 2
        };

    // Multipliers to apply for MegaTag2/ConstrainedPNP observations
    public static final double LINEAR_STD_DEV_CONSTRAINED_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_CONSTRAINED_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class DriveConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double ROTATION_LOCK_TIME = 0.25;

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double MAX_LINEAR_ACCELERATION = 20.0;
    public static final double DRIVE_GEARING = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEARING = 150.0 / 7.0;
    public static final double COUPLE_RATIO = 27.0 / 17.0 / 3;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double WHEEL_COF = 1.2;

    public static final double TRACK_WIDTH_X = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(112);
    public static final double ROBOT_MOI = 5.278;

    public static final double TURN_CURRENT_LIMIT = 30.0;
    public static final double SLIP_CURRENT_LIMIT = 70.0;
    public static final double DRIVE_SUPPLY_LIMIT = 40.0;
    public static final double MAX_MODULE_VELOCITY_RAD = Units.degreesToRadians(1080.0);

    public static final double[] COMP_ENCODER_OFFSETS = {
      -0.1467, -0.3962, 0.3662 + 0.5, -0.3867 + 0.5
    };

    public static final double[] ENCODER_OFFSETS = COMP_ENCODER_OFFSETS;

    // Pitch, Roll, Yaw
    public static final double[] PIGEON_CALIBRATED_MOUNT_POSE = {0, 0, 0};

    public static final boolean IS_CANFD = false;
    public static final double ODOMETRY_FREQUENCY = IS_CANFD ? 250.0 : 100.0;
  }

  public static final class ClawConstants {
    public static final double SENSOR_THRESHOLD = 0.02;

    public static final MotorConfig CLAW_CONFIG =
        new MotorConfig()
            .withIdleMode(IdleMode.BRAKE)
            .withSupplyCurrentLimit(30)
            .withMotorInverted(true);

    public static final DCMotor CLAW_SIM_MOTOR = DCMotor.getKrakenX60(1);
    public static final DCMotorSim CLAW_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(CLAW_SIM_MOTOR, 0.003, 5), CLAW_SIM_MOTOR);
  }

  public static final class ClimberConstants {
    public static final double GEAR_RATIO = 144;
    public static final double ARM_LENGTH = Units.inchesToMeters(16.6);
    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
    public static final double MIN_ANGLE = -Math.PI;
    public static final double MAX_ANGLE = Math.PI;

    public static final MotorConfig CLIMBER_CONFIG =
        new MotorConfig()
            .withIdleMode(IdleMode.BRAKE)
            .withSupplyCurrentLimit(40)
            .withMotorInverted(false);

    public static final DCMotor CLIMBER_SIM_MOTOR = DCMotor.getKrakenX60(1);

    public static final SingleJointedArmSim CLIMBER_SIM =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(CLIMBER_SIM_MOTOR, 0.04, GEAR_RATIO),
            CLIMBER_SIM_MOTOR,
            GEAR_RATIO,
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            true,
            MAX_ANGLE);
  }

  public static final class WristConstants {
    public static final double GEAR_RATIO = 45;
    public static final double CLAW_LENGTH = Units.inchesToMeters(16.6);
    public static final double MIN_ANGLE = -0.93;
    public static final double MAX_ANGLE = 1.43;
    public static final double CUTOFF_ANGLE = 1.2;
    public static final double MOVE_ANGLE = 0.5;
    public static final double AT_TARGET_TOLERANCE = 0.01;

    public static final SmartMotorConfig WRIST_CONFIG =
        new SmartMotorConfig()
            .withGearRatio(GEAR_RATIO)
            .withProfileConstraintsRad(new TrapezoidProfile.Constraints(3.5, 3))
            .withIdleMode(IdleMode.BRAKE)
            .withControlType(ControlType.PROFILED_POSITION)
            .withSupplyCurrentLimit(40);
    public static final AbsoluteEncoderConfig ENCODER_CONFIG = new AbsoluteEncoderConfig(0, true);
  }

  public static final class CANConstants {
    public static final int PDH_ID = 14;

    public static final int PIGEON_ID = 13;
    public static final int[] FL_IDS = {1, 5, 9};
    public static final int[] FR_IDS = {2, 6, 10};
    public static final int[] BL_IDS = {3, 7, 11};
    public static final int[] BR_IDS = {4, 8, 12};

    public static final int WRIST_ID = 17;
    public static final int WRIST_ENCODER_ID = 18;
    public static final int CLAW_ID = 19;
    public static final int CLAW_LASERCAN_ID = 20;
    public static final int CLIMBER_ID = 21;
  }
}
