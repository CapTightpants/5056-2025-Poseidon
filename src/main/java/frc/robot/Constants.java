// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purPosee. All
 * constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    // Swerve Drive
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 14;
    public static final int kRearLeftDrivingCanId = 16;
    // Swerve Steer
    public static final int kFrontLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 15;
    public static final int kRearLeftTurningCanId = 17;

    // Autonomous
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    public static RobotConfig kAutonConfig = new RobotConfig(43.454, 6.883, new ModuleConfig(
        0.031,
        4.8,
        1.2,
        DCMotor.getNEO(1).withReduction(kBackLeftChassisAngularOffset),
        38, 1), new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  public static final class SystemConstants {
    // Lift
    public static final int kLeftLiftCanId = 1;
    public static final int kRightLiftCanId = 8;

    // Arms
    public static final int kHarpoonCanId = 7;
    public static final int kHarpoonReelCanId = 6;
    public static final int kCoralArmCanId = 2;
    public static final int kCoralIntakeCanId = 3;
    public static final int kAlgaeArmCanId = 5;
    public static final int kAlgaeIntakeCanId = 4;

    // Sensors
    public static final int kGyroCanId = 20;
    public static final boolean kGyroReversed = false;
    public static final int kCoralLimitDIO = 0;

    // Other CAN IDs
    public static final int kPduCanId = 30;
  }

  public static final class Setpoints {
    public static enum kLiftPosition {
      Start(0.75, 8.5, 1.2),
      Base(0.75, 9.4, 0.9),
      Stage1(2.5, 7.8, 1.2),
      Stage2(4.25, 7.8, 3.6),
      Stage3(7.0, 7.8, 1.2),
      algae2(6.0, 8.5, 3.6),
      processor(1.0, 8.5, 3.0),
      Other(null, null, null);

      public final Double LiftPose;
      public final Double CoralPoseDeg;
      public final Double AlgaePoseDeg;

      kLiftPosition(Double LiftPose, Double CoralPoseDeg, Double AlgaePoseDeg) {
        this.LiftPose = LiftPose;
        this.CoralPoseDeg = CoralPoseDeg;
        this.AlgaePoseDeg = AlgaePoseDeg;
      }
    }

    public static enum kHarpoonPosition {
      Start(0.0),
      Deploy(0.0),
      Intake(0.0),
      Other(null);

      public final Double Degrees;

      kHarpoonPosition(Double Degrees) {
        this.Degrees = Degrees;
      }
    }
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.09;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
