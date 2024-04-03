// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAIcontroller = 1;
  }

  public static class SwerveDriveConstants {
    // Max Speeds For Swerve Drive
    public static final double MAX_TELE_DRIVE_SPEED = (TunerConstants.kSpeedAt12VoltsMps * 0.7);
    public static final double MAX_ALL_DRIVE_SPEED = (TunerConstants.kSpeedAt12VoltsMps * 0.7);
    public static final double MAX_ANGULAR_RATE = (2 * Math.PI);

    // Front Left Module
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;

    // Front Right Module
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightSteerMotorId = 4;

    // Back Left Module
    public static final int kBackLeftDriveMotorId = 7;
    public static final int kBackLeftSteerMotorId = 8;

    // Back Right Module
    public static final int kBackRightDriveMotorId = 5;
    public static final int kBackRightSteerMotorId = 6;

    // Swerve Gyro
    public static final int SwervePigeon_ID = 1;

  }

  public static class HarvesterConstants {
    public static final int OuterBar_ID = 1;
    public static final int InnerBar_ID = 2;
  }

  public static class IndexerConstants {
    public static final int Indexer_ID = 3;
    public static final int LeftBarrelLimitSwitch_PWM_ID = 0;
    public static final int RightBarrelLimitSwitch_PWM_ID = 1;
    public static final int LaserNoteDetector_ID = 0;
  }

  public static class ActuatorConstants {
    public static final int Actuator_ID = 9;
    public static final int ActuatorGyro_Pigeon_ID = 2;
    // PID Constants for Actuator
    public static final double Actuator_Kp = 0.1;
    public static final double Actuator_Ki = 0.03;
    public static final double Actuator_Kd = 0.03;
    // Max Output Speeds of Actuator
    public static final double Max_actuation_up = 0.4;
    public static final double Max_actuation_down = -0.4;
    // Potential Motion Magic Constants for Actuator
    public static final double MagicAcceleration = 20;
    public static final double MagicCruiseVelocity = 40;
    // Min and Max Angles on Actuator for scoring
    public static final double minAngle = 30;
    public static final double maxAngle = 60;
    // Pre-Determined Shot Angles
    public static final double automTapeShotAngle = 35;
    public static final double defaultBellyUpAngle = 53;
  }

  public static class ShooterConstants {
    public static final int Shooter_Motor_ID = 10;

    // Max Output Speeds for Shooter Wheels
    public static final double DefaultShotPercent = 1;
    // RPM Control Loop Constants for Shooter Wheels
    public static final double ShooterVelocity_Kp = 0;
    public static final double ShooterVelocity_Ki = 0.2;
    public static final double ShooterVelocity_Kd = 0;
    public static final double ShooterVelocity_Ks = 0.5;
    public static final double ShooterVelocity_Kv = 0.1;
  }

  public static class ClimberConstants {
    public static final int Climber_ID   = 11;
    // Motion Magic Constants for Climber
    public static final double MagicAcceleration = 60;
    public static final double MagicCruiseVelocity = 60;
    // PID Loop Constants for Climber Wheels
    public static final double Climber_Kp = 0.1;
    public static final double Climber_Ki = 0;
    public static final double Climber_Kd = 0;
    public static final double Climber_Ks = 0;
    public static final double Climber_Kv = 0;
    public static final double Climber_Ka = 0;
  }

  public static class CameraConstants
  {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static class ShooterCamera
    {
      public static final String name = "ShooterCamera";
      // Harvester is the Front of Robot, Positive X = Closer to Front, Positive Y = Farther to the Left, Positive Z = Up.
      public static final double yDistanceLeftFromCenter = Units.inchesToMeters(-0.25);
      public static final double xDistanceForwardFromCenter = Units.inchesToMeters(-15.5);
      public static final double zDistanceUpFromFloor = Units.inchesToMeters(13.1875);

      public static final double xRoll = Units.degreesToRadians(0);
      public static final double yPitchUp = Units.degreesToRadians(-16.38);
      public static final double zYaw = Units.degreesToRadians(180);
      }
    public static class HarvesterCamera
    {
      public static final String name = "HarvesterCamera";
      // Harvester is the Front of Robot, Positive X = Closer to Front, Positive Y = Farther to the Left, Positive Z = Up.
      public static final double yDistanceLeftFromCenter = Units.inchesToMeters(-11.125);
      public static final double xDistanceForwardFromCenter = Units.inchesToMeters(13.9375);
      public static final double zDistanceUpFromFloor = Units.inchesToMeters(12.6875);

      public static final double xRoll = Units.degreesToRadians(180);
      public static final double yPitchUp = Units.degreesToRadians(0);
      public static final double zYaw = Units.degreesToRadians(8); // 8 Degrees inwards
    }
  }
}