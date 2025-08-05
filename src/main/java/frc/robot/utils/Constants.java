// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
// all numbers have to be checked with ms p
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public final class Constants {
    public static class GlobalConstants {
      public static final int kDriverControllerPort = 0;
      public static final boolean useLEDLights = true;
    }
  //yes we want LED
    public static class OIConstants {
      public static final boolean useDebugModeLayout = true;
      public static final double kDrivingDeadband = 0.1;
    }
  
    public static class DriveConstants {
      // TODO: Update these constants
      public static final double kTrackWidth = Units.inchesToMeters(25.75);
      public static final double kWheelBase = Units.inchesToMeters(22.75);
  
      public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;
  
      public static final Translation2d[] swerveModuleLocations = {
          new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
      };
  
      public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          swerveModuleLocations[0],
          swerveModuleLocations[1],
          swerveModuleLocations[2],
          swerveModuleLocations[3]
      );
  
      // TODO: Change this value
      public static final double kMaxFloorSpeed = 5.0; // meters per second
      public static final double kMaxAngularSpeed = Math.PI; // radians per second
  
      public static final double kWheelRadius = 2.0;
  
      // Steps to doing offsets for swerves:
      // 1. remove offsets (set all to 0) and then deploy code
      // 2. spin all modules so that bevel gears face left relative to robot (shooter
      // in front)
      // 3. read the cancoder values from dashboard, and put those values for these
      // offsets (check robotmap for ids)
      public static final double kFrontLeftModuleAngularOffset = -2.896;
      public static final double kFrontRightModuleAngularOffset = -3.073;
      public static final double kBackLeftModuleAngularOffset = -2.045;
      public static final double kBackRightModulelAngularOffset = -1.978;
  
      public static final double kHeadingCorrectionP = 0.1;
      public static final double kHeadingCorrectionTolerance = 1.0;
  
      public static final boolean kUseMegaTag = true; // Enable megatag botpose updates in teleop. Force calibrate works no matter what this is set to (in both auto and teleop).
    }
  
    public static class ModuleConstants {
      public static final double kWheelDiameterInches = 3.6;
  
      public static final double kDriveMotorReduction = 6.12;
      public static final double kDrivingEncoderPostionFactor = (Units.inchesToMeters(kWheelDiameterInches) * Math.PI)
          / kDriveMotorReduction;
      public static final double kDrivingEncoderVelocityFactor = (Units.inchesToMeters(kWheelDiameterInches) * Math.PI
          / kDriveMotorReduction);
  
      public static final double kTurningMotorReduction = 150.0 / 7.0;
      public static final double kTurningEncoderPositonFactor = kTurningMotorReduction / (2 * Math.PI);
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60;
  
      public static final double kDrivingMotorCurrentLimit = 50;
      public static final double kTurningMotorCurrentLimit = 50;
  
      public static final double kDrivingS = 0.05;
      public static final double kDrivingV = 0.13;
      public static final double kDrivingA = 0.0;
      public static final double kDrivingP = 0.11;
      public static final double kDrivingI = 0.0;
      public static final double kDrivingD = 0.0;
      public static final double kDrivingFF = 0.0;
  
      public static final double kTurningP = 36;
      public static final double kTurningI = 0.0;
      public static final double kTurningD = 0.0;
      public static final double kTurningFF = 0.0;
  
    }
  }