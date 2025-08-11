package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class AutoConstants {
        // TODO: update or real values (Default was TranslationP = ThetaP = 5.0)
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;

        public static final double kThetaP = 3.0;
        public static final double kThetaI = 0.0;
        public static final double kThetaD = 0.0;
    }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class ModuleConstants {
        public static final int kDriveMotorSupplyCurrentLimit = 60;
        public static final int kSteerMotorSupplyCurrentLimit = 40;
        // TODO: find and update these values!
        public static final double kDriveMotorStatorCurrentLimit = 0.0;
        public static final double kSteerMotorStatorCurrentLimit = 0.0;

        public static final double kWheelDiameterInches = 3.82;
        public static final double kDriveMotorReduction = 7.13;
        public static final double kDriveEncoderPositionFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
                / kDriveMotorReduction;
        public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
                / kDriveMotorReduction;

        public static final double kSteerMotorReduction = 18.75;

        public static final double kDriveS = 0.49; 
        public static final double kDriveV = 0.11;
        public static final double kDriveA = 0.0;
        public static final double kDriveP = 0.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerS = 0.0;
        public static final double kSteerV = 0.0;
        public static final double kSteerA = 0.0;
        public static final double kSteerP = 100.0; 
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 4.0; 
        public static final double kSteerFF = 0.0; // might need to be changed as i used my own method 

    }

    public static class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        public static final double kWheelBase = Units.inchesToMeters(22.75);

        public static final double kDriveRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2));

        public static final double kMaxFloorSpeed = 4;
        public static final double kMaxRotationSpeed = (3 / 2) * Math.PI;
        public static final double kMaxModuleSpeed = 4.4;

        public static final Translation2d[] kSwerveModuleLocations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]);

        public static final SwerveDriveKinematics kSkidKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]);

        // Steps to doing offsets for swerves:
        // 1. remove offsets (set all to 0) and then deploy code
        // 2. spin all modules so that bevel gears face left relative to robot (shooter
        // in front)
        // 3. read the cancoder values from dashboard, and negate those values
        // // offsets (check robotmap for ids) and put them here

        // public static final double kFrontLeftMagnetOffset = 0.238525;
        // public static final double kFrontRightMagnetOffset = 0.157471;
        // public static final double kBackLeftMagnetOffset = 0.496094;
        // public static final double kBackRightMagnetOffset = 0.130127;

        public static final double kFrontLeftMagnetOffset = 0.236084;
        public static final double kFrontRightMagnetOffset = 0.154541;
        public static final double kBackLeftMagnetOffset = 0.496338;
        public static final double kBackRightMagnetOffset = 0.129883;

        public static final double kSkidThreshold = 0.2;
        public static final double kDrivingDeadband = 0.05;

        public static final double kCardinalDirectionSpeedScale = 0.1;

    }
  }
