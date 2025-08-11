// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.DriveConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private Autonomous autonomous;
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private DriverOI driverOI;
    // private Lights lights;

    public RobotContainer() {
        autonomous = Autonomous.getInstance();
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();
        driverOI = DriverOI.getInstance();
        // lights = Lights.getInstance();
        SmartDashboard.putNumber("Drive: cardinal scale", DriveConstants.kCardinalDirectionSpeedScale);

        SmartDashboard.putNumber("Scoring Pose Offset", 0);

        SmartDashboard.putBoolean("Use Milstein Poles?", false);
    }

    public Command getAutonomousCommand() {
        return Autonomous.getAutonomousCommand();
    }
}
