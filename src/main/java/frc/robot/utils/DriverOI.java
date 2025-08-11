package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;

public class DriverOI {
    private static DriverOI instance;
    private Superstructure superstructure;
    private PS4Controller controller;

    public enum DPadDirection {
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };


    public DriverOI() {
        controller = new PS4Controller(0);

        superstructure = Superstructure.getInstance();
        configureController();
    }

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }
        return instance;
    }


    public void configureController() {

        // READ: Press FN + X on the PS5 edge controller to activate the 2025 binding
        // profile

        controller = new PS4Controller(0);

        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        //Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        // squareButton.onTrue(new ConditionalCommand(
        //     new TwitchClimb(false),
        //     new InstantCommand(() -> {
        //         if (Arrays.asList(SuperstructureState.ALGAE_LOLLIPOP_INTAKE).contains(superstructure.getCurrentState()))
        //             superstructure.requestState(SuperstructureState.ALGAE_GROUND_INTAKE);
        //         else
        //             superstructure.requestState(SuperstructureState.ALGAE_LOLLIPOP_INTAKE);
        //     }),
        //     Superstructure.getInstance()::isClimbState
        // ));
        /* 
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        // circleButton.onTrue(new ConditionalCommand(
        //     new TwitchClimb(true),
        //     new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)),
        //     Superstructure.getInstance()::isClimbState
        // ));
        circleButton.onTrue(
            new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE))
        );

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> superstructure.sendToScore()));

        /*Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new InstantCommand(() -> {
            Optional<Pose2d> pose = LimelightFrontLeft.getInstance().getEstimatedPoseMT2();
            if (pose.isPresent())
                Drivetrain.getInstance().resetTranslation(pose.get().getTranslation());
        }));
        muteButton.onFalse(new InstantCommand(() -> {
            Optional<Pose2d> pose = LimelightFrontLeft.getInstance().getEstimatedPoseMT2();
            if (pose.isPresent())
                Drivetrain.getInstance().resetTranslation(pose.get().getTranslation());
        }));*/
        // DO NOT BIND: USED FOR ROTATION OF DRIVETRAIN
        @SuppressWarnings("unused")
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        // DO NOT BIND: USED FOR ROTATION OF DRIVETRAIN
        @SuppressWarnings("unused")
        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);
    }

    public double getForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getRightForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < DriveConstants.kDrivingDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, DriveConstants.kDrivingDeadband);

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getRotation() {
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation = (rightRotation - leftRotation) / 2.0;
        combinedRotation = Math.signum(combinedRotation) * Math.pow(combinedRotation, 2);

        return Math.abs(combinedRotation) < 0.05 ? 0 : combinedRotation * DriveConstants.kMaxRotationSpeed; // TODO:
                                                                                                           // convert to
                                                                                                           // rad/s
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public DPadDirection getDriverDPadInput() {
        switch (controller.getPOV()) {
            case 0:
                return DPadDirection.FORWARDS;
            case 90:
                return DPadDirection.RIGHT;
            case 270:
                return DPadDirection.LEFT;
            case 180:
                return DPadDirection.BACKWARDS;
            default:
                return DPadDirection.NONE;
        }
    }

    public Translation2d getCardinalDirection() {
        double cardinal = SmartDashboard.getNumber("Drive: cardinal scale", DriveConstants.kCardinalDirectionSpeedScale);
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(cardinal * DriveConstants.kMaxFloorSpeed,
                        0.0);
            case RIGHT:
                return new Translation2d(0.0,
                        -cardinal * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0,
                        cardinal * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(-cardinal * DriveConstants.kMaxFloorSpeed,
                        0.0);
            default:
                return new Translation2d(0.0, 0.0);
        }

    }

}