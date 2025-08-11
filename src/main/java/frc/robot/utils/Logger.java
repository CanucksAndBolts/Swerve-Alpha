package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveModule;
public class Logger {
    private static Logger instance;
    private Drivetrain drivetrain;
    private Superstructure superstructure;

    private DataLog log = DataLogManager.getLog();
    private StringLogEntry commandEntry, superstructureCurrentStateEntry, superstructureRequestedStateEntry;
    private BooleanLogEntry LLIntakeHasTargetEntry, clawTopSensorEntry, clawBottomSensorEntry, clawAlgaeSensorEntry;
    private DoubleLogEntry gyroAngleEntry, gyroAngleEntryBlue, driveTrainSpeedDesiredEntry, driveTrainAngleEntry, driveTrainXEntry,
            driveTrainYEntry, driveTrainXVelEntry, driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry,
            armAngleEntry, armPositionEntry, armPositionSetpointEntry, armCANcoderPositionEntry,
            armSupplyCurrentEntry, armStatorCurrentEntry, armTorqueCurrentEntry, armVelocityEntry,
            clawCoralMotorPositionEntry, clawCoralMotorVelocityEntry, clawCoralMotorSupplyCurrentEntry, clawCoralMotorStatorCurrentEntry,
            clawAlgaeMotorPositionEntry, clawAlgaeMotorVelocityEntry, clawAlgaeMotorSupplyCurrentEntry, clawAlgaeMotorStatorCurrentEntry,
            elevatorPositionEntry, elevatorVelocityEntry, elevatorPositionSetpointEntry,
            elevatorMainMotorSupplyCurrentEntry, elevatorMainMotorStatorCurrentEntry, elevatorMainMotorTorqueCurrentEntry,
            elevatorFollowerMotorSupplyCurrentEntry, elevatorFollowerMotorStatorCurrentEntry, elevatorFollowerMotorTorqueCurrentEntry,
            climberSupplyCurrentEntry, climberStatorCurrentEntry, climberTemperatureEntry,
            climberPositionEntry;
    
    private DoubleLogEntry reefAlignLateralErrorEntry, reefAlignDepthErrorEntry, reefAlignRotationErrorEntry;
    private DoubleLogEntry reefAlignLateralEntry, reefAlignDepthEntry, reefAlignRotationEntry;
    private DoubleLogEntry HPAlignLateralErrorEntry, HPAlignDepthErrorEntry, HPAlignRotationErrorEntry;
    private DoubleLogEntry HPAlignLateralEntry, HPAlignDepthEntry, HPAlignRotationEntry;
    private DoubleLogEntry processorAlignXErrorEntry, processorAlignXEntry, processorAlignStrafeEntry;
    private DoubleLogEntry processorAlignRotationErrorEntry, processorAlignRotationEntry;
    private DoubleLogEntry cageAlignTxEntry, cageAlignYEntry;
    private DoubleLogEntry cageAlignRotationErrorEntry, cageAlignRotationEntry;
    
    private DoubleArrayLogEntry moduleSpeedsDesiredEntry, modulePositionsDesiredEntry;
    private DoubleArrayLogEntry moduleSpeedsActualEntry, modulePositionsActualEntry;
    // private StructPublisher<Pose2d> fusedOdometryEntry;

    private DoubleArrayLogEntry fusedOdometryEntry;
    private DoubleArrayLogEntry[] limelightMT2Entry;
    
    private DoubleLogEntry[] limelightTyDistanceEntry, limelightPoseDistanceEntry, limelightFilteredPoseDistanceEntry,
        limelightFilteredTyDistanceEntry, limelightNumOfApriltagEntry, limelightTxEntry, limelightTyEntry, limelightTargetEntry,
        limelightLatencyEntry;

    private DoubleLogEntry L4offsetEntry;

    private Map<Integer, DoubleLogEntry> moduleDriveSupplyCurrentEntry, moduleSteerSupplyCurrentEntry;
    private Map<Integer, DoubleLogEntry> moduleDriveStatorCurrentEntry, moduleSteerStatorCurrentEntry;
    private Map<Integer, DoubleLogEntry> moduleCanCoderPositionEntry;

    // private List<StructPublisher<Pose2d>> limelightMT2Entry;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();


        // Superstructure Logs
        superstructureCurrentStateEntry = new StringLogEntry(log, "/Superstructure/Current Superstructure State");
        superstructureRequestedStateEntry = new StringLogEntry(log, "/Superstructure/Requested Superstructure State");

        // Field Logs
        fusedOdometryEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Fused Odometry");

        // Drivetrain Logs
        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        gyroAngleEntryBlue = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle Blue");

        driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        driveTrainSpeedDesiredEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Desired Speed");
        moduleSpeedsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Speeds");
        modulePositionsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Positions");

        moduleSpeedsActualEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Actual Speeds");
        modulePositionsActualEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Actual Positions");

        // Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");

        // Swerve Module Logs
        moduleDriveSupplyCurrentEntry = new HashMap<>();
        moduleSteerSupplyCurrentEntry = new HashMap<>();

        moduleDriveSupplyCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Drive Supply Current"));

        moduleSteerSupplyCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Steer Supply Current"));


        
        moduleDriveStatorCurrentEntry = new HashMap<>();
        moduleSteerStatorCurrentEntry = new HashMap<>();

        moduleDriveStatorCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Drive Stator Current"));

        moduleSteerStatorCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Steer Stator Current"));



        moduleCanCoderPositionEntry = new HashMap<>();

        moduleCanCoderPositionEntry.put(12, new DoubleLogEntry(log, "Modules/12 CANCoder Position"));
        moduleCanCoderPositionEntry.put(22, new DoubleLogEntry(log, "Modules/22 CANCoder Position"));
        moduleCanCoderPositionEntry.put(32, new DoubleLogEntry(log, "Modules/32 CANCoder Position"));
        moduleCanCoderPositionEntry.put(42, new DoubleLogEntry(log, "Modules/42 CANCoder Position"));

        L4offsetEntry = new DoubleLogEntry(log, "Superstructure/L4 Offset");
    }

    public void logEvent(String event, boolean isStart) {
        commandEntry.append(event + (isStart ? " Started" : " Ended"));
    }

    public void updateLogs() {

        // Drivetrain Logs
        updateDrivetrainLogs();

        // Superstructure Logs
        superstructureCurrentStateEntry.append(superstructure.getCurrentState().toString());
        superstructureRequestedStateEntry.append(superstructure.getRequestedState().toString());
    }
    private double[] pose2dToDoubleArray(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    private double[] pose2dToDoubleArray(Optional<Pose2d> pose) {
        if (pose.isEmpty())
            return new double[] { 0, 0, 0 };
        return new double[] {
            pose.get().getX(),
            pose.get().getY(),
            pose.get().getRotation().getRadians()
        };
    }

    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(DriverStation.isAutonomous() ? drivetrain.getHeadingForceAdjust() : drivetrain.getHeading());
        gyroAngleEntryBlue.append(DriverStation.isAutonomous() ? drivetrain.getHeadingBlueForceAdjust() : drivetrain.getHeadingBlue());

        driveTrainXAccEntry.append(drivetrain.getGyroAccX());
        driveTrainYAccEntry.append(drivetrain.getGyroAccY());
        driveTrainZAccEntry.append(drivetrain.getGyroAccZ());
        driveTrainSpeedDesiredEntry.append(drivetrain.getSpeed());

        SwerveModule[] modules = drivetrain.getSwerveModules();
        SwerveModuleState[] moduleStates = drivetrain.getSwerveModuleStates();
        double[] swerveModulePositions = { moduleStates[0].angle.getDegrees(), moduleStates[1].angle.getDegrees(),
                moduleStates[2].angle.getDegrees(), moduleStates[3].angle.getDegrees() };
        double[] swerveModuleSpeeds = { moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond };
                
        double[] swerveModulePositionsActual = { modules[0].getAngle(), modules[1].getAngle(),
                modules[2].getAngle(), modules[3].getAngle() };
        double[] swerveModuleSpeedsActual = { modules[0].getVelocity(), modules[1].getVelocity(),
                modules[2].getVelocity(), modules[3].getVelocity() };

        moduleSpeedsDesiredEntry.append(swerveModuleSpeeds);
        modulePositionsDesiredEntry.append(swerveModulePositions);
        moduleSpeedsActualEntry.append(swerveModuleSpeedsActual);
        modulePositionsActualEntry.append(swerveModulePositionsActual);

        fusedOdometryEntry.append(pose2dToDoubleArray(drivetrain.getPose()));
    }
    
    public void logAlignToReef(double lateralError, double depthError, double rotationError, double lateral, double depth, double rot) {
        reefAlignLateralErrorEntry.append(lateralError);
        reefAlignDepthErrorEntry.append(depthError);
        reefAlignRotationErrorEntry.append(rotationError);

        reefAlignLateralEntry.append(lateral);
        reefAlignDepthEntry.append(depth);
        reefAlignRotationEntry.append(rot);
    }
    public void logAlignToHP(double lateralError, double depthError, double rotationError, double lateral, double depth, double rot) {
        HPAlignLateralErrorEntry.append(lateralError);
        HPAlignDepthErrorEntry.append(depthError);
        HPAlignRotationErrorEntry.append(rotationError);
        
        HPAlignLateralEntry.append(lateral);
        HPAlignDepthEntry.append(depth);
        HPAlignRotationEntry.append(rot);
    }
    public void logAlignToProcessor(double xError, double rotationError, double strafe, double x, double rotation) {
        processorAlignXErrorEntry.append(xError);
        processorAlignXEntry.append(x);
        processorAlignStrafeEntry.append(strafe);
        processorAlignRotationErrorEntry.append(rotationError);
        processorAlignRotationEntry.append(rotation);
    }
    public void logAlignToCage(double txAverage, double y, double rotationError, double rotation) {
        cageAlignTxEntry.append(txAverage);
        cageAlignYEntry.append(y);
        cageAlignRotationErrorEntry.append(rotationError);
        cageAlignRotationEntry.append(rotation);
    }

    public void logModuleSupplyCurrents(int module, double drive, double steer) {
        if (moduleDriveSupplyCurrentEntry.containsKey(module))
            moduleDriveSupplyCurrentEntry.get(module).append(drive);
            
        if (moduleSteerSupplyCurrentEntry.containsKey(module))
            moduleSteerSupplyCurrentEntry.get(module).append(steer);
    }
    public void logModuleStatorCurrents(int module, double drive, double steer) {
        if (moduleDriveStatorCurrentEntry.containsKey(module))
            moduleDriveStatorCurrentEntry.get(module).append(drive);
            
        if (moduleSteerStatorCurrentEntry.containsKey(module))
            moduleSteerStatorCurrentEntry.get(module).append(steer);
    }
    public void logModuleCANCoderPosition(int module, double position) {
        if (moduleCanCoderPositionEntry.containsKey(module))
            moduleCanCoderPositionEntry.get(module).append(position);
    }

    public void logScoreEvent(int level, double elevator, double arm) {
        logEvent("Score L" + level + " with elevator " + elevator + ", arm " + arm, false);
    }

    public void logL4offset(double offset) {
        L4offsetEntry.append(offset);
    }
}
