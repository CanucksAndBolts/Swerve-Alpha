// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.Destination;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VortexMotor;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

	private final CANcoder steerEncoder;
  
	private final int drivingCANId;
	private final int steeringCANId;
	private final int CANCoderId;
  
	private final VortexMotor driveMotor;
	private final VortexMotor steerMotor;
  
	private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));
  
	private double moduleAngularOffset;
  
	public SwerveModule(String canbusName, int drivingCANId, int steeringCANId, int CANCoderId, double moduleAngularOffset) {
	  this.drivingCANId = drivingCANId;
	  this.steeringCANId = steeringCANId;
	  this.CANCoderId = CANCoderId;
  
  
	  driveMotor = new VortexMotor(drivingCANId, canbusName);
	  steerMotor = new VortexMotor(steeringCANId, canbusName);
  
	  driveMotor.setInverted(true);
	  steerMotor.setInverted(true);
  
	  driveMotor.setSupplyCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
	  steerMotor.setSupplyCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
  
	  driveMotor.setBrake();
	  steerMotor.setBrake();
  
	  driveMotor.setClosedLoopRampRate(0.1);
	  steerMotor.setClosedLoopRampRate(0.1);
  
	  driveMotor.setEncoder(0);
	  steerMotor.setEncoder(0);
  
	  // Steer Encoder Setup
	  steerEncoder = new CANcoder(CANCoderId, canbusName);
	  this.moduleAngularOffset = moduleAngularOffset;
	  configureCANcoder();
	  // steerEncoder.setPosition(0);
  
	  steerMotor.setContinuousOutput();
	  steerMotor.setFeedbackDevice(CANCoderId, FeedbackSensorSourceValue.FusedCANcoder);
  
	  driveMotor.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
	  steerMotor.setRotorToSensorRatio(ModuleConstants.kTurningMotorReduction);
	  steerMotor.setSensorToMechanismRatio(1.0);
  
	  driveMotor.setVelocityPIDValues(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA,
		  ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
	  steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD,
		  ModuleConstants.kTurningFF);
  
	  putSmartDashboard();
	}
  
	public SwerveModuleState getState() {
	  return new SwerveModuleState(driveMotor.getMPS(), new Rotation2d(getCANCoderReading()));
	}
  
	public SwerveModulePosition getPosition() {
	  return new SwerveModulePosition(
		  driveMotor.getPosition() * ModuleConstants.kDrivingEncoderPostionFactor, new Rotation2d(getCANCoderReading()));
	}
  
	public double getRotations() {
	  return driveMotor.getPosition();
	}
  
	public void setDesiredState(SwerveModuleState desiredModuleState) {
	  desiredState = desiredModuleState;
  
	  SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
		  new Rotation2d(getCANCoderReading()));
  
	  double desiredVelocity = optimizedDesiredState.speedMetersPerSecond * ModuleConstants.kDriveMotorReduction
		  / (2 * DriveConstants.kWheelRadius);
	  double desiredAngle = optimizedDesiredState.angle.getRadians() / (2 * Math.PI);
  
	  SmartDashboard.putNumber(drivingCANId + " optimized desired velocity", desiredVelocity);
	  SmartDashboard.putNumber(steeringCANId + " optimized desired position", desiredAngle * 2 * Math.PI);
	  SmartDashboard.putNumber(steeringCANId + " steering motor position", getCANCoderReading());
	  
	  driveMotor.setVelocityWithFeedForward(desiredVelocity);
	  steerMotor.setPositionWithFeedForward(desiredAngle);
	}
  
	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
	  // This method will be called once per scheduler run during simulation
	}
}
