/** define how motor work 
TODO: go back to code and properly comment things
get help from miss p to make this better or fix mistakes
yippie 
*/
package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VortexMotor {
   // -----------------------
   // Fields / Constants
   // -----------------------
   private final SparkFlex spark;
   private final RelativeEncoder encoder;
   private final SparkClosedLoopController controller;
   private SparkFlexConfig config;
   private final int deviceID;
   private final String busID;

   // Cached PID and feedforward gains
   private double kP = 0.0;
   private double kI = 0.0;
   private double kD = 0.0;
   private double kF = 0.0; // Constant feedforward (volts)

   private double kS = 0.0;
   private double kV = 0.0;
   private double kA = 0.0;

   private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

   private double velocityConversionFactor = 1.0;
   private double positionConversionFactor = 1.0; // Default 1:1 ratio for encoder to mechanism units
   
      // -----------------------
      // Constructor
      // -----------------------
      public VortexMotor(int deviceID, String busID, SparkLowLevel.MotorType mType) {
          this.deviceID = deviceID;
          this.busID = busID;
   
          spark = new SparkFlex(deviceID, mType);
          encoder = spark.getEncoder();
          encoder.setPosition(0.0);
   
          config = new SparkFlexConfig();
          controller = spark.getClosedLoopController();
   
          resetAndApplyConfig();
      }
   
      // -----------------------
      // Configuration / Reset
      // -----------------------
      public void resetAndApplyConfig() {
          // Factory-safe reset
          SparkFlexConfig defaults = new SparkFlexConfig();
          REVLibError resetErr = spark.configure(defaults, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
          if (resetErr != REVLibError.kOk) {
              System.err.println("Factory reset failed: " + resetErr);
              return;
          }
   
          // Apply current config (persist parameters)
          REVLibError configErr = spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
          if (configErr != REVLibError.kOk) {
              System.err.println("Config apply failed: " + configErr);
          }
      }
   
      // -----------------------
      // Basic Helpers / State
      // -----------------------
      public void setEncoder(double position) {
          encoder.setPosition(position);
      }
   
      public void resetEncoder() {
          RelativeEncoder enc = spark.getEncoder();
          if (enc != null) {
              enc.setPosition(0.0);
          }
      }
   
      public double getPosition() {
          return encoder.getPosition(); // rotations by default
      }
   
      public double getPercentOutput() {
          return spark.get(); // duty cycle [-1..1]
      }
   
      public double getMPS() {
         double rpm = encoder.getVelocity(); // RPM native
         return rpm * velocityConversionFactor; // meters per second
      }
   
      public double getRPS() {
          return encoder.getVelocity() / 60.0; // rotations per second
      }
   
      public double getRPM() {
          return encoder.getVelocity(); // RPM
      }
   
      public double getSupplyCurrent() {
          return spark.getOutputCurrent(); // amps
      }
   
      public double getMotorTemperature() {
          return spark.getMotorTemperature();
      }
   
      // -----------------------
      // Motor Mode / Output Control
      // -----------------------
      public void setPercentOutput(double percent) {
          percent = Math.max(-1.0, Math.min(1.0, percent));
          spark.set(percent);
      }
   
      public void setNeutralControl() {
          spark.stopMotor();
      }
   
      public void setBrake() {
          config.idleMode(IdleMode.kBrake);
          resetAndApplyConfig();
      }
   
      public void setCoast() {
          config.idleMode(IdleMode.kCoast);
          resetAndApplyConfig();
      }
   
      public void setInverted(boolean inverted) {
          config.inverted(inverted);
          resetAndApplyConfig();
      }
   
      public void setCurrentLimit(int currentLimit) {
          config.smartCurrentLimit(currentLimit);
          resetAndApplyConfig();
      }
   
      public void setClosedLoopRampRate(double rampRateSeconds) {
          config.closedLoopRampRate(rampRateSeconds);
          resetAndApplyConfig();
      }
   
      // -----------------------
      // Conversion Ratios / Scaling
      // -----------------------
      public void setVelocityConversionFactor(double conversion) {
          this.velocityConversionFactor = conversion;
      }
   
      public void setRotorToSensorRatio(double ratio) {
         this.positionConversionFactor = ratio;
      this.velocityConversionFactor = ratio;
  
      config.encoder.positionConversionFactor(ratio);
      config.encoder.velocityConversionFactor(ratio);
      resetAndApplyConfig();
  }
  
  public void setSensorToMechanismRatio(double ratio) {
      this.positionConversionFactor *= ratio;
      this.velocityConversionFactor *= ratio;
  
      config.encoder.positionConversionFactor(positionConversionFactor);
      config.encoder.velocityConversionFactor(velocityConversionFactor);
      resetAndApplyConfig();
  }

   // -----------------------
   // PID / Feedforward Configuration
   // -----------------------
   /**
    * Set PID values (kP, kI, kD) and constant feedforward kF.
    */
   public void setPIDValues(double kP, double kI, double kD, double kF) {
       this.kP = kP;
       this.kI = kI;
       this.kD = kD;
       this.kF = kF;

       config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
       resetAndApplyConfig();
   }

   /**
    * Set full feedforward model (kS, kV, kA) and PID gains.
    */
   public void setPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF) {
       this.kS = kS;
       this.kV = kV;
       this.kA = kA;
       this.kP = kP;
       this.kI = kI;
       this.kD = kD;
       this.kF = kF;

       config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
       ff = new SimpleMotorFeedforward(kS, kV, kA);
       resetAndApplyConfig();
   }

   public void setFeedforwardGains(double kS, double kV, double kA) {
       this.kS = kS;
       this.kV = kV;
       this.kA = kA;
       ff = new SimpleMotorFeedforward(kS, kV, kA);
   }

   // PID getters
   public double getKS() { return kS; }
   public double getKV() { return kV; }
   public double getKA() { return kA; }
   public double getKP() { return kP; }
   public double getKI() { return kI; }
   public double getKD() { return kD; }

   // -----------------------
   // Motion Control Commands
   // -----------------------
   /**
    * Velocity closed-loop setpoint with feedforward.
    * @param mechanismVelocity velocity in mechanism units
    * @param mechanismAccel acceleration in mechanism units
    */
   public void setVelocityClosedLoopWithFF(double mechanismVelocity, double mechanismAccel) {
       double arbFFVolts = ff.calculate(mechanismVelocity, mechanismAccel);
       double deviceVelocity = mechanismVelocity / velocityConversionFactor;

       REVLibError err = controller.setReference(
           deviceVelocity,
           SparkBase.ControlType.kVelocity,
           ClosedLoopSlot.kSlot0,
           arbFFVolts
       );

       if (err != REVLibError.kOk) {
           System.err.println("setVelocityClosedLoopWithFF failed: " + err);
       }
   }

   /**
    * Position closed-loop control with arbitrary feedforward voltage.
    */
   public void setPositionClosedLoop(double position, double arbFFVolts) {
       REVLibError err = controller.setReference(
           position,
           SparkBase.ControlType.kPosition,
           ClosedLoopSlot.kSlot0,
           arbFFVolts
       );

       if (err != REVLibError.kOk) {
           System.err.println("setPositionClosedLoop failed: " + err);
       }
   }

   public void setPositionClosedLoopWithKf(double position) {
       REVLibError err = controller.setReference(
           position,
           SparkBase.ControlType.kPosition,
           ClosedLoopSlot.kSlot0,
           kF
       );

       if (err != REVLibError.kOk) {
           System.err.println("setPositionClosedLoopWithKf failed: " + err);
       }
   }

   // -----------------------
   // Motion Profile / SmartMotion / MAXMotion Helpers
   // -----------------------
   /**
    * Configure MAXMotion parameters (cruise velocity and max acceleration).
    */
   public void setMaxMotionParameters(double cruiseVelocityRPM, double maxAccelerationRPMperSec) {
       config.closedLoop.maxMotion.maxVelocity(cruiseVelocityRPM, ClosedLoopSlot.kSlot0);
       config.closedLoop.maxMotion.maxAcceleration(maxAccelerationRPMperSec, ClosedLoopSlot.kSlot0);
       resetAndApplyConfig();
   }

   public void setSmartMotionParameters(double cruiseVelocityRPM, double maxAccelerationRPMperSec) {
       setMaxMotionParameters(cruiseVelocityRPM, maxAccelerationRPMperSec);
   }

   /**
    * Move to target position using MAXMotion position control.
    */
   public void moveToPosition(double rotations) {
       REVLibError err = controller.setReference(
           rotations,
           ControlType.kMAXMotionPositionControl,
           ClosedLoopSlot.kSlot0
       );

       if (err != REVLibError.kOk) {
           System.err.println("moveToPosition failed: " + err);
       }
   }

   // -----------------------
   // Soft Limits / Wrapping / Follower
   // -----------------------
   public void setSoftLimits(boolean enable, double forwardLimitValue, double reverseLimitValue) {
       if (enable) {
           config.softLimit.forwardSoftLimit(forwardLimitValue).forwardSoftLimitEnabled(true);
           config.softLimit.reverseSoftLimit(reverseLimitValue).reverseSoftLimitEnabled(true);
       } else {
           config.softLimit.forwardSoftLimitEnabled(false);
           config.softLimit.reverseSoftLimitEnabled(false);
       }
       resetAndApplyConfig();
   }

   public void setContinuousOutput(boolean enabled, double minInput, double maxInput) {
       config.closedLoop.positionWrappingEnabled(enabled);
       if (enabled) {
           config.closedLoop.positionWrappingInputRange(minInput, maxInput);
       }
       resetAndApplyConfig();
   }

   public void setFollower(int leaderCanId, boolean invert) {
       config.follow(leaderCanId, invert);
       resetAndApplyConfig();
   }
   /**
 * Sets velocity closed-loop setpoint with feedforward voltage.
 * @param velocityMetersPerSecond desired velocity in meters per second (mechanism units).
 */
public void setVelocityVoltageWithFeedForward(double velocityMetersPerSecond) {
   // Convert velocity from meters per second to device native units (e.g., RPM)
   double deviceVelocity = velocityMetersPerSecond / velocityConversionFactor;

   // Calculate feedforward voltage (assuming zero acceleration)
   double ffVolts = ff.calculate(velocityMetersPerSecond, 0);

   REVLibError err = controller.setReference(
       deviceVelocity,
       SparkBase.ControlType.kVelocity,
       ClosedLoopSlot.kSlot0,
       ffVolts
   );
   if (err != REVLibError.kOk) {
       System.err.println("setVelocityVoltageWithFeedForward failed: " + err);
   }
}

/**
* Sets position closed-loop setpoint with feedforward voltage.
* @param positionRotations desired position in rotations.
*/
public void setPositionVoltageWithFeedForward(double positionRotations) {
   // Feedforward voltage for position is often zero
   double ffVolts = ff.calculate(0, 0);

   REVLibError err = controller.setReference(
       positionRotations,
       SparkBase.ControlType.kPosition,
       ClosedLoopSlot.kSlot0,
       ffVolts
   );
   if (err != REVLibError.kOk) {
       System.err.println("setPositionVoltageWithFeedForward failed: " + err);
   }
}
   // -----------------------
   // Diagnostics / Dashboard
   // -----------------------
   public void updateSmartDashboard() {
       String prefix = busID + " " + deviceID + " ";

       // Motor telemetry
       SmartDashboard.putNumber(prefix + "Motor Position (rotations)", encoder.getPosition());
       SmartDashboard.putNumber(prefix + "Velocity (RPM)", getRPM());
       SmartDashboard.putNumber(prefix + "Velocity (ft/s)", getMPS());
       SmartDashboard.putNumber(prefix + "Output Percent", getPercentOutput());
       SmartDashboard.putNumber(prefix + "Supply Current (A)", getSupplyCurrent());
       SmartDashboard.putNumber(prefix + "Motor Temperature (°C)", getMotorTemperature());

       // PID gains
       SmartDashboard.putNumber(prefix + "kP", getKP());
       SmartDashboard.putNumber(prefix + "kI", getKI());
       SmartDashboard.putNumber(prefix + "kD", getKD());
       SmartDashboard.putNumber(prefix + "kF", kF);
       
       // Warnings
       final double CURRENT_THRESHOLD = 40.0; // amps
       final double TEMP_THRESHOLD = 70.0;    // Celsius

       SmartDashboard.putString(prefix + "Current Warning", getSupplyCurrent() > CURRENT_THRESHOLD ? "High Current!" : "Normal");
       SmartDashboard.putString(prefix + "Temp Warning", getMotorTemperature() > TEMP_THRESHOLD ? "High Temperature!" : "Normal");
   }
}//🤑🤑🤑
