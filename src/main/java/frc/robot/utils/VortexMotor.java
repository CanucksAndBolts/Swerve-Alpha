// define how the motors work
package frc.robot.utils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VortexMotor {  
  private final SparkFlex flex;
  // configurator??? for sparkflex copying peddies code 
  private final int deviceID;
  private final String canbusName;
  private SparkFlexConfig Config;
  
  private double feetForawrd = 0.0;
  private double velocityConversionFactor = 1.0;
  
  
   public VortexMotor(int deviceID, String canbusName2) {
          this.flex = new SparkFlex(deviceID, canbusName2);
          factoryReset();
          this.deviceID = deviceID;
          this.canbusName = canbusName2;
        Config = new SparkFlexConfig();
        flex.getConfigurator().setPosition(0);
    
  } 
  public void factoryReset() {
    flex.getConfigurator().apply(new SparkFlexConfig());
}

}