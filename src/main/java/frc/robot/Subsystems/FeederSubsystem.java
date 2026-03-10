package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;

public class FeederSubsystem extends SubsystemBase {

  SparkFlex flywheelMotor;
  SparkFlex beltMotor;
  SparkFlexConfig config;
  boolean isOn = false;

  public FeederSubsystem() {
    beltMotor = new SparkFlex(CANIds.FEEDER_BELT_ID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelMotor = new SparkFlex(CANIds.FEEDER_FLYWHEEL_ID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void toggleState() {
    isOn = !isOn;
  }

  public void turnOn() {
    isOn = true;
  }

  public void turnOff() {
    isOn = false;
  }

  public void run() {
    if (isOn) {
      beltMotor.set(0.3);
      flywheelMotor.set(0.3);
    }
  }
}
