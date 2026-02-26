package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.IntakeConstants;

public class HopperSubsystem extends SubsystemBase {

  SparkMax motor;
  SparkMaxConfig config;
  boolean isOn = false;

  public HopperSubsystem() {
    motor = new SparkMax(CANIds.HOPPER_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.inverted(IntakeConstants.Pivot.INVERSION).idleMode(IdleMode.kCoast);
    motor.configure(
        config,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
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
    if (isOn) motor.set(1);
  }
}
