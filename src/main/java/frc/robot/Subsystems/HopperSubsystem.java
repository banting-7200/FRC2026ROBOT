package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.IntakeConstants;
import frc.robot.Utilites.Tunable.TunableSparkFlexPid;

public class HopperSubsystem extends SubsystemBase {

  SparkFlex motor;
  SparkFlexConfig config;
  boolean isOn = false;
  DoubleEntry hopperRPM;
  SparkClosedLoopController hopperController;

  public HopperSubsystem() {
    motor = new SparkFlex(CANIds.HOPPER_ID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    hopperController = motor.getClosedLoopController();
    config.voltageCompensation(12.5);
    config.inverted(IntakeConstants.Pivot.INVERSION).idleMode(IdleMode.kCoast);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pid(0.0001, 0, 0).feedForward.kV(0.003);
    config.smartCurrentLimit(80);
    motor.configure(
        config,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    hopperRPM = TunableSparkFlexPid.create("HopperRPM", motor, config, 0);
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
    // if (isOn) motor.set(0.2);
    hopperController.setSetpoint(hopperRPM.get(), ControlType.kVelocity);
    // System.out.println(motor.getEncoder().getVelocity());
    System.out.println(motor.getOutputCurrent() + " | " + motor.getEncoder().getVelocity());
  }
}
