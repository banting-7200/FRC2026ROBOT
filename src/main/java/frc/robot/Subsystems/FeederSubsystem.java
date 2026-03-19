package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;

public class FeederSubsystem extends SubsystemBase {

  SparkFlex flywheelMotor;
  SparkFlex beltMotor;
  SparkFlexConfig flywheelConfig;
  SparkFlexConfig beltConfig;
  boolean isOn = false;
  //   DoubleEntry feederFlywheelRPM;
  //   DoubleEntry beltRPM;
  double feederFlywheelRPM = 0;
  double beltRPM = 0;
  SparkClosedLoopController flywheelController;
  SparkClosedLoopController beltController;

  public FeederSubsystem() {
    beltMotor = new SparkFlex(CANIds.FEEDER_BELT_ID, MotorType.kBrushless);
    beltController = beltMotor.getClosedLoopController();
    beltConfig = new SparkFlexConfig();
    beltConfig.inverted(false).idleMode(IdleMode.kCoast);
    beltConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    beltConfig.closedLoop.pid(0, 0, 0).feedForward.kV(0.002);
    beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // beltRPM = TunableSparkFlexPid.create("BeltRPM", beltMotor, beltConfig, 0);

    flywheelMotor = new SparkFlex(CANIds.FEEDER_FLYWHEEL_ID, MotorType.kBrushless);
    flywheelController = flywheelMotor.getClosedLoopController();
    flywheelConfig = new SparkFlexConfig();
    flywheelConfig.inverted(true).idleMode(IdleMode.kCoast);
    flywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    flywheelConfig.closedLoop.pid(0, 0, 0).feedForward.kV(0.0018);
    flywheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // feederFlywheelRPM =
    //     TunableSparkFlexPid.create("FeederFlyWheelRPM", flywheelMotor, flywheelConfig, 0);
  }

  public void set(double beltRPM, double feederFlywheelRPM) {
    this.beltRPM = beltRPM;
    this.feederFlywheelRPM = feederFlywheelRPM;
    beltController.setSetpoint(beltRPM, ControlType.kVelocity);
    flywheelController.setSetpoint(feederFlywheelRPM, ControlType.kVelocity);
  }
}
