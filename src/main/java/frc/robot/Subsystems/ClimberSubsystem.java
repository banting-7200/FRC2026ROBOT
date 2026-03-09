package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.DIOPorts;

public class ClimberSubsystem extends SubsystemBase {

  enum ClimbMode {
    DISABLED,
    UP,
    DOWN
  }

  enum ActivatedLimitSwitches {
    ALL, // All Limit Switches
    TOP, // Top Limit Switch
    BOTTOM, // Bottom Limit Switch
    NONE // neither
  }

  SparkFlex motor;
  ClimbMode mode = ClimbMode.DISABLED;
  double motorSpeed = 0;
  DigitalInput topClimberLimit;
  DigitalInput bottomClimberLimit;

  public ClimberSubsystem() {
    motor = new SparkFlex(CANIds.CLIMB_ID, MotorType.kBrushless);
    topClimberLimit = new DigitalInput(DIOPorts.TOP_CLIMBER_LIMIT_SWITCH);
    bottomClimberLimit = new DigitalInput(DIOPorts.BOTTOM_CLIMBER_LIMIT_SWITCH);
  }

  public void beginDown(double speed) {
    mode = ClimbMode.DOWN;
    motorSpeed = speed;
  }

  public void beginUp(double speed) {
    mode = ClimbMode.UP;
    motorSpeed = speed;
  }

  public void end() {
    mode = ClimbMode.DISABLED;
    motorSpeed = 0;
  }

  public void run() {
    switch (mode) {
      case UP -> moveUp();
      case DOWN -> moveDown();
      case DISABLED -> disable();
    }
  }

  private ActivatedLimitSwitches limitsReached() {
    if (topClimberLimit.get() && bottomClimberLimit.get()) return ActivatedLimitSwitches.ALL;
    if (topClimberLimit.get()) return ActivatedLimitSwitches.TOP;

    if (bottomClimberLimit.get()) return ActivatedLimitSwitches.BOTTOM;

    return ActivatedLimitSwitches.NONE;
  }

  private void moveUp() {
    if (limitsReached() == ActivatedLimitSwitches.TOP
        || limitsReached() == ActivatedLimitSwitches.ALL) {
      mode = ClimbMode.DISABLED;
      motor.set(0);
      return;
    }
    motor.set(motorSpeed);
  }

  private void moveDown() {
    if (limitsReached() == ActivatedLimitSwitches.BOTTOM
        || limitsReached() == ActivatedLimitSwitches.ALL) {
      mode = ClimbMode.DISABLED;
      motor.set(0);
      return;
    }
    motor.set(-motorSpeed);
  }

  private void disable() {
    motor.set(0);
    motorSpeed = 0;
  }
}
