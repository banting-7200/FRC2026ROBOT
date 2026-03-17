package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.IntakeConstants;
import frc.robot.Utilites.Tunable.TunableSparkMaxPid;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    INTAKING_FUEL,
    AGITATING_FUEL,
    STORED,
    NORMAL;
  };

  SparkMax pivotMotor;
  SparkMax intakeMotor;
  SparkClosedLoopController pivotController;
  SparkClosedLoopController intakeController;
  SparkMaxConfig pivotConfig;
  SparkMaxConfig intakeConfig;
  IdleMode idleMode;
  double setpoint;
  boolean isAgitating = false;
  boolean isUp = false;
  boolean isSpinning = false;
  double intakeVel = 0;
  DoubleEntry pivotAngle;
  DoubleEntry intakeRPM;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(CANIds.INTAKE_ID, MotorType.kBrushless);
    intakeController = intakeMotor.getClosedLoopController();
    intakeConfig = new SparkMaxConfig();

    intakeConfig
        .closedLoop
        .pid(IntakeConstants.Intake.P, IntakeConstants.Intake.I, IntakeConstants.Intake.D)
        .feedForward
        .kV(IntakeConstants.Intake.V);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakeConfig.voltageCompensation(12.5);
    intakeConfig.smartCurrentLimit(IntakeConstants.Intake.CURRENT_LIMIT);
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeMotor.configure(
        intakeConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    intakeRPM = TunableSparkMaxPid.create("IntakeRPM", intakeMotor, intakeConfig, 0);

    pivotMotor = new SparkMax(CANIds.INTAKE_PIVOT_ID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotController = pivotMotor.getClosedLoopController();
    idleMode = IdleMode.kBrake;

    pivotConfig.inverted(IntakeConstants.Pivot.INVERSION).idleMode(IdleMode.kCoast);
    pivotConfig.smartCurrentLimit(IntakeConstants.Pivot.CURRENT_LIMIT);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotConfig.absoluteEncoder.inverted(true).zeroOffset(0.7);
    pivotConfig.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
    pivotConfig.closedLoop.pid(
        IntakeConstants.Pivot.P, IntakeConstants.Pivot.I, IntakeConstants.Pivot.D);

    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    pivotAngle = TunableSparkMaxPid.create("Pivot Angle", pivotMotor, pivotConfig, 160);
  }

  public void setIdleMode(IdleMode idleMode) {
    this.idleMode = idleMode;
    pivotConfig.idleMode(idleMode);
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  public void setState(IntakeState state) {
    switch (state) {
      case INTAKING_FUEL:
        isAgitating = false;
        setpoint = IntakeConstants.INTAKE_POSITION;
        break;
      case AGITATING_FUEL:
        isAgitating = true;
        break;
      case STORED:
        isAgitating = false;
        setpoint = IntakeConstants.STORED_POSITION;
        break;
      case NORMAL:
        isAgitating = false;
        setpoint = IntakeConstants.NORMAL_POSITION;
        break;
    }
  }

  public void togglePosition() {
    isUp = !isUp;
    if (isUp) setState(IntakeState.INTAKING_FUEL);
    if (!isUp) setState(IntakeState.NORMAL);
  }

  public void run() {
    // pivotController.setSetpoint(pivotAngle.get(), ControlType.kPosition);
    // pivotMotor.set(0.2);
    intakeController.setSetpoint(intakeRPM.get(), ControlType.kVelocity);
    // System.out.println(pivotMotor.getAbsoluteEncoder().getPosition());
  }

  public void spinUp() {
    isSpinning = !isSpinning;
    if (isSpinning) intakeVel = 2500;
    else intakeVel = 0;
  }
}
