package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.Timer;
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
  double agitatingSpeedFactor = 7;
  // DoubleEntry pivotAngle;
  DoubleEntry intakeRPM;
  double pivotAngle;

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
    // pivotConfig.softLimit.forwardSoftLimit(agitatingSpeedFactor)

    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // pivotAngle =
    //     TunableSparkMaxPid.create(
    //         "Pivot Angle", pivotMotor, pivotConfig, IntakeConstants.NORMAL_POSITION);
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
        pivotAngle = IntakeConstants.INTAKE_POSITION;
        intakeRPM.set(5000);
        break;
      case AGITATING_FUEL:
        isAgitating = true;
        intakeRPM.set(2000);
        break;
      case STORED:
        isAgitating = false;
        setpoint = IntakeConstants.STORED_POSITION;
        intakeRPM.set(0);
        break;
      case NORMAL:
        isAgitating = false;
        pivotAngle = IntakeConstants.NORMAL_POSITION;
        intakeRPM.set(0);
        break;
    }
  }

  public void togglePosition() {
    isUp = !isUp;
    if (isUp) setState(IntakeState.INTAKING_FUEL);
    if (!isUp) setState(IntakeState.NORMAL);
  }

  public void run() {
    if (isAgitating) agitate();
    pivotController.setSetpoint(pivotAngle, ControlType.kPosition);
    intakeController.setSetpoint(intakeRPM.get(), ControlType.kVelocity);
  }

  public void agitate() {
    double time = Timer.getFPGATimestamp();

    double midpoint =
        (IntakeConstants.LOW_AGITATE_POSITION + IntakeConstants.HIGH_AGITATE_POSITION) / 2.0;
    double amplitude =
        (IntakeConstants.HIGH_AGITATE_POSITION - IntakeConstants.LOW_AGITATE_POSITION) / 2.0;

    pivotAngle = midpoint + amplitude * Math.sin(time * agitatingSpeedFactor);
  }

  public void spinUp() {
    isSpinning = !isSpinning;
    if (isSpinning) intakeVel = 2500;
    else intakeVel = 0;
  }
}
