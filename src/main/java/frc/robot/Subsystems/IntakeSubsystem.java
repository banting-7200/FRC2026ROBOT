package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    INTAKING_FUEL,
    AGITATING_FUEL,
    STORED,
    NORMAL;
  };

  SparkMax pivotMotor;
  SparkClosedLoopController pivotController;
  SparkMaxConfig pivotConfig;

  TalonFX intakeMotor;
  VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(0);

  IdleMode idleMode;
  double setpoint;
  boolean isAgitating = false;
  boolean isUp = false;
  boolean isSpinning = false;
  double intakeVel = 0;
  double agitatingSpeedFactor = 7;
  DoubleEntry intakeRPM;
  double pivotAngle = IntakeConstants.NORMAL_POSITION;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(CANIds.INTAKE_ID);
    TalonFXConfiguration intakeConfigFX = new TalonFXConfiguration();

    intakeConfigFX.Slot0.kP = IntakeConstants.Intake.P;
    intakeConfigFX.Slot0.kI = IntakeConstants.Intake.I;
    intakeConfigFX.Slot0.kD = IntakeConstants.Intake.D;
    intakeConfigFX.Slot0.kV = IntakeConstants.Intake.V;

    intakeConfigFX.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Intake.CURRENT_LIMIT;
    intakeConfigFX.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfigFX.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfigFX.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(intakeConfigFX);

    pivotMotor = new SparkMax(CANIds.INTAKE_PIVOT_ID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotController = pivotMotor.getClosedLoopController();

    pivotConfig.inverted(IntakeConstants.Pivot.INVERSION).idleMode(IdleMode.kCoast);
    pivotConfig.smartCurrentLimit(IntakeConstants.Pivot.CURRENT_LIMIT);

    pivotConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60.0);

    //  pivotConfig.absoluteEncoder.zeroOffset(0.3);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotConfig.closedLoop.positionWrappingEnabled(false);

    pivotConfig.closedLoop.pid(
        IntakeConstants.Pivot.P, IntakeConstants.Pivot.I, IntakeConstants.Pivot.D);
    pivotConfig
        .closedLoop
        .feedForward
        .kCos(IntakeConstants.Pivot.G) // Voltage to hold at 0 degrees (horizontal)
        .kCosRatio(Math.PI / 180.0);

    pivotConfig
        .softLimit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(IntakeConstants.INTAKE_POSITION)
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(IntakeConstants.STORED_POSITION);

    // pivotConfig.voltageCompensation(12.0);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO CHECK THE ENCODER VALUES NEAR 0!!!!! BEFORE RUNNINGNGGGGGGGHGGGGGG
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
        intakeVel = 5000; // Updated to use local variable
        break;
      case AGITATING_FUEL:
        isAgitating = true;
        intakeVel = 1000;
        break;
      case STORED:
        isAgitating = false;
        setpoint = IntakeConstants.STORED_POSITION;
        intakeVel = 0;
        break;
      case NORMAL:
        isAgitating = false;
        pivotAngle = IntakeConstants.NORMAL_POSITION;
        intakeVel = 0;
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
    // pivotMotor.setVoltage(0.2);
    intakeMotor.setControl(intakeVelocityControl.withVelocity(intakeVel / 60.0));
    // System.out.println(intakeMotor.getVelocity());
  }

  public void setPivot(double speed) {
    pivotMotor.set(speed);
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

  public double getPosition() {
    return pivotMotor.getAbsoluteEncoder().getPosition();
  }
}
