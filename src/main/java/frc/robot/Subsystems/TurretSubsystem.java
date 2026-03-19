package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.TurretConstants;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.Tunable.TunableSparkMaxPid;

public class TurretSubsystem extends SubsystemBase {

  SparkFlex flyWheelMotor; // Vortex
  SparkMax pitchMotor; // Neo 550
  SparkMax yawMotor; // Neo 550

  SparkFlexConfig flywheelConfig;
  SparkMaxConfig pitchConfig;
  SparkMaxConfig yawConfig;

  SparkClosedLoopController flywheelController;
  SparkClosedLoopController pitchController;
  SparkClosedLoopController yawController;

  // DoubleEntry flywheelSetpoint;
  DoubleEntry turretAngle;
  DoubleEntry turretPitch;

  double flywheelRPM = 0;

  SlewRateLimiter speedFilter = new SlewRateLimiter(Double.MAX_VALUE, -1000, 0);

  // Turret moves 1.5 degrees for 1 degree of encoder motion
  double degreesPerEncoderRev = 360 * TurretConstants.Yaw.GEAR_RATIO;

  public TurretSubsystem() { // Yaw
    flyWheelMotor = new SparkFlex(Constants.CANIds.TURRET_FLYWHEEL_ID, MotorType.kBrushless);
    pitchMotor = new SparkMax(Constants.CANIds.TURRET_HOOD_ID, MotorType.kBrushless);
    yawMotor = new SparkMax(Constants.CANIds.TURRET_YAW_ID, MotorType.kBrushless);

    flywheelController = flyWheelMotor.getClosedLoopController();
    pitchController = pitchMotor.getClosedLoopController();
    yawController = yawMotor.getClosedLoopController();

    flywheelConfig = new SparkFlexConfig();
    pitchConfig = new SparkMaxConfig();
    yawConfig = new SparkMaxConfig();

    flywheelConfig.inverted(true).idleMode(IdleMode.kCoast);
    flywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    flywheelConfig
        .closedLoop
        .pid(TurretConstants.Flywheel.P, TurretConstants.Flywheel.I, TurretConstants.Flywheel.D)
        .feedForward
        .kV(TurretConstants.Flywheel.V);
    flywheelConfig.smartCurrentLimit(TurretConstants.Flywheel.CURRENT_LIMIT);
    flyWheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // flywheelSetpoint = TunableSparkFlexPid.create("Flywheel", flyWheelMotor, flywheelConfig, 0);

    pitchConfig.inverted(false).idleMode(IdleMode.kCoast);
    pitchConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pitchConfig.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
    pitchConfig.closedLoop.pid(
        TurretConstants.Hood.P, TurretConstants.Hood.I, TurretConstants.Hood.D);
    pitchConfig.smartCurrentLimit(TurretConstants.Hood.CURRENT_LIMIT);
    pitchConfig.softLimit.forwardSoftLimit(355).reverseSoftLimit(150);
    pitchConfig.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    pitchMotor.configure(
        pitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretPitch =
        TunableSparkMaxPid.create(
            "Turret Pitch", pitchMotor, pitchConfig, TurretConstants.MAX_DISTANCE_PITCH);

    yawConfig.inverted(true).idleMode(IdleMode.kCoast);
    yawConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    yawConfig
        .absoluteEncoder
        .positionConversionFactor(degreesPerEncoderRev)
        .velocityConversionFactor(6);
    yawConfig.absoluteEncoder.inverted(false);
    yawConfig.closedLoop.pid(TurretConstants.Yaw.P, TurretConstants.Yaw.I, TurretConstants.Yaw.D);
    // yawConfig.absoluteEncoder.zeroOffset(0.3404425);
    yawConfig.absoluteEncoder.zeroCentered(true);
    yawConfig.smartCurrentLimit(20);
    yawConfig
        .softLimit
        .forwardSoftLimit(TurretConstants.MAX_YAW)
        .reverseSoftLimit(TurretConstants.MIN_YAW);
    yawConfig.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    yawMotor.configure(yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretAngle = TunableSparkMaxPid.create("Turret Angle", yawMotor, yawConfig, 0);
  }

  public void run() {
    yawController.setSetpoint(turretAngle.get(), ControlType.kPosition);
    pitchController.setSetpoint(turretPitch.get(), ControlType.kPosition);
    flywheelController.setSetpoint(flywheelRPM, ControlType.kVelocity);
  }

  public void setFlywheelRPM(double flywheelRPM) {
    this.flywheelRPM = flywheelRPM;
  }

  public double getFlywheelRPM() {
    return flyWheelMotor.getEncoder().getVelocity();
  }

  public double getTurretAngle() {
    return yawMotor.getAbsoluteEncoder().getPosition();
  }

  public void setTurretAngle(double degrees) {
    turretAngle.set(degrees);
  }

  public void setTurretHubDistance(double distance) {
    // System.out.println("TurretAngle " + turretPitch.get() + " | DistanceToHub: " + distance);
    if (distance > TurretConstants.MAX_DISTANCE || distance < TurretConstants.MIN_DISTANCE) return;
    turretPitch.set(
        HelperFunctions.map(
            distance,
            TurretConstants.MIN_DISTANCE,
            TurretConstants.MAX_DISTANCE,
            TurretConstants.MAX_DISTANCE_PITCH,
            TurretConstants.MIN_DISTANCE_PITCH));
  }

  public void reset() {
    turretAngle.set(0);
    turretPitch.set(TurretConstants.MAX_DISTANCE_PITCH);
    flywheelRPM = 0;
    flyWheelMotor.stopMotor();
  }
}
