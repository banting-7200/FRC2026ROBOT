package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.TurretConstants;
import frc.robot.Utilites.Tunable.TunableSparkFlexPid;
import frc.robot.Utilites.Tunable.TunableSparkMaxPid;

public class TurretSubsystem extends SubsystemBase {

  private final SparkFlex flywheelLeader;
  private final SparkFlex flywheelFollower;
  private final SparkMax hoodMotor;
  private final SparkMax yawMotor;

  private final SparkClosedLoopController flywheelController;
  private final SparkClosedLoopController hoodController;
  private final SparkClosedLoopController yawController;

  private double flywheelSetpointRPM = 0;
  private double yawSetpointDegrees = 0;
  private double hoodSetpointDegrees = 0;

  DoubleEntry turretFlywheelSetpoint;
  DoubleEntry turretYawSetpoint;
  DoubleEntry turretHoodSetpoint;

  public TurretSubsystem() {
    flywheelLeader = new SparkFlex(Constants.CANIds.TURRET_FLYWHEEL_ID, MotorType.kBrushless);
    flywheelFollower =
        new SparkFlex(Constants.CANIds.TURRET_FLYWHEEL_FOLLOWER_ID, MotorType.kBrushless);

    hoodMotor = new SparkMax(Constants.CANIds.TURRET_HOOD_ID, MotorType.kBrushless);
    yawMotor = new SparkMax(Constants.CANIds.TURRET_YAW_ID, MotorType.kBrushless);

    flywheelController = flywheelLeader.getClosedLoopController();
    hoodController = hoodMotor.getClosedLoopController();
    yawController = yawMotor.getClosedLoopController();

    configureFlywheel();
    configureHood();
    configureYaw();
  }

  private void configureFlywheel() {
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig.inverted(true).idleMode(IdleMode.kCoast);
    leaderConfig.smartCurrentLimit(TurretConstants.Flywheel.CURRENT_LIMIT);

    leaderConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(TurretConstants.Flywheel.P, TurretConstants.Flywheel.I, TurretConstants.Flywheel.D);

    leaderConfig.closedLoop.feedForward.kV(TurretConstants.Flywheel.V);

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.follow(flywheelLeader.getDeviceId(), true);
    followerConfig.idleMode(IdleMode.kCoast).inverted(false);
    followerConfig.smartCurrentLimit(TurretConstants.Flywheel.CURRENT_LIMIT);

    followerConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(TurretConstants.Flywheel.P, TurretConstants.Flywheel.I, TurretConstants.Flywheel.D);

    followerConfig.closedLoop.feedForward.kV(TurretConstants.Flywheel.V);

    flywheelLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelFollower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretFlywheelSetpoint =
        TunableSparkFlexPid.create("Turret Flywheel", flywheelLeader, leaderConfig, 0);
  }

  private void configureHood() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(TurretConstants.Hood.CURRENT_LIMIT);

    config.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);

    config
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.Hood.P, TurretConstants.Hood.I, TurretConstants.Hood.D);

    config
        .softLimit
        .forwardSoftLimit(TurretConstants.MAX_DISTANCE_PITCH)
        .reverseSoftLimit(TurretConstants.MIN_DISTANCE_PITCH)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretHoodSetpoint =
        TunableSparkMaxPid.create(
            "Turret Hood", hoodMotor, config, TurretConstants.MIN_DISTANCE_PITCH);
  }

  private void configureYaw() {
    SparkMaxConfig config = new SparkMaxConfig();
    double degreesPerEncoderRev = 360 * TurretConstants.Yaw.GEAR_RATIO;

    config.inverted(true).idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(25);

    config
        .absoluteEncoder
        .positionConversionFactor(degreesPerEncoderRev)
        .velocityConversionFactor(degreesPerEncoderRev / 60.0)
        .zeroCentered(true);

    config
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.Yaw.P, TurretConstants.Yaw.I, TurretConstants.Yaw.D);

    config
        .softLimit
        .forwardSoftLimit(TurretConstants.MAX_YAW)
        .reverseSoftLimit(TurretConstants.MIN_YAW)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    config.closedLoop.feedForward.kS(TurretConstants.Yaw.S);

    yawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretYawSetpoint = TunableSparkMaxPid.create("Turret Yaw", yawMotor, config, 0);
  }

  public void run() {
    yawController.setSetpoint(yawSetpointDegrees, ControlType.kPosition);
    hoodController.setSetpoint(hoodSetpointDegrees, ControlType.kPosition);
    flywheelController.setSetpoint(flywheelSetpointRPM, ControlType.kVelocity);
  }

  public void setFlywheelRPM(double rpm) {
    this.flywheelSetpointRPM = rpm;
  }

  public void setTurretAngle(double degrees) {
    this.yawSetpointDegrees = degrees;
  }

  public void setHoodAngle(double degrees) {
    this.hoodSetpointDegrees = degrees;
  }

  public double getFlywheelRPM() {
    return flywheelLeader.getEncoder().getVelocity();
  }

  public double getTurretAngle() {
    return yawMotor.getAbsoluteEncoder().getPosition();
  }

  public void reset() {
    yawSetpointDegrees = 0;
    hoodSetpointDegrees = TurretConstants.MIN_DISTANCE_PITCH;
    flywheelSetpointRPM = 0;
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();
  }
}
