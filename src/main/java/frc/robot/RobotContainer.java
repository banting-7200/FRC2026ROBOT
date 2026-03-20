// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TurretCommands.FixJam;
import frc.robot.Commands.TurretCommands.ResetTurret;
import frc.robot.Commands.TurretCommands.ShootFuel;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeState;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;
import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
// Main class
public class RobotContainer {
  // Object initalizations
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  LightsSubsystem lights =
      new LightsSubsystem(
          PWMPorts.LIGHT_PORT,
          Constants.LIGHTS_AMOUNT); // Lights with the pwm port and amount of lights
  ElasticSubsystem elasticSubsystem = new ElasticSubsystem(); // The Driver dashboard
  PowerDistribution PDH =
      new PowerDistribution(
          Constants.CANIds.PDH_ID,
          ModuleType.kRev); // The Rev PowerDistribution board, with its CAN ID
  FieldLayout field = new FieldLayout(); // The layout of all the april tags
  IntakeSubsystem intake;
  HopperSubsystem hopper;
  FeederSubsystem feeder;
  TurretSubsystem turret;

  // The 3 PID Controllers needed for the accurate aligning to an april tag
  private PIDController forwardPID = new PIDController(3, 0, 0.001);
  private PIDController strafePID = new PIDController(3, 0, 0.001);
  private PIDController thetaPID = new PIDController(0.05, 0, 0.001);
  // The target pose the robot will drive to, with a random position
  Pose2d targetPose = new Pose2d(14, 4, new Rotation2d(3.14));
  boolean doRejectLeftLL;
  boolean doRejectRightLL;
  boolean isShooting = false;

  // #region Swerve Setup
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(getAdjustedRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  //   /** Clone's the angular velocity input stream and converts it to a fieldRelative input
  // stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(getAdjustedRightX(), getAdjustedRightY())
          .headingWhile(true);

  // #endregion

  // Constructor of the main class
  public RobotContainer() {

    // Logging configuration
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(PDH);

    intake = new IntakeSubsystem();
    feeder = new FeederSubsystem();
    turret = new TurretSubsystem();
    hopper = new HopperSubsystem();

    targetPose = field.getPoseInFrontOfTag(26, 1.5);
    elasticSubsystem.putAutoChooser();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    drivebase.setVisionStdDevs(VecBuilder.fill(1.5, 1.5, 9999));
    // drivebase.updateBotPose(new Pose2d(2, 3, new Rotation2d(0)));
  }

  private DoubleSupplier getAdjustedRightX() {
    return () -> {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      double multiplier = -1.0;

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        multiplier = 1.0;
      }
      return driverXbox.getRightX() * multiplier;
    };
  }

  private DoubleSupplier getAdjustedRightY() {
    return () -> {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      double multiplier = -1.0;

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        multiplier = 1.0;
      }
      return driverXbox.getRightY() * multiplier;
    };
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // #region Ctrl Bindings

    driverXbox
        .rightTrigger(0.2)
        .whileTrue(
            new StartEndCommand(
                () -> {
                  drivebase.setCreepDrive(true);
                },
                () -> {
                  drivebase.setCreepDrive(false);
                }));
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));

    driverXbox
        .leftTrigger(0.2)
        .and(operatorXbox.leftTrigger(0.2).negate())
        .whileTrue(
            new StartEndCommand(
                () -> intake.setState(IntakeState.INTAKING_FUEL),
                () -> intake.setState(IntakeState.NORMAL),
                intake));

    operatorXbox
        .rightTrigger(0.2)
        .and(operatorXbox.a().negate())
        .whileTrue(new ShootFuel(turret, drivebase::getPose, hopper, feeder, lights))
        .whileFalse(new ResetTurret(turret, hopper, feeder));

    operatorXbox
        .leftTrigger(0.2)
        .whileTrue(
            new StartEndCommand(
                () -> intake.setState(IntakeState.AGITATING_FUEL),
                () -> intake.setState(IntakeState.NORMAL),
                intake));

    operatorXbox
        .a()
        .whileTrue(new FixJam(turret, hopper, feeder))
        .onFalse(new ResetTurret(turret, hopper, feeder));

    // driverXbox.leftBumper().whileTrue(new ParallelCommandGroup(Commands.runOnce(() ->
    // hopper.run(), hopper), Commands.runOnce(() -> feeder.run(), feeder)));
    // #endregion
    // drivebase.updateBotPose(new Pose2d(2, 4, new Rotation2d(0)));
    // turret.setDefaultCommand(new ShootFuel(turret, drivebase::getPose));
    // driverXbox.b().onTrue(new ShootFuel(turret, drivebase::getPose));
  }

  public void enabledPeriodic() {
    turret.run();
    intake.run();
  }

  public void robotPeriodic() {
    setLights();
    lights.run();
    updateTelemetry();
  }

  // #endregion
  // #region Telemetry

  public void updateTelemetry() {
    Rotation2d gyroAngle = drivebase.getHeading();
    double angularVelocity = Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        "limelight-left", gyroAngle.getDegrees(), angularVelocity, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(
        "limelight-right", gyroAngle.getDegrees(), angularVelocity, 0, 0, 0, 0);

    try {
      processLimelightUpdate("limelight-left");
      //  processLimelightUpdate("limelight-right");
    } catch (NullPointerException e) {
      System.out.println("NO LL DATA");
    }
  }

  private void processLimelightUpdate(String name) {
    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    // Reject if no tags or spinning too fast
    if (estimate.tagCount == 0) return;
    if (Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > Math.toRadians(720)) return;

    double distanceToCurrent =
        drivebase.getPose().getTranslation().getDistance(estimate.pose.getTranslation());
    if (distanceToCurrent > 2) return;

    drivebase.updateBotPose(estimate.pose, estimate.timestampSeconds);
  }

  // #endregion
  // #region Generic

  public void setLights() {
    lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(Color.kGreen).withPriority(3));
    if (DriverStation.isDisabled())
      lights.requestLEDState(new LEDRequest(LEDState.RAINBOW).withPriority(-1));
  }

  //   public Command getAutonomousCommand() {
  //     switch(elasticSubsystem.getSelectedAuto()){
  //         case "DriveForward":
  //         return new DriveForward(drivebase, drivebase::getPose, lights);
  //     }

  //       return drivebase.getAutonomousCommand(elasticSubsystem.getSelectedAuto());
  //   }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  // Overrides the path following in the driver tries to break free
  private boolean driverOverride() {
    double x = driverXbox.getRightX();
    double y = driverXbox.getRightY();
    double threshold = 0.5;

    return Math.abs(x) > threshold || Math.abs(y) > threshold;
  }

  // #endregion
}
