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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeState;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;
import java.io.File;
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
  final CommandXboxController driverXbox =
      new CommandXboxController(0); // Controller, to USB port 0
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(
          new File(
              Filesystem.getDeployDirectory(),
              "swerve/neo")); // The swerve base, with the variable from the json files
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
  // TurretSubsystem turret;

  // The 3 PID Controllers needed for the accurate aligning to an april tag
  private PIDController forwardPID = new PIDController(3, 0, 0.001);
  private PIDController strafePID = new PIDController(3, 0, 0.001);
  private PIDController thetaPID = new PIDController(0.05, 0, 0.001);
  // The target pose the robot will drive to, with a random position
  Pose2d targetPose = new Pose2d(14, 4, new Rotation2d(3.14));
  boolean doRejectUpdate;
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
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  //   /** Clone's the angular velocity input stream and converts it to a fieldRelative input
  // stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  //   // /**
  //   //  * Clone's the angular velocity input stream and converts it to a robotRelative
  //   //  * input stream.
  //   //  */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  // #endregion

  // Constructor of the main class
  public RobotContainer() {

    // Logging configuration
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(PDH);

    intake = new IntakeSubsystem();
    feeder = new FeederSubsystem();
    // turret = new TurretSubsystem();
    hopper = new HopperSubsystem();

    targetPose = field.getPoseInFrontOfTag(26, 1.5);
    elasticSubsystem.putAutoChooser();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

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
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    driverXbox
        .leftTrigger(0.2)
        .whileTrue(
            new StartEndCommand(
                () -> intake.setState(IntakeState.INTAKING_FUEL),
                () -> intake.setState(IntakeState.NORMAL)));
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> isShooting = !isShooting));
    // #endregion

  }

  public void enabledPeriodic() {
    if (isShooting) {
      hopper.run();
      feeder.run();
      // turret.runFlywheel();
    }
  }

  public void robotPeriodic() {
    // setLights();
    // lights.run();
    // updateTelemetry();
  }

  // #endregion
  // #region Telemetry

  public void updateTelemetry() {
    try {
      doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          "limelight-turret",
          drivebase.getHeading().getDegrees(),
          Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond),
          drivebase.getPitch().getDegrees(),
          0,
          0,
          0);

      LimelightHelpers.PoseEstimate robotPosition =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret");

      if (Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > Math.toRadians(120)) {

        doRejectUpdate = true;
      }
      if (robotPosition.tagCount < 1) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        drivebase.setVisionStdDevs(VecBuilder.fill(1.5, 1.5, 9999));
        drivebase.updateBotPose(robotPosition.pose);
      }
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("NO DATA FROM LIMELIGHT(S) | " + e.getLocalizedMessage());
    }
  }

  // #endregion
  // #region Generic

  public void setLights() {
    // if (drivebase.getCreepDrive())
    //     lights.requestLEDState(new
    // LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kRed))
    //             .withPriority(4).withBlinkRate(0.7));
    // else
    //     lights.requestLEDState(new
    // LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kGreen))
    //             .withPriority(5));

    if (!ElasticSubsystem.getBoolean("Lights Switch"))
      lights.requestLEDState(new LEDRequest(LEDState.OFF).withPriority(-999));

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
