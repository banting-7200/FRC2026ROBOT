// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
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
  // TurretSubsystem turret = new TurretSubsystem(); // The turret
  IntakeSubsystem intake;

  //   HopperSubsystem hopper;
  //   FeederSubsystem feeder;
  //   TurretSubsystem turret;

  // The 3 PID Controllers needed for the accurate aligning to an april tag
  private PIDController forwardPID = new PIDController(3, 0, 0.001);
  private PIDController strafePID = new PIDController(3, 0, 0.001);
  private PIDController thetaPID = new PIDController(0.05, 0, 0.001);
  // The target pose the robot will drive to, with a random position
  Pose2d targetPose = new Pose2d(2, 5, new Rotation2d(0));

  boolean doRejectUpdate;

  // #region Swerve Setup
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * 1,
              () -> driverXbox.getLeftX() * 1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(
              () -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
          .headingWhile(true);

  // /**
  //  * Clone's the angular velocity input stream and converts it to a robotRelative
  //  * input stream.
  //  */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  // #endregion

  // Constructor of the main class
  public RobotContainer() {

    intake = new IntakeSubsystem();
    // feeder = new FeederSubsystem();
    // turret = new TurretSubsystem();
    // Changes the target pose to in front of blue hub
    targetPose = field.getPoseInFrontOfTag(26, 1.5);
    // Setup the drive dashboard
    elasticSubsystem.putAutoChooser();
    // Setup commands to be used in auton
    registerNamedCommands();
    // Configure the controller button bindings
    configureBindings();
    // Stop sounding the alarm when the controller isnt connected
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    // Main drive Command
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    // @SuppressWarnings("unused")
    // // "Better" drive command that still needs to be tested, requires roborio 2.0
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //         driveDirectAngle);

    //  // This line activates the drivebase
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // // #region Ctrl Bindings
    // // Creep drive
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

    // driverXbox
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               turret.toggleFlywheel();
    //               feeder.toggleState();
    //             }));
    // driverXbox.b().onTrue(new DriveToPose(drivebase, () -> targetPose, forwardPID, strafePID,
    // thetaPID,
    //         this::driverOverride, lights));
    driverXbox.y().onTrue(Commands.runOnce(intake::togglePosition));
    driverXbox.b().onTrue(Commands.runOnce(intake::spinUp));
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // #endregion

  }

  public void enabledPeriodic() {
    intake.run();
    // turret.run();
    // feeder.run();
  }

  public void robotPeriodic() {
    // setLights(); // Set the light pattern
    // lights.run(); // Turn the lights on
    sendDashboardData(); // Send dashboard data
  }

  // #region Dashboard

  public void sendDashboardData() {
    //     ElasticSubsystem.putBoolean("Rejecting Telemetry Updates", doRejectUpdate);
    //     ElasticSubsystem.putColor("Lights",
    // HelperFunctions.convertToGRB(lights.getLEDRequest().getColour()));
    //     ElasticSubsystem.putString("Target Pose", targetPose.toString());
    //    // ElasticSubsystem.putString("Robot Pose", drivebase.getPose().toString());
    //     ElasticSubsystem.putNumber("Total Current Pull", PDH.getTotalCurrent());
    // ElasticSubsystem.putBoolean("Is Creep Drive", drivebase.getCreepDrive());
    // ElasticSubsystem.putNumber("Flywheel rpm", turret.getRPM());
  }

  public void setupDashboard() {
    ElasticSubsystem.putBoolean("Lights Switch", true);
  }

  // #endregion
  // #region Telemetry

  // public void updateTelemetry() {
  //     // try-catch in case the limelight disconnects for a millisecond
  //     try {
  //         doRejectUpdate = false;
  //         // Send the ll4 IMU data from the pigeon
  //         LimelightHelpers.SetRobotOrientation(
  //                 "limelight-back",
  //                 drivebase.getHeading().getDegrees(),
  //                 Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond),
  //                 drivebase.getPitch().getDegrees(), 0, 0, 0);
  //         // Send the other ll4 IMU data from the pigeon
  //         LimelightHelpers.SetRobotOrientation(
  //                 "limelight-front",
  //                 drivebase.getHeading().getDegrees(),
  //                 Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond),
  //                 drivebase.getPitch().getDegrees(), 0, 0, 0);
  //         // Get the estimated position from the back camera
  //         LimelightHelpers.PoseEstimate robotPositionBack = LimelightHelpers
  //                 .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
  //         // Get the esitamted position form the front camera
  //         LimelightHelpers.PoseEstimate robotPositionFront = LimelightHelpers
  //                 .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

  //         // reject vision while spinning too fast
  //         if (Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > Math.toRadians(120))
  // {
  //             doRejectUpdate = true;
  //         }
  //         // reject vision while a tag is not seen
  //         if (robotPositionBack.tagCount < 1 && robotPositionFront.tagCount < 1) {
  //             doRejectUpdate = true;
  //         }

  //         if (!doRejectUpdate) {
  //             // set the "trust", in cameras
  //             drivebase.setVisionStdDevs(VecBuilder.fill(1.5, 1.5, 10));
  //             if (robotPositionBack.tagCount > 0) {
  //                 // Send the pose to the drivebase
  //                 drivebase.updateBotPose(robotPositionBack.pose);
  //             }
  //             if (robotPositionFront.tagCount > 0) {
  //                 // Send the pose to the drivebase
  //                 drivebase.updateBotPose(robotPositionFront.pose);
  //             }

  //         }

  //     } catch (Exception e) {
  //         e.printStackTrace();
  //         System.out.println("NO DATA FROM LIMELIGHT(S) | " + e.getLocalizedMessage());
  //     }
  // }
  // #endregion
  // #region Generic

  // Set the lights to different patterns
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

  // Run the auto that was selected in the driver dashboard
  // public Command getAutonomousCommand() {
  //     return drivebase.getAutonomousCommand(elasticSubsystem.getSelectedAuto());
  // }

  // Turn on the wheel brakes
  // public void setMotorBrake(boolean brake) {
  //     drivebase.setMotorBrake(brake);
  // }

  // Overrides the path following in the driver tries to break free
  private boolean driverOverride() {
    double x = driverXbox.getRightX();
    double y = driverXbox.getRightY();
    double threshold = 0.5;

    return Math.abs(x) > threshold || Math.abs(y) > threshold;
  }

  // #endregion
  // #region NamedCommands

  public void registerNamedCommands() {
    // NamedCommands.registerCommand("Tag26_1.5m",
    //         new DriveToPose(drivebase, () -> field.getPoseInFrontOfTag(26, 1.5), forwardPID,
    // strafePID, thetaPID, () -> false, lights));

  }
  // #endregion
}
