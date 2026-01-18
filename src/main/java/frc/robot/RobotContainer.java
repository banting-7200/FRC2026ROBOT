// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DrivebaseCommands.DriveToPose;
import frc.robot.Commands.FuelHandingCommands.IntakeFuel;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.Constants.OperatorConstants;
import frc.robot.Utilites.Constants.PWMPorts;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import frc.robot.Utilites.LimelightHelpers;

import java.io.File;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
    final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));
    LightsSubsystem lights = new LightsSubsystem(PWMPorts.LIGHT_PORT, Constants.LIGHTS_AMOUNT);
    ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    IntakeSubsystem intake = new IntakeSubsystem();
    PowerDistribution PDH = new PowerDistribution(Constants.CANIds.PDH_ID, ModuleType.kRev);
    FieldLayout field = new FieldLayout();

    // Fine align tuning
    private PIDController forwardPID = new PIDController(3, 0, 0.001);
    private PIDController strafePID = new PIDController(3, 0, 0.001);
    private PIDController thetaPID = new PIDController(0.05, 0, 0.001);

    Pose2d targetPose = new Pose2d(2, 5, new Rotation2d(0));

    boolean doRejectUpdate;

    // #region Swerve Setup
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverPS4.getLeftY() * -1,
            () -> driverPS4.getLeftX() * -1)
            .withControllerRotationAxis(driverPS4::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPS4::getRightX,
            driverPS4::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

  

    // #endregion

    public RobotContainer() {
        targetPose = field.getPoseInFrontOfTag(26, 1.5);
        elasticSubsystem.putAutoChooser();
        registerNamedCommands();

        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

    }

    private void configureBindings() {
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        @SuppressWarnings("unused")
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngle);

        
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle); // The actual drive command
        
        // #region Ctrl Bindings
        
        // driverXbox.rightTrigger(0.2).whileTrue(new StartEndCommand(() -> {
        //     drivebase.setCreepDrive(true);
        // }, () -> {
        //     drivebase.setCreepDrive(false);
        // }));

         driverPS4.axisGreaterThan(4, 0.2).whileTrue(new StartEndCommand(() -> {
            drivebase.setCreepDrive(true);
        }, () -> {
            drivebase.setCreepDrive(false);
        }));

        driverPS4.circle().onTrue(new DriveToPose(drivebase, () -> targetPose, forwardPID, strafePID, thetaPID,
                this::driverOverride, lights));

        driverPS4.button(0).onTrue((Commands.runOnce(drivebase::zeroGyro)));

        driverPS4.triangle().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(drivebase.getPose())));
        driverPS4.square().whileTrue(new IntakeFuel(intake));

        // #endregion

    }

    public void enabledPeriodic() {

    }

    public void robotPeriodic() {
        setLights();
        lights.run();
        sendDashboardData();
    }

    // #region Dashboard

    public void sendDashboardData() {
        ElasticSubsystem.putBoolean("Rejecting Telemetry Updates", doRejectUpdate);
        ElasticSubsystem.putColor("Lights", HelperFunctions.convertToGRB(lights.getLEDRequest().getColour()));
        ElasticSubsystem.putString("Target Pose", targetPose.toString());
        ElasticSubsystem.putString("Robot Pose", drivebase.getPose().toString());
        ElasticSubsystem.putNumber("Total Current Pull", PDH.getTotalCurrent());
        ElasticSubsystem.putBoolean("Is Creep Drive", drivebase.getCreepDrive());
    }

    public void setupDashboard() {
        ElasticSubsystem.putBoolean("Lights Switch", true);
    }

    // #endregion
    // #region Telemetry

    public void updateTelemetry() {
        try {
            doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(
                    "limelight-back",
                    drivebase.getHeading().getDegrees(),
                    Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond),
                    drivebase.getPitch().getDegrees(), 0, 0, 0);

            LimelightHelpers.SetRobotOrientation(
                    "limelight-front",
                    drivebase.getHeading().getDegrees(),
                    Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond),
                    drivebase.getPitch().getDegrees(), 0, 0, 0);

            LimelightHelpers.PoseEstimate robotPositionBack = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

            LimelightHelpers.PoseEstimate robotPositionFront = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

            // reject vision while spinning too fast
            if (Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > Math.toRadians(120)) {
                doRejectUpdate = true;
            }

            if (robotPositionBack.tagCount < 1 && robotPositionFront.tagCount < 1) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                drivebase.setVisionStdDevs(VecBuilder.fill(1.5, 1.5, 10)); // "trust", in cameras
                if (robotPositionBack.tagCount > 0) {
                    drivebase.updateBotPose(robotPositionBack.pose);
                }
                if (robotPositionFront.tagCount > 0) {
                    drivebase.updateBotPose(robotPositionFront.pose);
                }

            }

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("NO DATA FROM LIMELIGHT(S) | " + e.getLocalizedMessage());
        }
    }
    // #endregion
    // #region Generic

    public void setLights() {
        if (drivebase.getCreepDrive())
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kRed))
                    .withPriority(4).withBlinkRate(0.7));
        else
            lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(HelperFunctions.convertToGRB(Color.kGreen))
                    .withPriority(5));

        if (!ElasticSubsystem.getBoolean("Lights Switch")) {
            lights.requestLEDState(new LEDRequest(LEDState.OFF).withPriority(-999));
        }

        if (DriverStation.isDisabled())
            lights.requestLEDState(new LEDRequest(LEDState.RAINBOW).withPriority(-1));
    }

    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand(elasticSubsystem.getSelectedAuto());
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    private boolean driverOverride() {
        double x = driverPS4.getRightX();
        double y = driverPS4.getRightY();
        double threshold = 0.5;

        return Math.abs(x) > threshold || Math.abs(y) > threshold;
    }
    // #endregion
    // #region NamedCommands

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Tag26_1.5m",
                new DriveToPose(drivebase, () -> field.getPoseInFrontOfTag(26, 1.5), forwardPID, strafePID, thetaPID, () -> false, lights));

    }
    // #endregion
}