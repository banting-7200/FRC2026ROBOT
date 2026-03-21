package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Utilites.Constants.FeederConstants;
import frc.robot.Utilites.Constants.HopperConstants;
import frc.robot.Utilites.Constants.TurretConstants;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import java.util.Optional;
import java.util.function.Supplier;

public class ShootFuel extends Command {

  HopperSubsystem hopper;
  FeederSubsystem feeder;
  TurretSubsystem turret;
  LightsSubsystem lights;
  FieldLayout field = new FieldLayout();
  Supplier<Pose2d> robotPose;
  Pose2d pose;
  double desiredAngle;
  double tolerance = 50;
  boolean isAlignedWithFeeder;

  public ShootFuel(
      TurretSubsystem turret,
      Supplier<Pose2d> robotPose,
      HopperSubsystem hopper,
      FeederSubsystem feeder,
      LightsSubsystem lights) {
    this.turret = turret;
    this.robotPose = robotPose;
    this.hopper = hopper;
    this.feeder = feeder;
    this.lights = lights;
    addRequirements(turret, hopper, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pose = robotPose.get();
    Translation2d turretOffset =
        new Translation2d(
            TurretConstants.TURRET_FORWARD_OFFSET, TurretConstants.TURRET_RIGHT_OFFSET);
    Transform2d robotToTurret = new Transform2d(turretOffset, new Rotation2d());
    Translation2d turretFieldPosition = pose.transformBy(robotToTurret).getTranslation();
    Translation2d targetPosition = field.getBlueHubPose().getTranslation();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (field.isRobotInNeutralZone(pose)) {
        if (field.isRobotInTopNeutralZone(pose)) {
          targetPosition =
              (alliance.get() == Alliance.Blue)
                  ? field.getBlueTopPassingPose().getTranslation()
                  : field.getRedTopPassingPose().getTranslation();
        } else if (field.isRobotInBottomNeutralZone(pose)) {
          targetPosition =
              (alliance.get() == Alliance.Blue)
                  ? field.getBlueBottomPassingPose().getTranslation()
                  : field.getRedBottomPassingPose().getTranslation();
        }
      } else {
        targetPosition =
            (alliance.get() == Alliance.Blue)
                ? field.getBlueHubPose().getTranslation()
                : field.getRedHubPose().getTranslation();
      }
    }

    Translation2d turretToTargetVector = targetPosition.minus(turretFieldPosition); // CCW+
    Double distanceToHub = turretToTargetVector.getNorm();
    Rotation2d turretFieldAngle = turretToTargetVector.getAngle(); // Angle to HUB
    Rotation2d robotRelativeTurretAngle =
        turretFieldAngle.minus(pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    double desiredAngleInDegrees = robotRelativeTurretAngle.getDegrees();
    desiredAngle = desiredAngleInDegrees;
    turret.setTurretAngle(desiredAngleInDegrees);

    if (field.isRobotInNeutralZone(pose)) {
      if (!field.isRobotInTopNeutralZone(pose) && !field.isRobotInBottomNeutralZone(pose)) {
        lights.requestLEDState(
            new LEDRequest(LEDState.BLINK)
                .withBlinkRate(0.1)
                .withColour(Color.kOrange)
                .withPriority(0));
        return;
      } else {
        if (distanceToHub > TurretConstants.MAX_DISTANCE
            || distanceToHub < TurretConstants.MIN_DISTANCE) {
          lights.requestLEDState(
              new LEDRequest(LEDState.BLINK)
                  .withBlinkRate(0.1)
                  .withColour(Color.kOrange)
                  .withPriority(0));
          return;
        }
      }
    }

    turret.setTurretHubDistance(distanceToHub);
    isAlignedWithFeeder = (Math.abs(desiredAngleInDegrees - (-90))) <= tolerance;
    if (isAlignedWithFeeder)
      lights.requestLEDState(
          new LEDRequest(LEDState.SOLID).withColour(Color.kBlue).withPriority(0));
    if (field.isRobotInNeutralZone(pose)) {
      feeder.set(FeederConstants.BELT_RPM, FeederConstants.FLYWHEEL_RPM);
      hopper.set(HopperConstants.SHOOTING_RPM);
    } else {
      if (turret.getFlywheelRPM() > TurretConstants.SHOOTING_RPM - 600) {
        feeder.set(FeederConstants.BELT_RPM, FeederConstants.FLYWHEEL_RPM);
        hopper.set(HopperConstants.SHOOTING_RPM);
      } else {
        lights.requestLEDState(
            new LEDRequest(LEDState.BLINK)
                .withBlinkRate(0.1)
                .withColour(Color.kRed)
                .withPriority(1));
        feeder.set(0, 0);
        hopper.set(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.set(0, 0);
    hopper.set(0);
    turret.reset();
  }
}
