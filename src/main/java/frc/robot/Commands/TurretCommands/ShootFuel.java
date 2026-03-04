package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Utilites.Constants.TurretConstants;
import frc.robot.Utilites.FieldLayout;
import java.util.Optional;
import java.util.function.Supplier;

public class ShootFuel extends Command {

  HopperSubsystem hopper;
  FeederSubsystem feeder;
  TurretSubsystem turret;
  FieldLayout field = new FieldLayout();
  Supplier<Pose2d> robotPose;
  Pose2d pose;
  double desiredAngle;
  double tolerance = 1;

  public ShootFuel(TurretSubsystem turret, Supplier<Pose2d> robotPose) {
    this.turret = turret;
    // this.feeder = feeder;
    // this.hopper = hopper;
    this.robotPose = robotPose;
    addRequirements(turret);
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

    Translation2d targetPosition = new Translation2d();
    // Translation2d targetPosition = field.getTagPose(10).getTranslation();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      targetPosition =
          (alliance.get() == Alliance.Blue)
              ? field.getBlueHubPose().getTranslation()
              : field.getRedHubPose().getTranslation();
    } else {
      System.out.println("NO ALLIANCE | NO SHOOTING");
    }

    Translation2d turretToTargetVector = targetPosition.minus(turretFieldPosition); // CCW+
    Rotation2d turretFieldAngle = turretToTargetVector.getAngle(); // Angle to HUB
    Rotation2d robotRelativeTurretAngle = turretFieldAngle.minus(pose.getRotation());

    double desiredAngleInDegrees = robotRelativeTurretAngle.getDegrees();
    desiredAngle = desiredAngleInDegrees;
    turret.setTurretAngle(desiredAngleInDegrees);
    // feeder.run();
    // hopper.run();
  }

  @Override
  public boolean isFinished() {
    // return turret.getTurretAngle() >= desiredAngle - tolerance
    //     && turret.getTurretAngle() <= desiredAngle + tolerance;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AIMING DONE");
  }
}
