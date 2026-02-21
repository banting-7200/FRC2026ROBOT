package frc.robot.Utilites;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

public class FieldLayout {
  private AprilTagFieldLayout fieldLayout;

  public FieldLayout() {
    try {
      Path layoutPath =
          Filesystem.getDeployDirectory().toPath().resolve("fields/2026-rebuilt-welded.json");
      fieldLayout = new AprilTagFieldLayout(layoutPath);
    } catch (IOException e) {
      System.out.println("Could not load AprilTag layout");
    }
  }

  public Pose2d getTagPose(int tagID) {
    return fieldLayout.getTagPose(tagID).get().toPose2d();
  }

  public Pose2d getPoseInFrontOfTag(int tagID, double offset) {
    Pose2d tagPose = getTagPose(tagID);

    Translation2d forwardOffset = new Translation2d(offset, 0.0).rotateBy(tagPose.getRotation());

    Rotation2d facingTag = tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

    Pose2d robotPose = new Pose2d(tagPose.getTranslation().plus(forwardOffset), facingTag);

    return robotPose;
  }

  public Pose2d getBlueHubPose() {
    return new Pose2d(4.62, 4.05, new Rotation2d(0)); // Meters
  }

  public Pose2d getRedHubPose() {
    return new Pose2d(11.9, 4.05, new Rotation2d(0)); // Meters
  }

  public boolean isRobotInNeutralZone(Pose2d robotPose) {
    return robotPose.getX() > 5.8 && robotPose.getX() < 10.8; // Meters
  }

  public boolean isRobotNearTrench(Pose2d robotPose) {
    return (robotPose.getX() > 3.4 && robotPose.getX() < 5.8)
        || (robotPose.getX() > 10.8 && robotPose.getX() < 13); // Meters
  }

  public boolean canRobotShoot(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        if (robotPose.getX() < 3) // Meters
        return true;
      } else {
        if (robotPose.getX() > 13.6) // Meters
        return true;
      }
    } else {
      System.out.println("DRIVER STATION ERROR");
    }
    return false;
  }
}
