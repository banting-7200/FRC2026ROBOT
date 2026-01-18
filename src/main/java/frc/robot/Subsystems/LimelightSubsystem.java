package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.LimelightHelpers;
import frc.robot.Utilites.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
  private final String name;

  public LimelightSubsystem(String name) {
    this.name = name;
  }

  public Pose2d getBotPose(double robotYaw) {
    LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return limelightMeasurement.pose;
  }

  public int tagCount() {
    return LimelightHelpers.getTargetCount(name);
  }

  public double getTx(){
    return LimelightHelpers.getTX(name);
  }
  public double getTy(){
    return LimelightHelpers.getTY(name);
  }
  
}