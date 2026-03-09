package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import java.util.function.Supplier;

public class DriveForward extends SequentialCommandGroup {

  public DriveForward(
      SwerveSubsystem drivebase, Supplier<Pose2d> targetPose, LightsSubsystem lights) {

    addCommands(
        new ParallelDeadlineGroup(
            drivebase.driveToPose(targetPose.get()),
            new RunCommand(
                () ->
                    lights.requestLEDState(
                        new LEDRequest(LEDState.BLINK)
                            .withBlinkRate(0.4)
                            .withColour(HelperFunctions.convertToGRB(Color.kWhite))
                            .withPriority(2)))));
  }
}
