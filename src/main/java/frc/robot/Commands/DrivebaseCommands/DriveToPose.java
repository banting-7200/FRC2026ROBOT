package frc.robot.Commands.DrivebaseCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
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

public class DriveToPose extends SequentialCommandGroup {

    public DriveToPose(SwerveSubsystem drivebase, Supplier<Pose2d> targetPose, PIDController forwardPID, PIDController strafePID,
            PIDController thetaPID, BooleanSupplier driverOverride, LightsSubsystem lights) {

        addCommands(
            
        new ParallelDeadlineGroup(drivebase.driveToPose(targetPose.get()),
                new RunCommand(() -> lights.requestLEDState(new LEDRequest(LEDState.BLINK).withBlinkRate(0.4)
                        .withColour(HelperFunctions.convertToGRB(Color.kWhite)).withPriority(2)))),


                new ParallelDeadlineGroup(new AlignToPose(drivebase, targetPose, forwardPID, strafePID, thetaPID),
                        new RunCommand(() -> lights.requestLEDState(new LEDRequest(LEDState.BLINK).withBlinkRate(0.2)
                                .withColour(HelperFunctions.convertToGRB(Color.kBlue)).withPriority(1)))).until(driverOverride));
        addRequirements(drivebase);
    }

}
