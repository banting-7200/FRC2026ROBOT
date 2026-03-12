package frc.robot.Commands.AutoManualComands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.MainSubsystem;

public class ToggleManual extends Command {

  // Subsystems
  MainSubsystem mainSubsystem;

  public ToggleManual(MainSubsystem mainSubsystem) {
    this.mainSubsystem = mainSubsystem;
    // Takes control of the subsystem
    // Ex. Could drive the swerves, stealing control from operator
    // addRequirements(Subsystem);
  }

  // Runs once when command is run
  @Override
  public void initialize() {
    mainSubsystem.toggleState();
  }

  // Runs constantly while command is running
  @Override
  public void execute() {}

  // Decide when the command should be finished
  @Override
  public boolean isFinished() {
    return false;
  }

  // Do something when command is finished
  @Override
  public void end(boolean interrupted) {}
}
