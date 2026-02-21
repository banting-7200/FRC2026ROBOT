package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTemplate extends Command {

  // Subsystems

  public CommandTemplate() {
    // Takes control of the subsystem
    // Ex. Could drive the swerves, stealing control from operator
    // addRequirements(Subsystem);
  }

  // Runs once when command is run
  @Override
  public void initialize() {}

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
