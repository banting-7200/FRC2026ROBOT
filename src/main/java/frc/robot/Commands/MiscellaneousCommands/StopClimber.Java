package frc.robot.Commands.MiscellaneousCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class LowerClimber extends Command {

ClimberSubsystem climber;

  public LowerClimber(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Runs once when command is run
  @Override
  public void initialize() {}

  // Runs constantly while command is running
  @Override
  public void execute() {
    climber.end();
  }

  // Decide when the command should be finished
  @Override
  public boolean isFinished() {
    return false;
  }

  // Do something when command is finished
  @Override
  public void end(boolean interrupted) {}
}