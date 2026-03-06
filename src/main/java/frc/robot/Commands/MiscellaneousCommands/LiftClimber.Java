package frc.robot.Commands.MiscellaneousCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class LiftClimber extends Command {

ClimberSubsystem climber;
double speed = 0;
  public LiftClimber(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    addRequirements(climber);
    this.speed = speed;
  }

  // Runs once when command is run
  @Override
  public void initialize() {}

  // Runs constantly while command is running
  @Override
  public void execute() {
    climber.moveUp(speed);

  }

  // Decide when the command should be finished
  @Override
  public boolean isFinished() {
    return false;
  }

  // Do something when command is finished
  @Override
  public void end(boolean interrupted) {
    climber.end();
  }
}
