package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class FixJam extends Command {

  TurretSubsystem turret;
  HopperSubsystem hopper;
  FeederSubsystem feeder;

  public FixJam(TurretSubsystem turret, HopperSubsystem hopper, FeederSubsystem feeder) {
    this.turret = turret;
    this.hopper = hopper;
    this.feeder = feeder;
    addRequirements(turret, hopper, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
