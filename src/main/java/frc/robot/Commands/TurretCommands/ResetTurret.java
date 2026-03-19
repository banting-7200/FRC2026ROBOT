package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class ResetTurret extends Command {
  TurretSubsystem turret;
  HopperSubsystem hopper;
  FeederSubsystem feeder;

  public ResetTurret(TurretSubsystem turret, HopperSubsystem hopper, FeederSubsystem feeder) {
    this.turret = turret;
    this.hopper = hopper;
    this.feeder = feeder;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turret.reset();
    hopper.set(0);
    feeder.set(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
