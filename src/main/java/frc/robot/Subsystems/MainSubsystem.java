package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MainSubsystem extends SubsystemBase {

  private enum SubState {
    AUTOMATIC,
    MANUAL
  }

  private SubState state = SubState.AUTOMATIC;
  private TurretSubsystem turret;

  public MainSubsystem(TurretSubsystem turret) {
    this.turret = turret;
  }

  public void toggleState() {
    switch (state) {
      case AUTOMATIC -> state = SubState.MANUAL;
      case MANUAL -> state = SubState.AUTOMATIC;
    }
  }

  public void run() {
    switch (state) {
      case AUTOMATIC -> automatedRun();
      case MANUAL -> manualRun();
    }
  }

  private void manualRun() {}

  private void automatedRun() {
    turret.run();
  }
}
