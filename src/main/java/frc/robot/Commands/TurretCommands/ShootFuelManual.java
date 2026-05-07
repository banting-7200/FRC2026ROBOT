package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Utilites.Constants.FeederConstants;
import frc.robot.Utilites.Constants.HopperConstants;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;

// This code was commented and cleaned by AI, code was inspired by:
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
// Had no time to figure it out myself so I just copied this team above :(
public class ShootFuelManual extends Command {

  private final CommandXboxController controller;
  private final HopperSubsystem hopper;
  private final FeederSubsystem feeder;
  private final TurretSubsystem turret;
  private final LightsSubsystem lights;

  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(5);

  public ShootFuelManual(
      CommandXboxController controller,
      TurretSubsystem turret,
      HopperSubsystem hopper,
      FeederSubsystem feeder,
      LightsSubsystem lights) {
    this.turret = turret;
    this.controller = controller;
    this.hopper = hopper;
    this.feeder = feeder;
    this.lights = lights;

    addRequirements(turret, hopper, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double targetRPM = 6700;

    double filteredAngleDegrees =
        turretAngleFilter.calculate(
            HelperFunctions.ControllerToTurretDegrees(
                controller.getLeftX(), controller.getLeftY()));

    turret.setFlywheelRPM(targetRPM);
    // turret.setTurretAngle(filteredAngleDegrees);
    // turret.setHoodAngle(targetHoodAngle);

    handleFeeder(targetRPM);
  }

  private void handleFeeder(double targetRPM) {

    double currentAngle = turret.getTurretAngle();
    double deadzoneCenter = -90.0;
    double tolerance = 40.0;

    // Check if the turret is between -120 and -60
    boolean inDeadzone =
        (currentAngle > (deadzoneCenter - tolerance)
            && currentAngle < (deadzoneCenter + tolerance));

    // If we are in the deadzone, kill the motors and show a warning light
    if (inDeadzone) {
      lights.requestLEDState(
          new LEDRequest(LEDState.BLINK)
              .withColour(Color.kWhite)
              .withPriority(1)
              .withBlinkRate(0.15));
      feeder.set(0, 0);
      hopper.set(0);
      return;
    }

    // Check if both flywheel and turret are ready
    if (turret.getFlywheelRPM() > targetRPM - 1000) {
      lights.requestLEDState(
          new LEDRequest(LEDState.SOLID).withColour(Color.kPurple).withPriority(1));
      feeder.set(FeederConstants.BELT_RPM, FeederConstants.FLYWHEEL_RPM);
      hopper.set(HopperConstants.SHOOTING_RPM);
    } else {
      lights.requestLEDState(
          new LEDRequest(LEDState.BLINK)
              .withBlinkRate(0.05)
              .withColour(Color.kRed)
              .withPriority(1));
      feeder.set(0, 0);
      hopper.set(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.set(0, 0);
    hopper.set(0);
    turret.reset();
  }
}
