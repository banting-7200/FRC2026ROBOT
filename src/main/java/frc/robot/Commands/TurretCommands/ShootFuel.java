package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.LightsSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Utilites.Constants.FeederConstants;
import frc.robot.Utilites.Constants.HopperConstants;
import frc.robot.Utilites.Constants.TurretConstants;
import frc.robot.Utilites.FieldLayout;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import java.util.Optional;

// This code was commented and cleaned by AI, code was inspired by:
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
// Had no time to figure it out myself so I just copied this team above :(
public class ShootFuel extends Command {

  private final HopperSubsystem hopper;
  private final FeederSubsystem feeder;
  private final TurretSubsystem turret;
  private final LightsSubsystem lights;
  private final SwerveSubsystem swerve;

  private final FieldLayout field = new FieldLayout();
  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(5);

  // Interpolation Maps
  private final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  public ShootFuel(
      TurretSubsystem turret,
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      FeederSubsystem feeder,
      LightsSubsystem lights) {
    this.turret = turret;
    this.swerve = swerve;
    this.hopper = hopper;
    this.feeder = feeder;
    this.lights = lights;

    addRequirements(turret, hopper, feeder);
    setupMaps();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    Pose2d robotPose = swerve.getPose();
    var robotVelocity = swerve.getRobotVelocity();

    Translation2d turretOffset =
        new Translation2d(
            TurretConstants.TURRET_FORWARD_OFFSET, TurretConstants.TURRET_RIGHT_OFFSET);
    Translation2d turretFieldPos =
        robotPose.transformBy(new Transform2d(turretOffset, new Rotation2d())).getTranslation();

    Translation2d targetPosition = getTargetBasedOnAlliance(robotPose);

    double distanceToTarget = turretFieldPos.getDistance(targetPosition);
    double timeOfFlight = tofMap.get(distanceToTarget);

    var fieldRelativeVelocity =
        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d virtualTarget =
        new Translation2d(
            targetPosition.getX() - (fieldRelativeVelocity.getX() * timeOfFlight),
            targetPosition.getY() - (fieldRelativeVelocity.getY() * timeOfFlight));

    double virtualDistance = virtualTarget.getDistance(turretFieldPos);

    double targetRPM;
    double targetHoodAngle;
    ;

    if (field.isRobotInNeutralZone(robotPose)) {
      // Use your defined MAX constants instead of the map
      targetRPM = 4500;
      targetHoodAngle = 300;
    } else {
      // Use the maps for normal "Sniper" mode
      targetRPM = flywheelMap.get(virtualDistance);
      targetHoodAngle = hoodMap.get(virtualDistance);
    }

    Rotation2d fieldAngleToTarget = virtualTarget.minus(turretFieldPos).getAngle();
    Rotation2d robotRelativeAngle =
        fieldAngleToTarget.minus(robotPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    double filteredAngleDegrees = turretAngleFilter.calculate(robotRelativeAngle.getDegrees());

    turret.setTurretAngle(filteredAngleDegrees);
    turret.setFlywheelRPM(targetRPM);
    turret.setHoodAngle(targetHoodAngle);

    handleFeederAndLights(robotPose, virtualDistance);
  }

  private void handleFeederAndLights(Pose2d robotPose, double distance) {
    boolean inNeutralZone = field.isRobotInNeutralZone(robotPose);
    boolean validZone =
        field.isRobotInTopNeutralZone(robotPose) || field.isRobotInBottomNeutralZone(robotPose);

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

    // if in hub slot in neutral field
    if (inNeutralZone && !validZone) {
      lights.requestLEDState(
          new LEDRequest(LEDState.BLINK).withBlinkRate(0.1).withColour(Color.kOrange));
      return;
    }
    // if in neutral zone
    if (inNeutralZone) {
      lights.requestLEDState(
          new LEDRequest(LEDState.BLINK).withBlinkRate(0.1).withColour(Color.kBlue));
      feeder.set(FeederConstants.BELT_RPM, FeederConstants.FLYWHEEL_RPM);
      hopper.set(HopperConstants.SHOOTING_RPM);
      return;
    }

    // Check if both flywheel and turret are ready
    if (inNeutralZone || turret.getFlywheelRPM() > 500) {
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

  private Translation2d getTargetBasedOnAlliance(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isEmpty() || alliance.get() == Alliance.Blue;

    if (field.isRobotInNeutralZone(robotPose)) {
      if (field.isRobotInTopNeutralZone(robotPose)) {
        return isBlue
            ? field.getBlueTopPassingPose().getTranslation()
            : field.getRedTopPassingPose().getTranslation();
      } else if (field.isRobotInBottomNeutralZone(robotPose)) {
        return isBlue
            ? field.getBlueBottomPassingPose().getTranslation()
            : field.getRedBottomPassingPose().getTranslation();
      }
    }
    return isBlue
        ? field.getBlueHubPose().getTranslation()
        : field.getRedHubPose().getTranslation();
  }

  public void setupMaps() {
    // Distance (m) -> RPM
    flywheelMap.put(2.27, 3450.);
    flywheelMap.put(2.59, 3600.);
    flywheelMap.put(2.97, 3800.);
    flywheelMap.put(3.37, 4000.);
    flywheelMap.put(3.76, 4050.);
    flywheelMap.put(4.16, 4100.);
    flywheelMap.put(4.5, 4200.);
    flywheelMap.put(5., 4600.);

    // Distance (m) -> Hood Angle (Degrees)
    hoodMap.put(2.27, 325.);
    hoodMap.put(2.59, 322.);
    hoodMap.put(2.97, 319.);
    hoodMap.put(3.37, 290.);
    hoodMap.put(3.76, 287.);
    hoodMap.put(4.16, 285.);
    hoodMap.put(4.5, 283.);
    hoodMap.put(5., 285.);

    // Distance (m) -> Time of Flight (seconds)
    tofMap.put(2.27, 1.16);
    tofMap.put(2.59, 1.12);
    tofMap.put(2.97, 1.13);
    tofMap.put(3.37, 1.13);
    tofMap.put(3.76, 1.19);
    tofMap.put(4.16, 1.21);
    tofMap.put(4.5, 1.35);
    tofMap.put(5., 1.5);
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
