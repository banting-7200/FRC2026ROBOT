package frc.robot.Utilites;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  // #region Generic
  public static final int XBOX_PORT = 0;
  public static final int BUTTON_BOX_PORT = 1;
  public static final int LIGHTS_AMOUNT = 40;

  public static final class DrivebaseConstants {
    public static final double ROBOT_MASS = 63 * 0.453592; // 63lbs to kg
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double MAX_CREEP_SPEED = Units.feetToMeters(2);
    public static final double MAX_ANGULAR_VELOCITY = 5.627209491911525; // radians per second
    public static final double MAX_CREEP_ANGULAR_VELOCITY = 3;
    public static final double CRAZY_SPIN_SPEED = Units.feetToMeters(3);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.3;
    public static final double LEFT_Y_DEADBAND = 0.4;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class PWMPorts {
    public static final int LIGHT_PORT = 0;
  }

  public static final class DIOPorts {
    public static final int TALONFX_CW_LIMIT_SWITCH = 1;
    public static final int TALONFX_CCW_LIMIT_SWITCH = 0;
  }

  public static final class CANIds {
    public static final int GYRO_ID = 1;
    public static final int INTAKE_ID = 2;
    public static final int INTAKE_PIVOT_ID = 3;
    public static final int HOPPER_ID = 4;
    public static final int FEEDER_FLYWHEEL_ID = 5;
    public static final int FEEDER_BELT_ID = 6;
    public static final int TURRET_YAW_ID = 7;
    public static final int TURRET_HOOD_ID = 8;
    public static final int TURRET_FLYWHEEL_ID = 9;
    // 10 - 17 swerves
    public static final int CLIMBER_ID = 18;
    public static final int PDH_ID = 19;
  }

  // #endregion
  // #region Intake
  public static final class IntakeConstants {
    public static final class Pivot {
      public static final double P = 0.012;
      public static final double I = 0;
      public static final double D = 0.007;
      public static final int CURRENT_LIMIT = 20;
      public static final boolean INVERSION = false;
    }

    public static final class Intake {
      public static final double P = 0.0001;
      public static final double I = 0;
      public static final double D = 0;
      public static final double V = 0.0026;
      public static final int CURRENT_LIMIT = 20;
    }

    public static final double INTAKE_POSITION = 117;
    public static final double STORED_POSITION = 178; // 198
    public static final double NORMAL_POSITION = 214; // 160

    public static final double LOW_AGITATE_POSITION = 120;
    public static final double HIGH_AGITATE_POSITION = 214;

    public static final double INTAKE_RPM = 0;
  }

  public static final class HopperConstants {
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double CURRENT_LIMIT = 40;
    public static final double SHOOTING_RPM = 1000;
  }

  // #endregion
  // #region Feeder

  public static final class FeederConstants {
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double CURRENT_LIMIT = 40;

    public static final double BELT_RPM = 2600;
    public static final double FLYWHEEL_RPM = 3000;
  }

  // #endregion
  // #region Turret
  public static final class TurretConstants {
    public static final class Yaw {
      public static final double P = 0.02; // 0.022
      public static final double I = 0;
      public static final double D = 0.01; // 0.012
      public static final double CURRENT_LIMIT = 40;
      public static final double GEAR_RATIO = 1.6; // 15 / 9.375
    }

    public static final class Flywheel {
      public static final double P = 0.00001;
      public static final double I = 0;
      public static final double D = 0;
      public static final double V = 0.0021;
      public static final int CURRENT_LIMIT = 40;
    }

    public static final class Hood {
      public static final double P = 0.05;
      public static final double I = 0;
      public static final double D = 0.003;
      public static final int CURRENT_LIMIT = 10;
    }

    public static final double MAX_DISTANCE = 4.8;
    public static final double MAX_DISTANCE_RPM = 5000;
    public static final double MAX_DISTANCE_PITCH = 270;
    public static final double MIN_DISTANCE = 2;
    public static final double MIN_DISTANCE_RPM = 3480; // 3630
    public static final double MIN_DISTANCE_PITCH = 300;
    public static final double SHOOTING_RPM = 4200;

    // Max yaw -> turret rotated fully left
    public static final double MIN_YAW = -155;
    public static final double MAX_YAW = 169;

    public static final double TURRET_FORWARD_OFFSET = -0.0984504; // meters
    public static final double TURRET_RIGHT_OFFSET = 0.1682496; // meters, negative for left offset
  }
  // #endregion
}
