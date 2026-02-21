package frc.robot.Utilites;

import com.ctre.phoenix6.signals.InvertedValue;
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
  public static final int LIGHTS_AMOUNT = 50;

  public static final class DrivebaseConstants {
    public static final double ROBOT_MASS = 63 * 0.453592; // 63lbs to kg
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double MAX_CREEP_SPEED = Units.feetToMeters(5);
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
    public static final int GYRO_ID = 2;
    public static final int INTAKE_ID = 3;
    public static final int INTAKE_PIVOT_ID = 4;
    public static final int HOPPER_ID = 5;
    public static final int FEEDER_ID = 6;
    public static final int TURRET_YAW_ID = 7;
    public static final int TURRET_FLYWHEEL_ID = 8;
    public static final int TURRET_HOOD_ID = 9;
    public static final int PDH_ID = 20;
  }

  // #endregion
  // #region Intake
  public static final class IntakeConstants {
    public static final class Pivot {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double CURRENT_LIMIT = 40;
      public static final boolean INVERSION = false;
    }

    public static final class Intake {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double CURRENT_LIMIT = 40;
    }

    public static final double INTAKE_POSITION = 0;
    public static final double STORED_POSITION = 0;
    public static final double NORMAL_POSITION = 0;

    public static final double INTAKE_RPM = 0;
  }

  // #endregion
  // #region Feeder

  public static final class FeederConstants {
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double CURRENT_LIMIT = 40;

    public static final double OUTPUT_RPM = 0;
  }

  // #endregion
  // #region Turret
  public static final class TurretConstants {
    public static final class Yaw {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double CURRENT_LIMIT = 40;
      public static final InvertedValue INVERSION = InvertedValue.CounterClockwise_Positive;
      public static final double GEAR_RATIO = 10;
    }

    public static final class Flywheel {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double CURRENT_LIMIT = 40;
      public static final InvertedValue INVERSION = InvertedValue.Clockwise_Positive;
    }

    public static final class Hood {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double CURRENT_LIMIT = 40;
    }

    public static final double MAX_DISTANCE = 0;
    public static final double MAX_DISTANCE_RPM = 0;
    public static final double MAX_DISTANCE_PITCH = 0;
    public static final double MIN_DISTANCE = 0;
    public static final double MIN_DISTANCE_RPM = 0;
    public static final double MIN_DISTANCE_PITCH = 0;

    // Min yaw -> turret rotated fully left/west
    public static final double MIN_YAW = 0;
    public static final double MID_YAW = 0;
    public static final double MAX_YAW = 0;

    public static final double TURRET_FORWARD_OFFSET = 0.0; // meters
    public static final double TURRET_RIGHT_OFFSET = 0.0; // meters, negative for left offset
  }
  // #endregion
}
