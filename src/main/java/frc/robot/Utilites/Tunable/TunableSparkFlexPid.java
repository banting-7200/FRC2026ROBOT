package frc.robot.Utilites.Tunable;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;

public class TunableSparkFlexPid {

  /**
   * Attaches tunables for the PID values to a SparkFlex motor.
   *
   * <p>This allows you to tune the PID and setpoint values from the dashboard without having to
   * redeploy code. The tunables are added to Network Tables under the name "Tunable/name/P",
   * "Tunable/name/I", "Tunable/name/D", "Tunable/name/F", and "Tunable/name/Setpoint" where "name"
   * is the name that is provided to this function.
   *
   * <p>The default values for the tunables are taken from the motor's current configuration. When a
   * tunable is changed, the motor's configuration is updated with the new value, and the motor is
   * reconfigured with the new configuration on the fly.
   *
   * @param name the name to use for the tunables on the dashboard
   * @param motor the SparkFlex motor to attach the tunables to
   * @param config the SparkFlexConfig to use for configuring the motor when the tunables are
   *     changed
   * @param defaultSetpoint the default setpoint to use for the setpoint tunable
   * @return a DoubleEntry that can be used to programmatically set and get the setpoint of the
   *     motor.
   */
  public static DoubleEntry create(
      String name, SparkFlex motor, SparkFlexConfig config, double defaultSetpoint) {
    double p = motor.configAccessor.closedLoop.getP();
    double i = motor.configAccessor.closedLoop.getI();
    double d = motor.configAccessor.closedLoop.getD();
    double kV = motor.configAccessor.closedLoop.feedForward.getkV();

    DogLog.tunable(
        name + "/P",
        p,
        newP -> {
          config.closedLoop.p(newP);
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

    DogLog.tunable(
        name + "/I",
        i,
        newI -> {
          config.closedLoop.i(newI);
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

    DogLog.tunable(
        name + "/D",
        d,
        newD -> {
          config.closedLoop.d(newD);
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

    DogLog.tunable(
        name + "/F",
        kV,
        newkV -> {
          config.closedLoop.feedForward.kV(newkV);
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

    DoubleSubscriber setpoint =
        DogLog.tunable(
            name + "/Setpoint",
            0.0,
            newSetpoint -> {
              motor.getClosedLoopController().setSetpoint(newSetpoint, ControlType.kVelocity);
            });

    return setpoint.getTopic().getEntry(defaultSetpoint);
  }

  public static DoubleEntry create(String name, SparkFlex motor, SparkFlexConfig config) {
    return create(name, motor, config, 0.0);
  }

  private TunableSparkFlexPid() {}
}
