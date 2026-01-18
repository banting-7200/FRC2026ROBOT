package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.DIOPorts;
import frc.robot.Utilites.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    TalonFX yawMotor;
    TalonFX flywheelMotor;
    NeutralModeValue neutralModeValue;
    DigitalInput CWLimitSwitch;
    DigitalInput CCWLimitSwitch;

    boolean isZeroed = false;

    double turretAngleSetpoint = 0;
    double currentYaw = 0;
    double currentPitch = 0;

    public TurretSubsystem() { // Yaw CCW+
        CWLimitSwitch = new DigitalInput(DIOPorts.TALONFX_CW_LIMIT_SWITCH);
        CCWLimitSwitch = new DigitalInput(DIOPorts.TALONFX_CCW_LIMIT_SWITCH);
        neutralModeValue = NeutralModeValue.Coast;

        yawMotor = new TalonFX(CANIds.TURRET_YAW_ID);
        flywheelMotor = new TalonFX(CANIds.TURRET_FLYWHEEL_ID);
        TalonFXConfiguration yawConfigs = new TalonFXConfiguration();
        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();

        yawConfigs.MotorOutput.withInverted(TurretConstants.Yaw.INVERSION)
                .withNeutralMode(neutralModeValue);
        yawConfigs.Slot0.kP = TurretConstants.Yaw.P;
        yawConfigs.Slot0.kI = TurretConstants.Yaw.I;
        yawConfigs.Slot0.kD = TurretConstants.Yaw.D;
        yawConfigs.CurrentLimits.withSupplyCurrentLimit(TurretConstants.Yaw.CURRENT_LIMIT);
        yawConfigs.CurrentLimits.withStatorCurrentLimit(TurretConstants.Yaw.CURRENT_LIMIT);

        yawConfigs.Feedback.SensorToMechanismRatio = TurretConstants.Yaw.GEAR_RATIO;

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

        // 0 degrees forward, 175 degrees of allowed movement in each direction
        // Turret rotations = desiredAngleInDegrees / 360
        double forwardLimitRotations = (175.0 / 360.0);
        double reverseLimitRotations = (-175.0 / 360.0);

        // Forward (Counter-Clockwise) Limit
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = forwardLimitRotations;

        // Reverse (Clockwise) Limit
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = reverseLimitRotations;

        yawConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        ClosedLoopGeneralConfigs loopConfigs = new ClosedLoopGeneralConfigs();
        loopConfigs.ContinuousWrap = false;
        
        yawMotor.getConfigurator().apply(loopConfigs);
        yawMotor.getConfigurator().apply(yawConfigs);
        yawMotor.getConfigurator().apply(softLimits);

        flywheelConfigs.MotorOutput.withInverted(TurretConstants.Flywheel.INVERSION).withNeutralMode(neutralModeValue);
        flywheelConfigs.Slot0.kP = TurretConstants.Flywheel.P;
        flywheelConfigs.Slot0.kP = TurretConstants.Flywheel.I;
        flywheelConfigs.Slot0.kP = TurretConstants.Flywheel.D;
        flywheelConfigs.CurrentLimits.withSupplyCurrentLimit(TurretConstants.Flywheel.CURRENT_LIMIT);
        flywheelConfigs.CurrentLimits.withStatorCurrentLimit(TurretConstants.Flywheel.CURRENT_LIMIT);

        flywheelMotor.getConfigurator().apply(flywheelConfigs);
    }

    public boolean withinHardLimits() {
        return !(CWLimitSwitch.get() || CCWLimitSwitch.get());
    }

    public double getCurrentPosition() {
        currentYaw = yawMotor.getPosition().getValueAsDouble();
        return currentYaw;
    }

    public void reZero() {
        isZeroed = false;
    }

    public void zeroEncoderPeriodic() {
        if (isZeroed)
            return;

        if (CCWLimitSwitch.get()) {
            yawMotor.set(-0.06);
        } else {
            yawMotor.set(0);
            yawMotor.setPosition(0);
            turretAngleSetpoint = TurretConstants.MID_YAW;
            isZeroed = true;
        }
    }

    public void setAngle(double degrees){
        turretAngleSetpoint = degrees / 360;
    }

    public void run() {
        if (!isZeroed) {
            return;
        }
        if (withinHardLimits()) { //TODO Test motion magic
            yawMotor.setControl(new PositionVoltage(turretAngleSetpoint));
        } else {
            yawMotor.set(0);
        }

    }

}