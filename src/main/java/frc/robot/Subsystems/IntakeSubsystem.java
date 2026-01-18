package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakeState {
        INTAKING_FUEL,
        AGITATING_FUEL,
        STORED,
        NORMAL;
    };

    SparkMax pivotMotor;
    SparkMax intakeMotor;
    SparkClosedLoopController PIDController;
    SparkMaxConfig config;
    IdleMode idleMode;
    double setpoint;
    boolean isAgitating = false;
    boolean isUp = false;

    public IntakeSubsystem() {
        pivotMotor = new SparkMax(CANIds.INTAKE_PIVOT_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        PIDController = pivotMotor.getClosedLoopController();
        idleMode = IdleMode.kBrake;

        config.inverted(IntakeConstants.Pivot.INVERSION).idleMode(idleMode);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
        config.closedLoop.pid(IntakeConstants.Pivot.P, IntakeConstants.Pivot.I, IntakeConstants.Pivot.D);

        pivotMotor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setIdleMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        config.idleMode(idleMode);
        pivotMotor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);

    }

    public void setState(IntakeState state) {
        switch (state) {
            case INTAKING_FUEL:
                isAgitating = false;
                setpoint = IntakeConstants.INTAKE_POSITION;
                intakeMotor.set(1);
                break;
            case AGITATING_FUEL:
                isAgitating = true;
                intakeMotor.set(0);
                break;
            case STORED:
                isAgitating = false;
                setpoint = IntakeConstants.STORED_POSITION;
                intakeMotor.set(0);
                break;
            case NORMAL:
                isAgitating = false;
                setpoint = IntakeConstants.NORMAL_POSITION;
                intakeMotor.set(0);
                break;
        }
    }

    public void run() {
        if (isAgitating) {
            // TODO make setpoint move between stored and normal
        }
        PIDController.setSetpoint(setpoint, ControlType.kPosition); // TODO Better control and intregrated soft limits

    }

}
