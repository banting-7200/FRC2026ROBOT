package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants;

import frc.robot.Utilites.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    SparkFlex flyWheelMotor; // Vortex
    SparkMax pitchMotor; // Neo 550
    SparkMax yawMotor; // Neo 550

    SparkFlexConfig flywheelConfig;
    SparkMaxConfig pitchConfig;
    SparkMaxConfig yawConfig;
    
    SparkClosedLoopController flywheelController;
    SparkClosedLoopController pitchController;
    SparkClosedLoopController yawController;

    double turretAngleSetpoint = 0;
    double currentYaw = 0;
    double currentPitch = 0;

    double flywheelSpeed = 0;

    boolean isSpinning = false;

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    private GenericEntry flywheelRpm = debugTab.addPersistent("Flywheel RPM", 4000).getEntry();
    private GenericEntry flywheelP = debugTab.addPersistent("Flywheel P",0).getEntry();
    private GenericEntry flywheelI = debugTab.addPersistent("Flywheel I",0).getEntry();
    private GenericEntry flywheelD = debugTab.addPersistent("Flywheel D",0).getEntry();
    private GenericEntry flywheelKV = debugTab.addPersistent("Flywheel kv",0).getEntry();

    public TurretSubsystem() { // Yaw CCW+
       flyWheelMotor = new SparkFlex(Constants.CANIds.TURRET_FLYWHEEL_ID, MotorType.kBrushless);
    //    pitchMotor = new SparkMax(Constants.CANIds.TURRET_HOOD_ID, MotorType.kBrushless);
    //    yawMotor = new SparkMax(Constants.CANIds.TURRET_YAW_ID, MotorType.kBrushless);

       flywheelController = flyWheelMotor.getClosedLoopController();
    //    pitchController = pitchMotor.getClosedLoopController();
    //    yawController = yawMotor.getClosedLoopController();

       flywheelConfig = new SparkFlexConfig();
    //    pitchConfig = new SparkMaxConfig();
    //    yawConfig = new SparkMaxConfig();

       flywheelConfig.inverted(true).idleMode(IdleMode.kCoast);
       flywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
       flywheelConfig.closedLoop.pid(0.0000, 0, 0).feedForward.kV(0.00016); // 0.00016
       flyWheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //    pitchConfig.inverted(false).idleMode(IdleMode.kBrake);
    //    pitchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    //    pitchConfig.closedLoop.pid(TurretConstants.Hood.P, TurretConstants.Hood.I, TurretConstants.Hood.D);
    //    pitchMotor.configure(pitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //    yawConfig.inverted(false).idleMode(IdleMode.kCoast);
    //    yawConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    //    yawConfig.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
    //    yawConfig.absoluteEncoder.inverted(false);
    //    yawConfig.closedLoop.pid(TurretConstants.Yaw.P, TurretConstants.Yaw.I, TurretConstants.Yaw.D);
    //    yawMotor.configure(yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }




    public void setAngle(double degrees){
        turretAngleSetpoint = degrees / 360;
    }

    public void setFlywheelSpeed(double rpm){
        flywheelSpeed = rpm;
    }

    public void run() {
        System.out.println(flyWheelMotor.getEncoder().getVelocity());
        flywheelController.setSetpoint(flywheelSpeed, ControlType.kVelocity);    

    }

    public void toggleFlywheel(){
        isSpinning = !isSpinning;
        if(isSpinning) flywheelSpeed = flywheelRpm.getDouble(4000);
        else flywheelSpeed = 0;
    }

    public double getRPM(){
        return flyWheelMotor.getEncoder().getVelocity();
    }


}