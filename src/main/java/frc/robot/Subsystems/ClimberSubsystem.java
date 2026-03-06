package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.DIOPorts;

public class ClimberSubsystem extends SubsystemBase {

    enum ClimbMode {
        DISABLED,
        UP,
        DOWN
    }

    SparkFlex motor;
    ClimbMode mode = ClimbMode.DISABLED;
    double motorSpeed = 0; 
    DigitalInput topClimberLimit;
    DigitalInput bottomClimberLimit;

    public ClimberSubsystem() {
        motor = new SparkFlex(CANIds.CLIMB_ID, MotorType.kBrushless);
        topClimberLimit = new DigitalInput(DIOPorts.TOP_CLIMBER_LIMIT_SWITCH);
        bottomClimberLimit = new DigitalInput(DIOPorts.BOTTOM_CLIMBER_LIMIT_SWITCH);
    }

    public void beginDown(double speed) {
        mode = ClimbMode.DOWN;
        motorSpeed = speed;
    }

    public void beginUp(double speed) {
        mode = ClimbMode.UP;
        motorSpeed = speed;
    }

    public void end() {
        mode = ClimbMode.DISABLED;
    }

    public void run() {
        switch (mode) {
            case UP -> moveUp();
            case DOWN -> moveDown();
            case DISABLED -> disable();
        }
    }

    private boolean limitsReached() {
        if (topClimberLimit.get())
            return true;
        if (bottomClimberLimit.get())
            return true;
        return false;
    }

    private void moveUp() {
        if (limitsReached()){
            mode = ClimbMode.DISABLED;
            return;
        }
        motor.set(motorSpeed);
    }

    private void moveDown() {
        if (limitsReached()) {
            mode = ClimbMode.DISABLED;
            return;
        }
        motor.set(-motorSpeed);
    }

    private void disable() {
        motor.set(0);
        motorSpeed = 0; 
    }
}
