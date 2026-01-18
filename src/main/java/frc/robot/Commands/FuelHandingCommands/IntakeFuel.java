package frc.robot.Commands.FuelHandingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeState;

public class IntakeFuel extends Command {

    IntakeSubsystem intake;

    public IntakeFuel(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
        intake.setState(IntakeState.INTAKING_FUEL);
        intake.run();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(IntakeState.NORMAL);
    }
}
