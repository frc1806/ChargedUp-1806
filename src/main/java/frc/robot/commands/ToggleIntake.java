package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class ToggleIntake extends CommandBase{

    private Intake mIntake;
    boolean isOpen;

    public ToggleIntake(Intake intake){
        mIntake = intake;
        addRequirements(intake);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        //command meant to be ran once so we aren't using it periodically
    }

    @Override
    public void initialize() {
        if(mIntake.getIntakeState() == IntakeStates.Closed){
            isOpen = false;
        } else if(mIntake.getIntakeState() == IntakeStates.Opened){
            isOpen = true;
        }

        if(isOpen == false){
            isOpen = true;
            mIntake.openBoth();
        } else {
            isOpen = false;
            mIntake.closeBoth();
        }
    }

    @Override
    public boolean isFinished() {
        //allows it to run once then immediately stop
        return true;
    }
    
}
