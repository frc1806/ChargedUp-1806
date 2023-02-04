package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase{

    private Intake mIntake;

    public ToggleIntake(Intake intake){
        mIntake = intake;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        boolean isOpen = false;

        if(isOpen == false){
            isOpen = true;
            mIntake.openBoth();
        } else {
            isOpen = false;
            mIntake.closeBoth();
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
