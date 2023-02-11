package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeStates;

public class LeftSolenoid extends CommandBase{

    private Claw mClaw;
    private boolean isOpen;

    public LeftSolenoid(Claw claw){
        mClaw = claw;
        isOpen = false;
        addRequirements(claw);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        if(mClaw.getIntakeState() == IntakeStates.LeftClosed){
            isOpen = false;
        } else if(mClaw.getIntakeState() == IntakeStates.LeftOpen){
            isOpen = true;
        }

        if(isOpen == false){
            isOpen = true;
            mClaw.openLeft();
        } else {
            isOpen = false;
            mClaw.closeLeft();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
