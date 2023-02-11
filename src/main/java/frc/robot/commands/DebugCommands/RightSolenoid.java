package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeStates;

public class RightSolenoid extends CommandBase{

    private Claw mClaw;
    private boolean isOpen;

    public RightSolenoid(Claw claw){
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
        if(mClaw.getIntakeState() == IntakeStates.RightClosed){
            isOpen = false;
        } else if(mClaw.getIntakeState() == IntakeStates.RightOpened){
            isOpen = true;
        }

        if(isOpen == false){
            isOpen = true;
            mClaw.openRight();
        } else {
            isOpen = false;
            mClaw.closeRight();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
