package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase{

    public Claw mClaw;
    public Double mStartTime;

    public CloseClaw(Claw claw){
        
        mClaw = claw;
        addRequirements(claw);
        mStartTime = Double.MAX_VALUE;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        mClaw.closeBoth();
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if(Constants.areLimitSwitchesOnClaw)
        {
            return mClaw.isClawClosed();
        }
        else{
            return Timer.getFPGATimestamp() > mStartTime + 0.75;
        }
        
    }
    
}
