package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase{

    public Claw mClaw;

    public CloseClaw(Claw claw){
        mClaw = claw;
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
        mClaw.closeBoth();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
