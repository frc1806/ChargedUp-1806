package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RotateCone extends CommandBase{

    private static double mStartTime;

    public RotateCone(){
        addRequirements(RobotContainer.S_INTAKE);
    }
    public void initialize(){
        mStartTime = Timer.getFPGATimestamp();
    }

    public void end(){
        RobotContainer.S_INTAKE.rotateClaw(0);
    }

    public void execute(){
        RobotContainer.S_INTAKE.rotateClaw(Constants.kClawSpinnerSpeed);
    }

    public boolean isFinished(){
        return RobotContainer.S_INTAKE.isClawSpinnerStalled() ||isTimedOut();
    }


    public boolean isTimedOut(){

        return RotateCone.mStartTime + Constants.kClawTimeFrame < Timer.getFPGATimestamp();
    }
}
