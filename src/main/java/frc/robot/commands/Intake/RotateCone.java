package frc.robot.commands.Intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RotateCone extends CommandBase{

    private double mStartTime;

    public RotateCone(){
        addRequirements(RobotContainer.S_CYMBAL_SPEEEEEEN);
    }
    @Override
    public void initialize(){
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.S_CYMBAL_SPEEEEEEN.rotateClaw(0.0);
    }

    @Override
    public void execute(){
        RobotContainer.S_CYMBAL_SPEEEEEEN.rotateClaw(Constants.kClawSpinnerSpeed);
    }

    @Override
    public boolean isFinished(){
        return  RobotContainer.S_CYMBAL_SPEEEEEEN.isClawSpinnerStalled() ||isTimedOut();
    }


    public boolean isTimedOut(){

        return mStartTime + Constants.kClawTimeFrame < Timer.getFPGATimestamp();
    }


}
