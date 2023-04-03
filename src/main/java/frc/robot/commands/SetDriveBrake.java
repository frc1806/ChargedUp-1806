package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SetDriveBrake extends CommandBase{

    private DriveTrain mDriveTrain;
    
    public SetDriveBrake(DriveTrain driveTrain){
        mDriveTrain = driveTrain;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        if(mDriveTrain.isBrake){
            mDriveTrain.setCoastMode();
        } else{
            mDriveTrain.setBrakeMode();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    
}
