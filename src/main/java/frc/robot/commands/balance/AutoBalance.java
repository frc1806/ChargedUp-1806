package frc.robot.commands.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
    
    private boolean mHasBeenTipped;
    private boolean mIsForward;
    private DriveTrain mDriveTrain;
    private double power;

    public AutoBalance(DriveTrain driveTrain, boolean isForward){
        mHasBeenTipped = false;
        mIsForward = isForward;
        mDriveTrain = driveTrain;
        addRequirements(mDriveTrain);
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        mDriveTrain.setDriveMode(0.0, 0.0, false);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if(isTipped()){
            mHasBeenTipped = true;
        } 
        power = mHasBeenTipped?0.2:0.4;
        mDriveTrain.setDriveMode((mIsForward?power:-power), 0.0, false);
    }

    @Override
    public void initialize() {
        mHasBeenTipped = false;
        mDriveTrain.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return mHasBeenTipped && isLevel();
    }

    public boolean isTipped(){
        return (mIsForward?mDriveTrain.getRobotPitch():-mDriveTrain.getRobotPitch()) > Constants.kSlowBalanceTippedDegrees;
    }

    public boolean isLevel(){
        return Math.abs(mDriveTrain.getRobotPitch()) < Constants.kSlowBalanceLevelDegrees;
    }
        
}
