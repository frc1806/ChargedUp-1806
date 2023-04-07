package frc.robot.commands.balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
    
    private boolean mHasBeenTipped;
    private boolean mIsForward;
    private DriveTrain mDriveTrain;
    private double power;
    private double maxPitch;
    private PIDController mPid;
    private double tippedTime;

    public AutoBalance(DriveTrain driveTrain, boolean isForward){
        mHasBeenTipped = false;
        mIsForward = isForward;
        mDriveTrain = driveTrain;
        addRequirements(mDriveTrain);
        mPid = new PIDController(0.0080, 0.0, 0.0023);
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        mDriveTrain.setDriveMode(0.0, 0.0, false);
    }

    @Override
    public void execute() {
        double absolutePitch = Math.abs(mDriveTrain.getRobotPitch());
        if(absolutePitch > maxPitch ){
            maxPitch = absolutePitch;
        }
        super.execute();
        if(isTipped() && !mHasBeenTipped){
            mHasBeenTipped = true;
            tippedTime = Timer.getFPGATimestamp();
        } 
        power = shouldSlowDown()?-mPid.calculate(mDriveTrain.getRobotPitch(), 0.0):(mIsForward?0.6:-0.6);
        mDriveTrain.setDriveMode((power), 0.0, false);
    }

    @Override
    public void initialize() {
        mHasBeenTipped = false;
        mDriveTrain.setBrakeMode();
        mPid.reset();
        tippedTime = Double.MAX_VALUE;
    }

    @Override
    public boolean isFinished() {
        return false;
        //return mHasBeenTipped && isLevel();
    }

    public boolean isTipped(){
        return (mIsForward?mDriveTrain.getRobotPitch():-mDriveTrain.getRobotPitch()) > Constants.kSlowBalanceTippedDegrees;
    }

    public boolean isLevel(){
        return Math.abs(mDriveTrain.getRobotPitch()) < Constants.kSlowBalanceLevelDegrees;
    }

    public boolean isLeveling(){
        return mDriveTrain.getRobotPitch() < maxPitch - 3.0;
    }

    public boolean shouldSlowDown(){
        return Timer.getFPGATimestamp() > tippedTime + 0.375;
    }
        
}
