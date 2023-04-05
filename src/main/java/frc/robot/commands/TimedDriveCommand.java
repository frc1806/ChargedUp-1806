package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TimedDriveCommand extends CommandBase{

    private DriveTrain mDriveTrain;
    private Double mDriveTime, mDrivePower, start;

    public TimedDriveCommand(DriveTrain driveTrain, Double driveTime, Double drivePower){
        mDriveTrain = driveTrain;
        mDriveTime = driveTime;
        mDrivePower = drivePower;
        start=-Double.MAX_VALUE;
        addRequirements(driveTrain);
    }
    @Override
    public void end(boolean interrupted) {
        mDriveTrain.setDrivePower(0.0);
    }

    @Override
    public void execute() {
        mDriveTrain.setDrivePower(mDrivePower);
    }

    @Override
    public void initialize() {
        start = Timer.getFPGATimestamp();
        mDriveTrain.setDrivePower(mDrivePower);
        mDriveTrain.setCoastMode();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= start + mDriveTime;
    }
    
}
