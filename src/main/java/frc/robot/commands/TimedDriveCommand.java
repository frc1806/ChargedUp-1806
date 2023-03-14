package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TimedDriveCommand extends CommandBase{

    private DriveTrain mDriveTrain;
    private Double mDriveTime, mDrivePower;

    public TimedDriveCommand(DriveTrain driveTrain, Double driveTime, Double drivePower){
        mDriveTrain = driveTrain;
        mDriveTime = driveTime;
        mDrivePower = drivePower;
        addRequirements(driveTrain);
    }
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        mDriveTrain.setDrivePower(mDrivePower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
