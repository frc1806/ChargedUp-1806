package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;

public class Drive extends CommandBase{

    private DriveTrain mDriveTrain;
    private DriverControls mDriveControls;

    public Drive(DriveTrain drivetrain, DriverControls driveControls) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDriveTrain = drivetrain;
        mDriveControls = driveControls;
        addRequirements(drivetrain);
      }


    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(mDriveControls.getCreepMode()){
        mDriveTrain.setCreepMode(mDriveControls.getThrottle(), mDriveControls.getTurn(), mDriveControls.getQuickTurn());
    }
    else{
        mDriveTrain.setDriveMode(mDriveControls.getThrottle(), mDriveControls.getTurn(), mDriveControls.getQuickTurn());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
