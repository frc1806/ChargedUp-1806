package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.VisionSubsystem;

public class RearVisionSteerAndDrive extends CommandBase{
    private DriveTrain mDriveTrain;
    private DriverControls mDriveControls;
    private VisionSubsystem mVisionSubsystem;

    public RearVisionSteerAndDrive(DriveTrain drivetrain, DriverControls driveControls, VisionSubsystem visionSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDriveTrain = drivetrain;
        mDriveControls = driveControls;
        mVisionSubsystem = visionSubsystem;
        addRequirements(drivetrain);
      }


    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    mDriveTrain.setDriveMode(mDriveControls.getThrottle(), -mVisionSubsystem.getTargetYaw() * .015, true);
    
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
