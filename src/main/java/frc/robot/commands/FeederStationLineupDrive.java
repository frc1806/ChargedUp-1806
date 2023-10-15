package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.game.PosesOfInterest;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Vision;
import frc.robot.util.TrapezoidalDiffDriveTurnPID;

public class FeederStationLineupDrive extends CommandBase{
    private DriveTrain mDriveTrain;
    private DriverControls mDriveControls;
    private Vision mVisionSubsystem;
    private Translation2d mNearestFeederStation;
    TrapezoidalDiffDriveTurnPID trapezoidalDiffDriveTurnPID;

    public FeederStationLineupDrive(DriveTrain drivetrain, DriverControls driveControls, Vision visionSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDriveTrain = drivetrain;
        mDriveControls = driveControls;
        mVisionSubsystem = visionSubsystem;
        addRequirements(drivetrain);
        trapezoidalDiffDriveTurnPID = new TrapezoidalDiffDriveTurnPID(drivetrain);
      }


    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mNearestFeederStation = PosesOfInterest.GetClosestFeederStation(mVisionSubsystem.getCurrentAlliance(), mDriveTrain.getPose());
    trapezoidalDiffDriveTurnPID.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d robotToTarget = mNearestFeederStation.minus(mDriveTrain.getPose().getTranslation()).getAngle();
    Rotation2d yaw = robotToTarget.minus(mDriveTrain.getPose().getRotation());
    mDriveTrain.setDriveMode(mDriveControls.getThrottle(), trapezoidalDiffDriveTurnPID.calculate(yaw.getRadians()), true);
    
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
