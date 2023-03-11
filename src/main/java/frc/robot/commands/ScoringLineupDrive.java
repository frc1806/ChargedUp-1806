package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.game.PosesOfInterest;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.VisionSubsystem;

public class ScoringLineupDrive extends CommandBase{
    private DriveTrain mDriveTrain;
    private DriverControls mDriveControls;
    private VisionSubsystem mVisionSubsystem;
    private Translation2d mNearestScoringLocation;

    public ScoringLineupDrive(DriveTrain drivetrain, DriverControls driveControls, VisionSubsystem visionSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDriveTrain = drivetrain;
        mDriveControls = driveControls;
        mVisionSubsystem = visionSubsystem;
        addRequirements(drivetrain);
      }


    // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //TODO: When we add robot modes for cubes and cones, make this select  what to lineup on based on that.
    mNearestScoringLocation = PosesOfInterest.GetClosestCubeNode(mVisionSubsystem.getCurrentAlliance(), mDriveTrain.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d robotToTarget = mDriveTrain.getPose().getTranslation().minus(mNearestScoringLocation).getAngle();
    Rotation2d yaw = robotToTarget.minus(mDriveTrain.getPose().getRotation());
    double robotToTargetAngle = robotToTarget.getDegrees();
    double driveTrainAngle = mDriveTrain.getPose().getRotation().getDegrees();
    double error = yaw.getDegrees();
    mDriveTrain.setDriveMode(mDriveControls.getThrottle(), error * .0125, true);
    
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
