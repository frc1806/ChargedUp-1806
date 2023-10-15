package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.game.PosesOfInterest;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Vision;
import frc.robot.util.TrapezoidalDiffDriveTurnPID;

public class ScoringLineupDrive extends CommandBase{
    private DriveTrain mDriveTrain;
    private DriverControls mDriveControls;
    private Vision mVisionSubsystem;
    private Translation2d mNearestScoringLocation;
    private TrapezoidalDiffDriveTurnPID trapezoidalDiffDriveTurnPID;

    public ScoringLineupDrive(DriveTrain drivetrain, DriverControls driveControls, Vision visionSubsystem) {
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

    //TODO: When we add robot modes for cubes and cones, make this select  what to lineup on based on that.
    switch(RobotContainer.GetCurrentGamePieceMode()){
      case ConeMode:
        mNearestScoringLocation = PosesOfInterest.GetClosestConeNode(mVisionSubsystem.getCurrentAlliance(), mDriveTrain.getPose());
        break;
        case OffMode:
      default:
      case CubeMode:
        mNearestScoringLocation = PosesOfInterest.GetClosestCubeNode(mVisionSubsystem.getCurrentAlliance(), mDriveTrain.getPose());
        break;

      
    }

    trapezoidalDiffDriveTurnPID.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d averagedXPointToTarget = new Translation2d( (.666* mNearestScoringLocation.getX()) + (.333*mDriveTrain.getPose().getTranslation().getX()), mNearestScoringLocation.getY() );
    Rotation2d robotToTarget = mDriveTrain.getPose().getTranslation().minus(averagedXPointToTarget).getAngle();
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
