package frc.robot.commands.AutoModes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DeadReckoningNoObstacle extends CommandBase{

    private DriveTrain mDriveTrain;

    public DeadReckoningNoObstacle(DriveTrain driveTrain){
        mDriveTrain = driveTrain;
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
        mDriveTrain.followTrajectoryCommand(PathPlanner.loadPath("NoObstacleDeadReckoning", new PathConstraints(4.0, 3.0)), true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
