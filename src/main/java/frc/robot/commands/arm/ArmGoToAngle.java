package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmGoToAngle extends CommandBase{
    double wantedAngle;

    public ArmGoToAngle(double wantedAngle){
        this.wantedAngle = wantedAngle;
        addRequirements(RobotContainer.S_PIVOTARM);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        RobotContainer.S_PIVOTARM.goToPosition(wantedAngle);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.S_PIVOTARM.atPosition();
    }
    
}
