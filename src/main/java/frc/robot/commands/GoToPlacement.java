package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmGoToAngle;
import frc.robot.commands.protruder.ProtruderGoToExtension;
import frc.robot.game.Placement;

public class GoToPlacement extends CommandBase{

    Placement desiredPlacement;

    public GoToPlacement(Placement placement){
        desiredPlacement = placement;
    }
    @Override
    public void cancel() {
        // TODO Auto-generated method stub
        super.cancel();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void initialize() {
        RobotContainer.SetCurrentPlacement(desiredPlacement);
        if(checkIfNeedRetractionFirst())
        {
            CommandScheduler.getInstance().schedule(new ProtruderGoToExtension(Constants.kProtruderDistanceAtFullRetract)
                .andThen(new ArmGoToAngle(desiredPlacement.getPivotAngle())
                .andThen(new ProtruderGoToExtension(desiredPlacement.getExtendDistance()))));
        }
        else
        {
            CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                new ArmGoToAngle(desiredPlacement.getPivotAngle()), 
                new ProtruderGoToExtension(desiredPlacement.getExtendDistance())));
        }
        
        RobotContainer.S_PIVOTARM.goToPosition(RobotContainer.GetCurrentPlacement().getPivotAngle());
        // TODO Auto-generated method stub

        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(desiredPlacement.getPivotAngle() - RobotContainer.S_PIVOTARM.getAngle()) < Constants.kAcceptableAngleDelta
            && RobotContainer.S_PROTRUDER.checkIfAtPosition() && RobotContainer.S_PROTRUDER.getTargetDistance() == desiredPlacement.getExtendDistance();
    }
    

    private boolean checkIfNeedRetractionFirst(){
        double currentAngle = RobotContainer.S_PIVOTARM.getAngle();
        if(desiredPlacement.getPivotAngle() > 230.0 && currentAngle > 230.0){
            return false;
        }
        if(desiredPlacement.getPivotAngle() < 130.0 && currentAngle < 130.0){
            return false;
        }
        return true;

    }

}
