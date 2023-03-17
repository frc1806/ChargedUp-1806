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
import frc.robot.subsystems.Claw;

public class GoToPlacement extends CommandBase{

    Placement desiredPlacement;
    Command m_command;
    Boolean mWasInterupted;
    private Claw mClaw;

    public GoToPlacement(Placement placement, Claw claw){
        desiredPlacement = placement;
        mWasInterupted = false;
        mClaw = claw;
        addRequirements(claw);
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
        System.out.println("Init Running for GoToPlacement");
        mClaw.closeBoth();
        RobotContainer.SetCurrentPlacement(desiredPlacement);
        if(checkIfNeedRetractionFirst())
        {
            m_command = new ProtruderGoToExtension(Constants.kProtruderDistanceAtFullRetract)
            .andThen(new ArmGoToAngle(desiredPlacement.getPivotAngle())
            .andThen(new ProtruderGoToExtension(desiredPlacement.getExtendDistance())));
        }
        else
        {
            m_command = new ParallelCommandGroup(
                new ArmGoToAngle(desiredPlacement.getPivotAngle()), 
                new ProtruderGoToExtension(desiredPlacement.getExtendDistance()));
            
        }
        // TODO Auto-generated method stub

        CommandScheduler.getInstance().schedule(m_command);
    }

    @Override
    public boolean isFinished() {
        if(RobotContainer.GetCurrentPlacement() != desiredPlacement){
            return true; //SOME OTHER COMMAND HAS RUN
        }

        if(mWasInterupted){
            return true;
        }
        return Math.abs(desiredPlacement.getPivotAngle() - RobotContainer.S_PIVOTARM.getAngle()) < Constants.kAcceptableAngleDelta
            && ((RobotContainer.S_PROTRUDER.checkIfAtPosition() && RobotContainer.S_PROTRUDER.getTargetDistance() == desiredPlacement.getExtendDistance()) || !Constants.isArmWiringPresent);
    }

    @Override
    public void end(boolean wasInterrupted){
        System.out.println("Go to placement ended. " + (wasInterrupted?"Interrupted":"Not Interrupted"));
        mWasInterupted = wasInterrupted;
        m_command.end(wasInterrupted);
    }
    

    private boolean checkIfNeedRetractionFirst(){
        if(!Constants.isArmWiringPresent) return false;
        
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
