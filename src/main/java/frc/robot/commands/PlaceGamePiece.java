package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Protruder;

public class PlaceGamePiece extends CommandBase{

    private Protruder mProtruder;
    private PivotArm mArm;

    public PlaceGamePiece(Protruder protruder, PivotArm arm){
        mArm = arm;
        mProtruder = protruder;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void initialize() {
        mProtruder.Extend();
        mArm.goToPosition(mProtruder.getCurrentPlacement().getPivotAngle());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
