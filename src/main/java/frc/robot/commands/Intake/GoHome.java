package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.game.Placement;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Protruder;

public class GoHome extends CommandBase {

    private PivotArm mArm;
    private Protruder mProtruder;

    public GoHome(PivotArm arm, Protruder protruder){
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
        mArm.goToPosition(0);
        mProtruder.goToExtension(Placement.Home);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
