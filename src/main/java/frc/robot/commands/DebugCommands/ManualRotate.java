package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.PivotArm;

public class ManualRotate extends CommandBase{

    private PivotArm mArm;
    private DriverControls mControls;

    public ManualRotate(PivotArm arm, DriverControls controls){
        mArm = arm;
        mControls = controls;
        addRequirements(arm, controls);
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public void execute() {
        if (mControls.d_pivotArmManual() != 0){
            mArm.setMotor(mControls.d_pivotArmManual() *12);
        }
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
