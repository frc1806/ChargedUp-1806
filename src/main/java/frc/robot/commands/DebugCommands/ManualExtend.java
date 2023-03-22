package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Protruder;

public class ManualExtend extends CommandBase{

    private Protruder mProtruder;
    private DriverControls mControls;

    public ManualExtend(Protruder protruder, DriverControls controls){
        mProtruder = protruder;
        mControls = controls;
        addRequirements(protruder, controls);
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public void execute() {
        if (mControls.d_pivotArmManual() != 0){
            mProtruder.Extend(Math.abs((Constants.kProtruderFirstStageExtension + Constants.kProtruderSecondStageLength) * mControls.d_extendThrottle()));
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
