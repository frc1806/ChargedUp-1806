package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.TwoLEDSubsytem;

public class ToggleGamePieceMode extends CommandBase{

    private TwoLEDSubsytem mLED;
    private DriverControls mControls;

    public ToggleGamePieceMode(TwoLEDSubsytem led, DriverControls controls){
        mLED = led;
        mControls = controls;
        addRequirements(mLED, mControls);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        if(mControls.isConeMode == true){
            mLED.setCubeAnim();
            mControls.isConeMode = false;
        } else {
            mLED.setConeAnim();
            mControls.isConeMode = true;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
