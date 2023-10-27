package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class rainbowLED extends CommandBase {
    private LED mLED;

    public rainbowLED(LED led) {
        mLED = led;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void initialize() {
        mLED.setRainbowAnim();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
