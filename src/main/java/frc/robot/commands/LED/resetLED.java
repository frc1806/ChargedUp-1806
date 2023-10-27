package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.LED;

public class resetLED extends CommandBase {

    private LED mLED;
    public resetLED(LED led){
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
        if(RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode){
            mLED.setConeAnim();
        } else if (RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode){
            mLED.setCubeAnim();
        } else {
            mLED.setIdleAnimation();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
