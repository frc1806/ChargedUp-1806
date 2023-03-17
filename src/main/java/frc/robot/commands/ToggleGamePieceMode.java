package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;

public class ToggleGamePieceMode extends CommandBase{

    public ToggleGamePieceMode(){
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        switch(RobotContainer.GetCurrentGamePieceMode())
        {
            default:
            case OffMode:
            case ConeMode:
                RobotContainer.SetCurrentGamePieceMode(GamePieceMode.CubeMode);
                break;
            case CubeMode:
                RobotContainer.SetCurrentGamePieceMode(GamePieceMode.ConeMode);
                break;
            
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
