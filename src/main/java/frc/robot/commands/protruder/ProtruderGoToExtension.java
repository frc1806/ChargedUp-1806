package frc.robot.commands.protruder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ProtruderGoToExtension extends CommandBase{

    double wantedExtension;

    public ProtruderGoToExtension(double wantedExtension){
        this.wantedExtension = wantedExtension;
        addRequirements(RobotContainer.S_PROTRUDER);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub

        super.execute();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        RobotContainer.S_PROTRUDER.Extend(wantedExtension);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return RobotContainer.S_PROTRUDER.checkIfAtPosition() || !Constants.isArmWiringPresent;
        }
    
}
