package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protruder;

public class Extend extends CommandBase {

    private Protruder mProtruder;

    public Extend(Protruder protruder){
        mProtruder = protruder;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mProtruder.goToExtension(mProtruder.getCurrentPlacement());
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return mProtruder.checkIfAtPosition(mProtruder.getCurrentPlacement());
    }
    
}
