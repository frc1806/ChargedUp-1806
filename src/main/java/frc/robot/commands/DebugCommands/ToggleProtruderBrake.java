package frc.robot.commands.DebugCommands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protruder;

public class ToggleProtruderBrake extends CommandBase{

    private Protruder mProtruder;
    boolean isInBrakeMode;

    public ToggleProtruderBrake(Protruder protruder){
        mProtruder = protruder;
        isInBrakeMode = true;
        addRequirements(mProtruder);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        if(isInBrakeMode){
            mProtruder.getInnerStageMotor().setNeutralMode(NeutralMode.Coast);
            mProtruder.getOuterStageMotor().setNeutralMode(NeutralMode.Coast);
        } else {
            mProtruder.getInnerStageMotor().setNeutralMode(NeutralMode.Brake);
            mProtruder.getOuterStageMotor().setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
