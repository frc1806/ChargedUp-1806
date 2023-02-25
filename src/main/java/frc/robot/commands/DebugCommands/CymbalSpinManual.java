package frc.robot.commands.DebugCommands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriverControls;

public class CymbalSpinManual extends CommandBase{

    private Claw mClaw;
    private DriverControls mControls;

    public CymbalSpinManual(Claw claw, DriverControls controls){
        mClaw = claw;
        mControls = controls;
        addRequirements(claw, controls);
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public void execute() {
        if (mControls.d_cymbalThrottle() != 0){
            mClaw.getSpinner().set(TalonSRXControlMode.PercentOutput, mControls.d_pivotArmManual());
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
