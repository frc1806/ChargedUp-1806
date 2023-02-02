package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase{

    private Solenoid mLeftSolenoid, mRightSolenoid;

    public Intake(){
        mLeftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.leftSolenoid);
        mRightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.rightSolenoid);
    }

    public void openBoth(){
        mLeftSolenoid.set(true);
        mRightSolenoid.set(true);
    }

    public void openLeft(){
        mLeftSolenoid.set(true);
    }

    public void openRight(){
        mRightSolenoid.set(true);
    }

    public void closeBoth(){
        mLeftSolenoid.set(false);
        mRightSolenoid.set(false);
    }

    public void closeLeft(){
        mLeftSolenoid.set(false);
    }

    public void closeRight(){
        mRightSolenoid.set(false);
    }

    private void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Left Solenoid Open?", mLeftSolenoid.isDisabled());
        SmartDashboard.putBoolean("Right Solenoid Open?", mRightSolenoid.isDisabled());
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
    }
    
}
