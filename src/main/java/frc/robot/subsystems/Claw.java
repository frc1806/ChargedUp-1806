package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase{

    private Solenoid mLeftSolenoid, mRightSolenoid;
    private TalonSRX mCymbalSpinner;
    public enum IntakeStates {
        Opened,
        Closed,
        LeftOpen,
        LeftClosed,
        RightOpened,
        RightClosed,
    }
    private IntakeStates mIntakeStates;

    public Claw(){
        mLeftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.leftSolenoid);
        mRightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.rightSolenoid);
        mIntakeStates = IntakeStates.Closed;
    }

    public void openBoth(){
        System.out.println("Opened");
        mIntakeStates = IntakeStates.Opened;
        mLeftSolenoid.set(true);
        mRightSolenoid.set(true);
    }

    public void openLeft(){
        mIntakeStates = IntakeStates.LeftClosed;
        mLeftSolenoid.set(true);
    }

    public void openRight(){
        mIntakeStates = IntakeStates.RightOpened;
        mRightSolenoid.set(true);
    }

    public void closeBoth(){
        mIntakeStates = IntakeStates.Closed;
        System.out.println("closed");
        mLeftSolenoid.set(false);
        mRightSolenoid.set(false);
    }

    public void closeLeft(){
        mIntakeStates = IntakeStates.LeftClosed;
        mLeftSolenoid.set(false);
    }

    public void closeRight(){
        mIntakeStates = IntakeStates.RightClosed;
        mRightSolenoid.set(false);
    }

    public IntakeStates getIntakeState(){
        return mIntakeStates;
    }

    private void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Left Solenoid Open?", mLeftSolenoid.get());
        SmartDashboard.putBoolean("Right Solenoid Open?", mRightSolenoid.get());
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
    }
    
}
