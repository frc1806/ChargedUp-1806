package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.drivers.BeamBreak;

public class Claw extends SubsystemBase{
    private Solenoid mLeftSolenoid, mRightSolenoid;
    private TalonSRX mCymbalSpinner;
    private BeamBreak mBeamBreak;
    private CircularBuffer mCircularBuffer;
    private Double mRunningTotal;
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
        mCymbalSpinner = new TalonSRX(RobotMap.clawSpinMotor);
        mBeamBreak = new BeamBreak(RobotMap.clawBeamBreak);
        mCircularBuffer = new CircularBuffer(Constants.kClawSpinnerBufferSize);
        mRunningTotal = 0.0;
    }

    public void openBoth(){
        System.out.println("Opened");
        mIntakeStates = IntakeStates.Opened;
        mLeftSolenoid.set(true);
        mRightSolenoid.set(true);
    }

    public void openLeft(){
        mIntakeStates = IntakeStates.LeftOpen;
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

    public boolean isBeamBreakTripped(){
        return mBeamBreak.get(); //May have to ! this, if it's normally closed.
    }

    public void rotateClaw(double power){
        mCymbalSpinner.set(TalonSRXControlMode.PercentOutput, power);
    }

    public boolean isClawSpinnerStalled(){
        return (mRunningTotal / mCircularBuffer.size()) > Constants.kClawSpinnerStalledCurrent;
    }



    public IntakeStates getIntakeState(){
        return mIntakeStates;
    }

    public TalonSRX getSpinner(){
        return mCymbalSpinner;
    }

    public BeamBreak getBeamBreak(){
        return mBeamBreak;
    }

    public Solenoid getLeftSolenoid(){
        return mLeftSolenoid;
    }

    public Solenoid getRightSolenoid(){
        return mRightSolenoid;
    }

    private void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Left Solenoid Open?", mLeftSolenoid.get());
        SmartDashboard.putBoolean("Right Solenoid Open?", mRightSolenoid.get());
    }
    
    private void updateClawRotationCurrentBuffer(){
        if(mCircularBuffer.size() == Constants.kClawSpinnerBufferSize)
        {
            mRunningTotal -= mCircularBuffer.getFirst();
        }
        double spinnerCurrent = mCymbalSpinner.getStatorCurrent();
        mRunningTotal += spinnerCurrent;
        mCircularBuffer.addLast(spinnerCurrent);
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
        updateClawRotationCurrentBuffer();
    }
    
}
