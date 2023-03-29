package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.drivers.BeamBreak;

public class Claw extends SubsystemBase{
    private Solenoid mLeftSolenoid, mRightSolenoid;
    private TalonSRX mCymbalSpinner;
    private CircularBuffer mCircularBuffer;
    private Double mRunningTotal;
    private DigitalInput mGPSensor0;
    private DigitalInput mGPSensor1;
    private DigitalInput mClosedLimit1;
    private DigitalInput mClosedLimit2;
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
        mLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.leftSolenoid);
        mRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.rightSolenoid);
        mIntakeStates = IntakeStates.Closed;
        mCymbalSpinner = new TalonSRX(RobotMap.clawSpinMotor);
        mCircularBuffer = new CircularBuffer(Constants.kClawSpinnerBufferSize);
        mRunningTotal = 0.0;
        //All these read backard, they are on when you would think they're off.
        mGPSensor0 = new DigitalInput(RobotMap.gPSensor0);
        mGPSensor1 = new DigitalInput(RobotMap.gPSensor1);
        mClosedLimit1 = new DigitalInput(RobotMap.clawCloseLimitLeft);
        mClosedLimit2 = new DigitalInput(RobotMap.clawCloseLimitRight);
    }

    public void openBoth(){
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

    public CommandBase rotateClaw(double power){
        return this.runOnce(() -> mCymbalSpinner.set(TalonSRXControlMode.PercentOutput, power));
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

    public Solenoid getLeftSolenoid(){
        return mLeftSolenoid;
    }

    public Solenoid getRightSolenoid(){
        return mRightSolenoid;
    }

    public boolean getGPSense(){
        return !mGPSensor0.get() || !mGPSensor1.get();
    }

    public boolean isClawClosed(){
        return !mClosedLimit1.get() && !mClosedLimit2.get();
        }

    public boolean isClawOpened(){
        return mClosedLimit1.get() && mClosedLimit2.get();
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
        updateClawRotationCurrentBuffer();
        if(Constants.clawDIOsDebug){
            logDIOs();
        }


    }

    public void reset(){
        mCircularBuffer.clear();
        mRunningTotal = 0.0;
    }

    public void logDIOs(){
        SmartDashboard.putBoolean("gpSense0", mGPSensor0.get());
        SmartDashboard.putBoolean("gpSense1", mGPSensor1.get());
        SmartDashboard.putBoolean("clawClosedLimit1", mClosedLimit1.get());
        SmartDashboard.putBoolean("clawClosedLimit2", mClosedLimit2.get());
        SmartDashboard.putBoolean("isClawClosed", isClawClosed());
        SmartDashboard.putBoolean("isClawOpen", isClawOpened());
        SmartDashboard.putBoolean("SenseAGamePiece?", getGPSense());

    }
    
}
