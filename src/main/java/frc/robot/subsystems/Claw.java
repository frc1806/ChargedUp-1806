package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase{
    private Solenoid mLeftSolenoid, mRightSolenoid;
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

    public boolean canClawOpen(){
        return isAngleSafeForClawOpen(RobotContainer.S_PIVOTARM.getAngle());

    }

    public boolean isAngleSafeForClawOpen(double angle){
        return !(angle >= Constants.kClawNotAcceptableMinimumDegree
             && angle <= Constants.kClawNotAcceptableMaximumDegree);
    }



    public IntakeStates getIntakeState(){
        return mIntakeStates;
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


    @Override
    public void periodic() {
        
        if(Constants.clawDIOsDebug){
            logDIOs();
        }


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
