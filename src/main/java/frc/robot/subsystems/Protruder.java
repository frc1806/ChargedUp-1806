package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.drivers.StringPotentiometer;
import frc.robot.game.Placement;
import frc.robot.util.TelescopingRotatingArmFeedForwards;

public class Protruder extends SubsystemBase{

    private TalonSRX mFirstStageMotor, mSecondStageMotor;
    private enum FirstStageStates {
        Idle,
        GoingToPosition,
        AtPosition
    };
    private enum SecondStageStates {
        HoldIn,
        Retract,
        HoldOut,
        Extend,
        Disabled
    }
    private double  mTargetTotalDistance;
    private PIDController mPidController;
    private FirstStageStates mFirstStageStates;
    private SecondStageStates mSecondStageStates;
    private StringPotentiometer mPotentiometer;
    private DigitalInput mSecondStageRetractLimit, mSecondStageExtendLimit;
    private double mFirstStageTargetDistance;
    private CircularBuffer mSecondStageAmpsCircularBuffer;
    private double mSecondStageCurrentRunningTotal = 0.0;
    private double mSecondStageRetractLimitSwitchActiveTime = 0.0;
    private double mSecondStageExtendLimitSwitchActiveTime = 0.0;

    private boolean mWasSecondStageRetractLimitSwitchOn = false;
    private boolean mWasSecondStageExtendLimitSwitchOn = false;
    
    public Protruder(){
        mFirstStageMotor = new TalonSRX(RobotMap.protruderOuterStage);
        mSecondStageMotor = new TalonSRX(RobotMap.protruderInnerStage);
        mFirstStageMotor.setNeutralMode(NeutralMode.Brake);
        mSecondStageMotor.setNeutralMode(NeutralMode.Brake);
        mPotentiometer = new StringPotentiometer(RobotMap.protrusionStringPotentiometer);
        mSecondStageRetractLimit = new DigitalInput(RobotMap.protrusionLimitSwitchFront);
        mSecondStageExtendLimit = new DigitalInput(RobotMap.protrusionLimitSwitchEnd);
        mFirstStageStates = FirstStageStates.Idle;
        mSecondStageStates = SecondStageStates.Retract;
        mTargetTotalDistance = 0.0;
        mPidController = new PIDController(Constants.kProtruderkP, Constants.kProtruderkI, Constants.kProtruderkD);
        mSecondStageAmpsCircularBuffer = new CircularBuffer(15);

    }

    private void goToExtension() {
        //Protect against someone asking for an extension of 0.
        double projectedInnerExtensionDistance = Math.max(mTargetTotalDistance - Constants.kProtruderDistanceAtFullRetract, 0.0);

        if(projectedInnerExtensionDistance < Constants.kProtruderFirstStageExtension){
            mFirstStageTargetDistance = projectedInnerExtensionDistance;
            mFirstStageStates = FirstStageStates.GoingToPosition;

        } else {
            //THERE IS DEAD SPOT WHERE A FEW DISTANCES WILL JUST BE THE OUTER STAGE AND NO INNER STAGE AND THE ARM WILL EXTEND ONLY TO THE OUTER PROTRUDER LENGTH INSTEAD OF THE ASKED FOR DISTANCE
            //Sorry, I guess  ¯\_(ツ)_/¯
            mFirstStageTargetDistance = Math.max(projectedInnerExtensionDistance - Constants.kProtruderSecondStageLength, 0.0);

            mFirstStageStates = FirstStageStates.GoingToPosition;
            mSecondStageStates = SecondStageStates.Extend;
        }
    }

    public boolean checkIfAtPosition(){
        return (isFirstStatgeAtPosition() && isSecondStageAtPosition()) || ! Constants.isArmWiringPresent;
    }

    public void stop(){
        mFirstStageStates = FirstStageStates.Idle;
        mSecondStageStates = SecondStageStates.Disabled;
    }

    public Double getDistance(){
        return Constants.kProtruderDistanceAtFullRetract + getFirstStageDistance() + getSecondStageDistance();
    }

    public Double getTargetDistance(){
        return mTargetTotalDistance;
    }

    private Double getFirstStageDistance(){
        return mPotentiometer.getExtensionInInches(); //TODO: May be some trig here.
    }

    private Double getSecondStageDistance(){
        return mSecondStageStates == SecondStageStates.HoldIn?0:Constants.kProtruderSecondStageLength;
    }

    private boolean isSecondStageAtPosition(){
        return mSecondStageStates == SecondStageStates.HoldOut || mSecondStageStates == SecondStageStates.HoldIn;
    }

    private boolean isFirstStatgeAtPosition(){
        return Math.abs(getFirstStageDistance() - mFirstStageTargetDistance) < Constants.kProtruderAcceptableDistanceDelta;
    }

    public StringPotentiometer getPotentiometer(){
        return mPotentiometer;
    }

    public TalonSRX getInnerStageMotor(){
        return mFirstStageMotor;
    }

    public TalonSRX getOuterStageMotor(){
        return mSecondStageMotor;
    }

    public CommandBase Extend(Double distance){
        mTargetTotalDistance = distance;
        return this.runOnce(() -> goToExtension());
    }

    public double calculateFeedForward(){
        return TelescopingRotatingArmFeedForwards.CalculateTelescopeFeedForward(RobotContainer.S_PIVOTARM.getAngle(), Constants.kProtruderFeedFowardGain);
    }

    public boolean isSecondStageStalled(){
       return  mSecondStageCurrentRunningTotal / mSecondStageAmpsCircularBuffer.size() > Constants.kProtruderSecondStageStallAmps;
    }

    public boolean isSecondStageRetractTimedOut(){
        return Timer.getFPGATimestamp() > mSecondStageRetractLimitSwitchActiveTime + Constants.kProtruderSecondStageStallTimeout;
    }

    public boolean isSecondStageExtendTimedOut(){
        return Timer.getFPGATimestamp() > mSecondStageExtendLimitSwitchActiveTime + Constants.kProtruderSecondStageStallTimeout;
    }

    public boolean isSafeToPassThroughRobot(){
        return !Constants.isArmWiringPresent || (mSecondStageRetractLimit.get() && getFirstStageDistance() < Constants.kProtruderAcceptableFirstStageExtensionToPassThrough);
    }

    @Override
    public void periodic() {
        if(Constants.isArmWiringPresent){
            mSecondStageCurrentRunningTotal -= mSecondStageAmpsCircularBuffer.getFirst();
            mSecondStageCurrentRunningTotal += mSecondStageMotor.getStatorCurrent();
            mSecondStageAmpsCircularBuffer.addLast(mSecondStageMotor.getStatorCurrent());

            if(!mWasSecondStageRetractLimitSwitchOn && mSecondStageRetractLimit.get()){
                mSecondStageRetractLimitSwitchActiveTime = Timer.getFPGATimestamp();
            }
            mWasSecondStageRetractLimitSwitchOn = mSecondStageRetractLimit.get();
            if(!mWasSecondStageExtendLimitSwitchOn && mSecondStageExtendLimit.get()){
                mSecondStageExtendLimitSwitchActiveTime = Timer.getFPGATimestamp();
            }
            mWasSecondStageExtendLimitSwitchOn = mSecondStageExtendLimit.get();

            switch(mFirstStageStates){
                case AtPosition:
                    mFirstStageMotor.set(ControlMode.PercentOutput, calculateFeedForward());
                    break;
                case GoingToPosition:
                    mFirstStageMotor.set(ControlMode.PercentOutput, mPidController.calculate(getFirstStageDistance(), mFirstStageTargetDistance) + calculateFeedForward());
                    if(isFirstStatgeAtPosition()) mFirstStageStates = FirstStageStates.AtPosition;
                    break;
                default:
                case Idle:
                    mFirstStageMotor.set(ControlMode.PercentOutput, 0.0);
                    break;
            }

            switch(mSecondStageStates){
                case Retract:
                    if(mSecondStageRetractLimit.get())
                    {
                        mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, -0.25);
                        if(isSecondStageStalled() || isSecondStageRetractTimedOut()){
                            mSecondStageStates = SecondStageStates.HoldIn;
                        }
                    }
                    else{
                        mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, -1.0);
                    }
                break;
                case HoldIn:
                    mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, -0.08);
                break;
                case HoldOut: 
                    mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, 0.08);
                break;
                case Extend:
                    if(mSecondStageExtendLimit.get())
                    {
                        mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, 0.25);
                        if(isSecondStageStalled() || isSecondStageExtendTimedOut()){
                            mSecondStageStates = SecondStageStates.HoldOut;
                        }
                    }
                    else{
                        mSecondStageMotor.set(TalonSRXControlMode.PercentOutput, 1.0);
                    }
                break;
                default:
                case Disabled:
                    mSecondStageMotor.set(ControlMode.PercentOutput, 0);
                    break;
            }
        }
    }
}
