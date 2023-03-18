package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.drivers.StringPotentiometer;
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
        RampToRetractStop,
        HoldOut,
        Extend,
        RampToExtensionStop,
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
    
    public Protruder(){
        mFirstStageMotor = new TalonSRX(RobotMap.protruderFirstStage);
        mSecondStageMotor = new TalonSRX(RobotMap.protruderSecondStage);
        mFirstStageMotor.setNeutralMode(NeutralMode.Brake);
        mSecondStageMotor.setNeutralMode(NeutralMode.Brake);
        mPotentiometer = new StringPotentiometer(RobotMap.protrusionStringPotentiometer);
        mSecondStageRetractLimit = new DigitalInput(RobotMap.protrusionLimitSwitchFront);
        mSecondStageExtendLimit = new DigitalInput(RobotMap.protrusionLimitSwitchEnd);
        mFirstStageStates = FirstStageStates.GoingToPosition;
        mSecondStageStates = SecondStageStates.RampToRetractStop;
        mTargetTotalDistance = Constants.kProtruderDistanceAtFullRetract;
        mPidController = new PIDController(Constants.kProtruderkP, Constants.kProtruderkI, Constants.kProtruderkD);
        mSecondStageAmpsCircularBuffer = new CircularBuffer(15);
        mFirstStageMotor.configPeakCurrentLimit(40);
        mSecondStageMotor.configPeakCurrentLimit(40);
        mSecondStageMotor.follow(mFirstStageMotor);
        mFirstStageMotor.setInverted(true);
        mSecondStageMotor.setInverted(false);

    }

    private void goToExtension() {
        //Protect against someone asking for an extension of 0.
        mPidController.reset();
        double projectedInnerExtensionDistance = Math.max(mTargetTotalDistance - Constants.kProtruderDistanceAtFullRetract, 0.0);
        mFirstStageStates = FirstStageStates.GoingToPosition;
        mFirstStageTargetDistance = projectedInnerExtensionDistance;
    }

    public boolean checkIfAtPosition(){
        return (isFirstStatgeAtPosition() || ! Constants.isArmWiringPresent);
    }

    public void stop(){
        mFirstStageStates = FirstStageStates.Idle;
        mSecondStageStates = SecondStageStates.Disabled;
    }

    public Double getDistance(){
        return getFirstStageDistance();
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

    public void Extend(Double distance){
        mTargetTotalDistance = distance;
        goToExtension();
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

    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Limit Extend Tripped", mSecondStageExtendLimit.get());
        SmartDashboard.putBoolean("Limit Retract Tripped", mSecondStageRetractLimit.get());
        SmartDashboard.putString("First Stage States", mFirstStageStates.name());
        SmartDashboard.putString("Second Stage States", mSecondStageStates.name());
        SmartDashboard.putNumber("First Stage Target Distance", mFirstStageTargetDistance);
    }

    @Override
    public void periodic() {
        
        outputToSmartDashboard();
        if(Constants.isArmWiringPresent){
            switch(mFirstStageStates){
                case AtPosition:
                    if(mFirstStageTargetDistance < 1.0)
                    {
                        mFirstStageMotor.set(ControlMode.PercentOutput, -0.07);
                    }
                    else
                    {
                        mFirstStageMotor.set(ControlMode.PercentOutput, calculateFeedForward());
                        if(!isFirstStatgeAtPosition()) mFirstStageStates = FirstStageStates.GoingToPosition;  
                    }
                    if(!isFirstStatgeAtPosition())
                    {
                        mFirstStageStates = FirstStageStates.GoingToPosition;
                    }
                   
                    break;
                case GoingToPosition:
                    mFirstStageMotor.set(ControlMode.PercentOutput, mPidController.calculate(getFirstStageDistance(), mFirstStageTargetDistance) );//+ calculateFeedForward());
                    if(isFirstStatgeAtPosition()) mFirstStageStates = FirstStageStates.AtPosition;
                    break;
                default:
                case Idle:
                    mFirstStageMotor.set(ControlMode.PercentOutput, 0.0);
                    break;
            }
        }
    }
}
