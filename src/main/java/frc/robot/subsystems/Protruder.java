package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.game.Placement;
import frc.robot.util.TelescopingRotatingArmFeedForwards;

public class Protruder extends SubsystemBase{

    private TalonSRX mInnerStageMotor, mOuterStageMotor;
    private enum InnerStates {
        HoldIn,
        Retract,
        HoldOut,
        Extend,
    };
    private enum OuterStates {
        HoldIn,
        Retract,
        HoldOut,
        Extend,
    }
    private Placement currentPlacement;
    private PIDController mPidController;
    private InnerStates mInnerStates;
    private OuterStates mOuterStates;
    private AnalogPotentiometer mPotentiometer;
    private Double encoderSnapshot;
    private DigitalInput mInnerLimitSwitch, mOuterLimitSwitch;
    
    public Protruder(){
        mInnerStageMotor = new TalonSRX(RobotMap.protruderOuterStage);
        mOuterStageMotor = new TalonSRX(RobotMap.protruderInnerStage);
        mPotentiometer = new AnalogPotentiometer(RobotMap.protrusionStringPotentiometer);
        mInnerLimitSwitch = new DigitalInput(RobotMap.protrusionLimitSwitchFront);
        mOuterLimitSwitch = new DigitalInput(RobotMap.protrusionLimitSwitchEnd);
        mInnerStates = InnerStates.Retract;
        mOuterStates = OuterStates.Retract;
        currentPlacement = Placement.Home;
        mPidController = new PIDController(Constants.kProtruderkP, Constants.kProtruderkI, Constants.kProtruderkD);
        encoderSnapshot = 0.0;
    }

    public Placement getCurrentPlacement(){
        return currentPlacement;
    }

    public void setCurrentPlacement(Placement placement){
        currentPlacement = placement;
    }

    private void goToExtension() {
        encoderSnapshot = getDistance();

        if(getCurrentPlacement().getExtendDistance() < Constants.kProtruderInnerStageLength){
            mInnerStates = InnerStates.Extend;
        } else {
            mInnerStates = InnerStates.Extend;
            mOuterStates = OuterStates.Extend;
        }
    }

    private boolean checkIfAtPosition(){
        if((getDistance() - encoderSnapshot - getCurrentPlacement().getExtendDistance()) < Constants.kProtruderAcceptableDistanceDelta){
            mInnerStates = InnerStates.HoldOut;
            mOuterStates = OuterStates.HoldOut;
            return true;
        }
        return false;
    }

    public void stop(){
        mInnerStates = InnerStates.Retract;
        mOuterStates = OuterStates.Retract;
    }

    private Double getDistance(){
        return mPotentiometer.get();
    }

    public AnalogPotentiometer getPotentiometer(){
        return mPotentiometer;
    }

    public TalonSRX getInnerStageMotor(){
        return mInnerStageMotor;
    }

    public TalonSRX getOuterStageMotor(){
        return mOuterStageMotor;
    }

    public CommandBase Extend(Placement placement){
        return this.runOnce(() -> goToExtension());
    }

    public double calculateFeedForward(){
        return TelescopingRotatingArmFeedForwards.CalculateTelescopeFeedForward(RobotContainer.S_PIVOTARM.getAngle(), Constants.kProtruderFeedFowardGain);
    }

    @Override
    public void periodic() {

        if(mInnerLimitSwitch.get()){
            mInnerStates = InnerStates.HoldOut;
        }

        if(mOuterLimitSwitch.get()){
            mOuterStates = OuterStates.HoldOut;
        }

        switch(mInnerStates){
            case Retract:
                mInnerStageMotor.set(TalonSRXControlMode.PercentOutput, -1.0);
            case HoldIn:
                mInnerStageMotor.set(TalonSRXControlMode.PercentOutput, 0.08);
            case HoldOut: 
                mInnerStageMotor.set(TalonSRXControlMode.PercentOutput, -0.08);
            case Extend:
                mInnerStageMotor.set(TalonSRXControlMode.PercentOutput, mPidController.calculate(getCurrentPlacement().getExtendDistance()));
                checkIfAtPosition();
        }

        switch(mOuterStates){
            case Retract:
                mOuterStageMotor.set(TalonSRXControlMode.PercentOutput, -1.0);
            case HoldIn:
                mOuterStageMotor.set(TalonSRXControlMode.PercentOutput, 0.08);
            case HoldOut: 
                mOuterStageMotor.set(TalonSRXControlMode.PercentOutput, -0.08);
            case Extend:
                mOuterStageMotor.set(TalonSRXControlMode.PercentOutput, mPidController.calculate(getCurrentPlacement().getExtendDistance()));
                checkIfAtPosition();
        }
    }
}
