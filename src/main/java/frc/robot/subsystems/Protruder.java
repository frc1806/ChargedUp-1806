package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private enum ProtruderStates {
        Idle,
        Extending,
        Calculating
    };
    private Placement currentPlacement;
    private PIDController mPidController;
    private ProtruderStates mProtruderStates;
    private AnalogPotentiometer mPotentiometer;
    private Double encoderSnapshot;
    private DigitalInput mFrontLimitSwitch, mEndLimitSwitch;
    
    public Protruder(){
        mInnerStageMotor = new TalonSRX(RobotMap.protruderOuterStage);
        mOuterStageMotor = new TalonSRX(RobotMap.protruderInnerStage);
        mPotentiometer = new AnalogPotentiometer(RobotMap.protrusionStringPotentiometer);
        mFrontLimitSwitch = new DigitalInput(RobotMap.protrusionLimitSwitchFront);
        mEndLimitSwitch = new DigitalInput(RobotMap.protrusionLimitSwitchEnd);
        mProtruderStates = ProtruderStates.Idle;
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

    private void goToExtension(Placement placement) {
        mProtruderStates = ProtruderStates.Extending;
        encoderSnapshot = getDistance();
        setMotors(placement.getExtendDistance());
        mProtruderStates = ProtruderStates.Calculating;
    }

    private boolean checkIfAtPosition(Placement placement){
        if((getDistance() - encoderSnapshot - placement.getExtendDistance()) < Constants.kProtruderAcceptableDistanceDelta){
            return true;
        }
        return false;
    }

    private void setMotors(Double number){
        mProtruderStates = ProtruderStates.Extending;
        mInnerStageMotor.set(TalonSRXControlMode.PercentOutput,mPidController.calculate(number));
        mOuterStageMotor.set(TalonSRXControlMode.PercentOutput,mPidController.calculate(number));
    }

    public void stop(){
        mProtruderStates = ProtruderStates.Idle;
        mInnerStageMotor.setNeutralMode(NeutralMode.Brake);
        mOuterStageMotor.setNeutralMode(NeutralMode.Brake);
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
        return this.runOnce(() -> goToExtension(getCurrentPlacement()));
    }

    public double calculateFeedForward(){
        return TelescopingRotatingArmFeedForwards.CalculateTelescopeFeedForward(RobotContainer.S_PIVOTARM.getAngle(), Constants.kProtruderFeedFowardGain);
    }

    @Override
    public void periodic() {
        if(mProtruderStates == ProtruderStates.Calculating){
            if (checkIfAtPosition(currentPlacement)){
                mProtruderStates = ProtruderStates.Idle;
                stop();
            }
        }
    }
}
