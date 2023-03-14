package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class PivotArm extends SubsystemBase{

    private CANSparkMax mArmPivotMotor;
    private DutyCycleEncoder mArmPivotEncoder;
    private PIDController mPidController;
    private double mCurrentDesiredAngle; //the angle the rest of the robot wants this at.
    private double mCurrentAngle;
    
    public PivotArm() {

        mArmPivotMotor = new CANSparkMax(RobotMap.armPivotMotor, MotorType.kBrushless);
        mArmPivotMotor.getEncoder().setPositionConversionFactor((1/Constants.kArmGearRatio) * 360);
        mArmPivotMotor.setIdleMode(IdleMode.kBrake);
        mArmPivotMotor.setInverted(true);
        mArmPivotEncoder = new DutyCycleEncoder(RobotMap.armPivotEncoder);
        mArmPivotEncoder.setDutyCycleRange(1.0/1025.0,  1024.0/1025.0);
        mArmPivotEncoder.setDistancePerRotation(360.0);
        mCurrentDesiredAngle =180.0;
        mPidController = new PIDController(Constants.kPivotArmAngleKp, Constants.kPivotArmAngleKi, Constants.kPivotArmAngleKd);
        mCurrentAngle = mArmPivotMotor.getEncoder().getPosition();
    }

    public void resetMotorEncoderToAbsoluteEncoder(){
        double proposedNewPos = mArmPivotEncoder.getDistance();
        if(proposedNewPos < 0)
        {
            proposedNewPos +=360;
        }
        mArmPivotMotor.getEncoder().setPosition(proposedNewPos);
    }

    /**
     * 
     * @return
     */
    public double getAngle(){

        return mCurrentAngle;
    }

    /**
     * Tells the pivot arm to go to an angle
     * @param angle desired angle to go to, 0 degrees is straight up, 180 is straight down.
     * This arrangement allows us to use cosine for feedforwards.
     */
    public void goToPosition(double angle){
        
        mCurrentDesiredAngle = angle;
    }

    public boolean atPosition(){
        return Math.abs(getAngle()-mCurrentDesiredAngle) < Constants.kAcceptableAngleDelta;
    }

    public void setMotor(Double num){
        mArmPivotMotor.setVoltage(num);
    }

    public CANSparkMax getPivotMotor(){
        return mArmPivotMotor;
    }

    public DutyCycleEncoder getEncoder(){
        return mArmPivotEncoder;
    }

    public double calculateCustomArmFeedForwarad()
    {
        return 0.0; //TODO? UNIMPLEMENTED UNTIL NEEDED
        //return TelescopingRotatingArmFeedForwards.CalculateArmFeedForward(RobotContainer.S_PROTRUDER.getExtensionDistance(), getAngle(), Constants.kPivotFeedForwardGain); //UNIMPLEMENTED
    }

    @Override
    public void periodic(){
        mCurrentAngle = mArmPivotMotor.getEncoder().getPosition();
        if (atPosition()){
            //true stuff here
            setMotor(0.0);
        }
        else{
            //false stuff here
            setMotor(mPidController.calculate(getAngle(), mCurrentDesiredAngle) * 12);
        }
        if(RobotState.isDisabled())
        {
            resetMotorEncoderToAbsoluteEncoder();
        }

    }



}


