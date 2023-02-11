package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class PivotArm extends SubsystemBase{

    private CANSparkMax mArmPivotMotor;
    private DutyCycleEncoder mArmPivotEncoder;
    private Double mInternalMotorEncoderOffset;
    private PIDController mPidController;
    private double mCurrentDesiredAngle; //the angle the rest of the robot wants this at.
    
    public PivotArm() {
        mInternalMotorEncoderOffset = 0.0;

        mArmPivotMotor = new CANSparkMax(RobotMap.armPivotMotor, MotorType.kBrushless);
        mArmPivotMotor.getEncoder().setPositionConversionFactor((1/Constants.kArmGearRatio) * 360);

        mArmPivotEncoder = new DutyCycleEncoder(RobotMap.armPivotEncoder);
        mArmPivotEncoder.setDutyCycleRange(1.0/1025.0,  1024.0/1025.0);
        mArmPivotEncoder.setDistancePerRotation(360);
        mCurrentDesiredAngle =0.0;
        mPidController = new PIDController(Constants.kPivotArmAngleKp, Constants.kPivotArmAngleKi, Constants.kPivotArmAngleKd);
    }

    public void resetMotorEncoderToAbsoluteEncoder(){
        mArmPivotMotor.getEncoder().setPosition(mArmPivotEncoder.getAbsolutePosition());
    }

    /**
     * 
     * @return
     */
    public double getAngle(){

        return mArmPivotMotor.getEncoder().getPosition();
    }

    /**
     * Tells the pivot arm to go to an angle
     * @param angle desired angle to go to, 0 degrees is straight up, 180 is straight down.
     */
    public CommandBase goToPosition(double angle){
        return runOnce(() -> mCurrentDesiredAngle = angle);
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

    @Override
    public void periodic(){

        if (atPosition()){
            //true stuff here
            mArmPivotMotor.setVoltage(0.0);
        }
        else{
            //false stuff here
            mArmPivotMotor.setVoltage(mPidController.calculate(getAngle(), mCurrentDesiredAngle) * 12);
        }

    }



}


