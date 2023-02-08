package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class PivotArmSubsytem extends SubsystemBase{

    private CANSparkMax mArmPivotMotor;
    private DutyCycleEncoder mArmPivotEncoder;
    public Double mInternalMotorEncoderOffset;
    
    public PivotArmSubsytem() {
        mInternalMotorEncoderOffset = 0.0;
       
        mArmPivotMotor = new CANSparkMax(RobotMap.armPivotMotor, MotorType.kBrushless);
        mArmPivotMotor.getEncoder().setPositionConversionFactor((1/Constants.kArmGearRatio) * 360);

        mArmPivotEncoder = new DutyCycleEncoder(RobotMap.armPivotEncoder);
        mArmPivotEncoder.setDutyCycleRange(1.0/1025.0,  1024.0/1025.0);
        mArmPivotEncoder.setDistancePerRotation(360);
    }

    public void resetMotorEncoderToAbsoluteEncoder(){
        mArmPivotMotor.getEncoder().setPosition(mArmPivotEncoder.getAbsolutePosition());
    }

    public double getAngle(){

        return mArmPivotMotor.getEncoder().getPosition();
    }

    public void goToPosition(){
        
    }



}


