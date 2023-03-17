package frc.robot.shuffleboard.tabs;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotContainer;
import frc.robot.drivers.BeamBreak;
import frc.robot.drivers.StringPotentiometer;
import frc.robot.game.Placement;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Protruder;

public class ArmTab extends ShuffleboardTabBase {
    private PivotArm mPivotArm;
    private Claw mClaw;
    private Protruder mProtruder;
    private CANSparkMax mPivotArmMotor;
    private DutyCycleEncoder mPivotArmEncoder;
    private TalonSRX mProtruderMotorA, mProtrusionMotorB, mCymbalSpinner;
    private StringPotentiometer mPotentiometer;
    private BeamBreak mBeamBreak;
    private Solenoid mLeftSolenoid, mRightSolenoid;
    private Placement currentPlacement;

    private GenericEntry leftSolenoidOn, rightSolenoidOn, beamTripped;
    private GenericEntry ProtruderDistance, PotentiometerRawVoltage, ProtruderOutputA, ProtruderOutputB, ProtruderFirstAmps, ProtruderSecondAmps;
    private GenericEntry SpinnerOutput, SpinnerTemp;
    private GenericEntry PivotDistance, PivotOutput, PivotAmps, PivotTemp, PivotAngle, PivotAngleRelative;
    private GenericEntry placement;

    public ArmTab(){
        mPivotArm = RobotContainer.S_PIVOTARM;
        mClaw = RobotContainer.S_INTAKE;
        mProtruder = RobotContainer.S_PROTRUDER;
        mPivotArmMotor = mPivotArm.getPivotMotor();
        mPivotArmEncoder = mPivotArm.getEncoder();
        mProtruderMotorA = mProtruder.getInnerStageMotor();
        mProtrusionMotorB = mProtruder.getOuterStageMotor();
        mCymbalSpinner = mClaw.getSpinner();
        mPotentiometer = mProtruder.getPotentiometer();
        mBeamBreak = mClaw.getBeamBreak();
        mLeftSolenoid = mClaw.getLeftSolenoid();
        mRightSolenoid = mClaw.getRightSolenoid();
        currentPlacement = RobotContainer.GetCurrentPlacement();

    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Arm");

        leftSolenoidOn = mTab.add("Left Solenoid", mLeftSolenoid.get())
            .withPosition(0,0)
            .withSize(1,1)
            .getEntry();
        
        beamTripped = mTab.add("Beam Tripped", mBeamBreak.get())
            .withPosition(4,0)
            .withSize(1,1)
            .getEntry();
        
        rightSolenoidOn = mTab.add("Right Solenoid", mRightSolenoid.get())
            .withPosition(8,0)
            .withSize(1,1)
            .getEntry();
        
        ProtruderOutputA = mTab.add("Inner Stage Output", mProtruderMotorA.getMotorOutputPercent())
            .withPosition(0,1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        ProtruderOutputB = mTab.add("Outer Stage Output", mProtrusionMotorB.getMotorOutputPercent())
            .withPosition(0,2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        ProtruderDistance = mTab.add("Protruder First Stage Distance", mPotentiometer.getExtensionInInches())
            .withPosition(0,3)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        ProtruderFirstAmps = mTab.add("First Stage Amps", mProtruderMotorA.getStatorCurrent())
            .withPosition(3,2)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        ProtruderSecondAmps = mTab.add("Second Stage Amps", mProtrusionMotorB.getStatorCurrent())
            .withPosition(4,2)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        PotentiometerRawVoltage = mTab.add("String Potentiometer Raw Voltage", mPotentiometer.getRawVoltage())
            .withPosition(3, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        placement = mTab.add("Current Placement", currentPlacement.getPlacementName())
            .withPosition(0,4)
            .withSize(2,1)
            .getEntry();
        
        SpinnerOutput = mTab.add("Spinner Output", mCymbalSpinner.getMotorOutputPercent())
            .withPosition(4,1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        SpinnerTemp = mTab.add("Spinner Temp", mCymbalSpinner.getTemperature())
            .withPosition(4,2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        PivotOutput = mTab.add("Pivot Output", mPivotArmMotor.getAppliedOutput())
            .withPosition(8,1)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        PivotAmps = mTab.add("Pivot Amps", mPivotArmMotor.getOutputCurrent())
            .withPosition(8,2)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        PivotTemp = mTab.add("Pivot Temp", mPivotArmMotor.getMotorTemperature())
            .withPosition(9,1)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        PivotAngle = mTab.add("Pivot Angle Absolute", mPivotArmEncoder.getDistance())
            .withPosition(9,2)
            .withSize(1,1)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();

        PivotAngleRelative = mTab.add("Pivot Angle Relative", mPivotArm.getAngle())
        .withPosition(8,3)
        .withSize(1,1)
        .withWidget(BuiltInWidgets.kDial)
        .getEntry();

        
        
        PivotDistance = mTab.add("Protruder Distance", mProtruder.getDistance())
            .withPosition(9,4)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
    }

    @Override
    public void update() {
        leftSolenoidOn.setBoolean(mLeftSolenoid.get());
        beamTripped.setBoolean(mBeamBreak.get());
        rightSolenoidOn.setBoolean(mRightSolenoid.get());
        ProtruderOutputA.setDouble(mProtruderMotorA.getMotorOutputPercent());
        ProtruderOutputB.setDouble(mProtrusionMotorB.getMotorOutputPercent());
        ProtruderDistance.setDouble(mPotentiometer.getExtensionInInches());
        SpinnerOutput.setDouble(mCymbalSpinner.getMotorOutputPercent());
        SpinnerTemp.setDouble(mCymbalSpinner.getTemperature());
        PivotOutput.setDouble(mPivotArmMotor.getAppliedOutput()) ;
        PivotAngle.setDouble(mPivotArmEncoder.getDistance());
        PivotAngleRelative.setDouble(mPivotArmEncoder.getDistance());
        PivotAmps.setDouble(mPivotArmMotor.getOutputCurrent());
        PivotTemp.setDouble(mPivotArmMotor.getMotorTemperature());
        PivotDistance.setDouble(mProtruder.getDistance());
        placement.setString(RobotContainer.GetCurrentPlacement().getPlacementName());
        PotentiometerRawVoltage.setDouble(mPotentiometer.getRawVoltage());
        ProtruderFirstAmps.setDouble(mProtruderMotorA.getStatorCurrent());
        ProtruderSecondAmps.setDouble(mProtrusionMotorB.getStatorCurrent());
    }
    
}
