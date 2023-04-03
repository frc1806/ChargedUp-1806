// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class CymbalSpinner extends SubsystemBase {

  private TalonSRX mCymbalSpinner;
  private Double mCymbalSpinnerWantedOutput;
  private CircularBuffer mCircularBuffer;
  private Double mRunningTotal;

  private Double mClawCurrentAmps;


  /** Creates a new CymbalSpinner. */
  public CymbalSpinner() {
    mCymbalSpinner = new TalonSRX(RobotMap.clawSpinMotor);
    mCymbalSpinnerWantedOutput = 0.0;
    mCircularBuffer = new CircularBuffer(Constants.kClawSpinnerBufferSize);
    mRunningTotal = 0.0;
    mClawCurrentAmps = 0.0;
  }

  public void rotateClaw(double power){
    mCymbalSpinnerWantedOutput = power;
   
}

private void updateClawRotationCurrentBuffer(){
  if(mCircularBuffer.size() == Constants.kClawSpinnerBufferSize)
  {
      mRunningTotal -= mCircularBuffer.getFirst();
  }
  double spinnerCurrent = mCymbalSpinner.getStatorCurrent();
  mClawCurrentAmps = spinnerCurrent;
  mRunningTotal += spinnerCurrent;
  mCircularBuffer.addLast(spinnerCurrent);
}

public boolean isClawSpinnerStalled(){
  return  mClawCurrentAmps > Constants.kClawSpinnerStalledCurrent;
}

public TalonSRX getSpinner(){
  return mCymbalSpinner;
}

  @Override
  public void periodic() {
    updateClawRotationCurrentBuffer();
    mCymbalSpinner.set(TalonSRXControlMode.PercentOutput, mCymbalSpinnerWantedOutput);
    // This method will be called once per scheduler run
  }

  public void reset(){
    mCircularBuffer.clear();
    mRunningTotal = 0.0;
}
}
