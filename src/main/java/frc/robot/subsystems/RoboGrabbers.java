// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class RoboGrabbers extends SubsystemBase {
  private CANSparkMax mGrabberMotor;

  /** Creates a new RoboGrabbers. */
  public RoboGrabbers() {
    mGrabberMotor = new CANSparkMax(RobotMap.robotGrabberMotor, MotorType.kBrushed);
    mGrabberMotor.setIdleMode(IdleMode.kBrake);

    mGrabberMotor.setSmartCurrentLimit(15); //775 and also we don't want to destroy things.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void moveOut(){
    mGrabberMotor.setVoltage(5.0);
  }

  public void stop(){
    mGrabberMotor.setVoltage(0.0);
  }
  public void moveInt(){
    mGrabberMotor.setVoltage(-3.0);
  }
}
