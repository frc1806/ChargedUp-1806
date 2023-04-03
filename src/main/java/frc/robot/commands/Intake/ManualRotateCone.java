// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CymbalSpinner;
import frc.robot.subsystems.DriverControls;

public class ManualRotateCone extends CommandBase {
  /** Creates a new ManualRotateCone. */
  CymbalSpinner mClawSpin;
  DriverControls mControls;
  
  public ManualRotateCone() {
    mClawSpin = RobotContainer.S_CYMBAL_SPEEEEEEN;
    mControls = RobotContainer.S_DRIVECONTROLS;
    addRequirements(mClawSpin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {mClawSpin.rotateClaw(mControls.o_getManualRotateCone());}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
