// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.util.TrapezoidalDiffDriveTurnPID;

public class FieldOrientedDrive extends CommandBase {
  DriveTrain drive;
  DriverControls controls;
  TrapezoidalDiffDriveTurnPID trapezoidalDiffDriveTurnPID;
  
  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedDrive(DriveTrain drive, DriverControls controls) {
    this.drive = drive;
    this.controls = controls;
    addRequirements(drive);
    trapezoidalDiffDriveTurnPID = new TrapezoidalDiffDriveTurnPID(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapezoidalDiffDriveTurnPID.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setDriveMode(controls.getThrottle(), trapezoidalDiffDriveTurnPID.calculate(controls.getWantedRadDriveRobotAngle()), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
