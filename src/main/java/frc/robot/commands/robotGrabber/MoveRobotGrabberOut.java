// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.robotGrabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RoboGrabbers;

public class MoveRobotGrabberOut extends CommandBase {

  private RoboGrabbers mRoboGrabbers;
  /** Creates a new MoveRobotGrabberOut. */
  public MoveRobotGrabberOut(RoboGrabbers robotGrabber) {
    mRoboGrabbers = robotGrabber;
    addRequirements(robotGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mRoboGrabbers.moveOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mRoboGrabbers.moveOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mRoboGrabbers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
