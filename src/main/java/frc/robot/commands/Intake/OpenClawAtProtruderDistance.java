// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OpenClawAtProtruderDistance extends CommandBase {

  private double mDistanceToOpenAt;
  private boolean mHasOpenedClaw;
  /** Creates a new OpenClawAtProtruderDistance. */
  public OpenClawAtProtruderDistance(Double distanceToOpenAt) {
    mDistanceToOpenAt = distanceToOpenAt;
    mHasOpenedClaw = false;
    addRequirements(RobotContainer.S_INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!mHasOpenedClaw && RobotContainer.S_PROTRUDER.getDistance() > mDistanceToOpenAt){
      RobotContainer.S_INTAKE.openBoth();
      mHasOpenedClaw = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mHasOpenedClaw;
  }
}
