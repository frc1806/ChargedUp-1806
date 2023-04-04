// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.game.Placement;

public class ArmGoToCurrentAngle extends ArmGoToAngle {
  /** Creates a new ArmGoToCurrentAngle. */
  public ArmGoToCurrentAngle() {
    super(Placement.HOME.getPivotAngle()); //this will be later ignored.
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wantedAngle = RobotContainer.S_PIVOTARM.getAngle(); //Set wanted angle to current angle
    RobotContainer.S_PIVOTARM.goToPosition(wantedAngle);
  }
}
