// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.OpenClaw;
import frc.robot.commands.arm.ArmGoToCurrentAngle;
import frc.robot.game.Placement;


public class PlaceSequence extends SequentialCommandGroup {
  /** Creates a new PlaceSequence. */
  public PlaceSequence() {
    //This will set the arm to the current angle, so the manual button can be released if being used, open the claw, and then bring the arm home.
    addCommands(new ArmGoToCurrentAngle(), new OpenClaw(RobotContainer.S_INTAKE), new MoveArmToPlacement(Placement.HOME));
  }
}
