// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.CloseClaw;
import frc.robot.commands.arm.ArmGoToAngle;
import frc.robot.commands.protruder.ProtruderGoToExtension;
import frc.robot.game.Placement;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToPlacement extends SequentialCommandGroup {
  /** Creates a new MoveArmToPlacement. */
  public MoveArmToPlacement(Placement placement) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ProtruderGoToExtension(Constants.kProtruderDistanceAtFullRetract)
      , new CloseClaw(RobotContainer.S_INTAKE)
      , new ArmGoToAngle(placement.getPivotAngle())
      , new ProtruderGoToExtension(placement.getExtendDistance()));
      //addRequirements(RobotContainer.S_INTAKE, RobotContainer.S_PIVOTARM, RobotContainer.S_PROTRUDER);
  }
}
