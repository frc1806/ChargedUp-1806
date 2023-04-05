// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveArmToPlacement;
import frc.robot.commands.WaitAndDoNothing;
import frc.robot.commands.arm.ArmGoToAngle;
import frc.robot.commands.protruder.ProtruderGoToExtension;
import frc.robot.game.Placement;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederStation extends SequentialCommandGroup {
  /** Creates a new MoveArmToPlacement. */
  public FeederStation() {


    addCommands(
      new ProtruderGoToExtension(Constants.kProtruderDistanceAtFullRetract)
      , new CloseClaw(RobotContainer.S_INTAKE)
      , new ArmGoToAngle(Placement.FEEDER_STATION.getPivotAngle())
      , new ProtruderGoToExtension(Placement.FEEDER_STATION.getExtendDistance())
      , new OpenClaw(RobotContainer.S_INTAKE)
      , new WaitForGPSenseThenCloseClaw()
      , new WaitAndDoNothing(.125)
      , new MoveArmToPlacement(Placement.HOME));
      
      //addRequirements(RobotContainer.S_INTAKE, RobotContainer.S_PIVOTARM, RobotContainer.S_PROTRUDER);
  }

  // private Placement getWantedIntakePlacement(GamePieceMode mode){
  //   switch(mode){
  //     case ConeMode:
  //       return Placement.GROUND_INTAKE_CONE;
  //     case OffMode:
  //     default:
  //     case CubeMode:
  //       return Placement.GROUND_INTAKE_CUBE;
  //   }
  // }
}
