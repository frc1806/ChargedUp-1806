// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveArmToPlacement;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.commands.WaitAndDoNothing;
import frc.robot.commands.arm.ArmGoToAngle;
import frc.robot.commands.protruder.DetectProtruderUnprotrude;
import frc.robot.commands.protruder.ProtruderGoToExtension;
import frc.robot.game.Placement;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederStationCone extends SequentialCommandGroup {
  /** Creates a new MoveArmToPlacement. */
  public FeederStationCone() {


    addCommands(
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new ProtruderGoToExtension(Constants.kProtruderDistanceAtFullRetract)
          , new CloseClaw(RobotContainer.S_INTAKE)
          , new ArmGoToAngle(Placement.FEEDER_STATION_CONE.getPivotAngle())
          , new ProtruderGoToExtension(Placement.FEEDER_STATION_CONE.getExtendDistance())
          , new OpenClaw(RobotContainer.S_INTAKE)
          , new DetectProtruderUnprotrude(RobotContainer.S_PROTRUDER)
        ),
        new Drive(RobotContainer.S_DRIVETRAIN, RobotContainer.S_DRIVECONTROLS))

      ,      new ParallelRaceGroup(
          new SequentialCommandGroup(
          new CloseClaw(RobotContainer.S_INTAKE)
          , new WaitAndDoNothing(.125))
        , new TimedDriveCommand(RobotContainer.S_DRIVETRAIN, 3.0, 0.0))
    
      , new ParallelRaceGroup(
          new MoveArmToPlacement(Placement.HOME)
        , new TimedDriveCommand(RobotContainer.S_DRIVETRAIN, 0.15, -1.0))
      , new ParallelRaceGroup(
         new MoveArmToPlacement(Placement.HOME)
        , new Drive(RobotContainer.S_DRIVETRAIN, RobotContainer.S_DRIVECONTROLS))
        );
      
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
