// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Protruder;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  public static final SendableChooser<Command> mSendableChooser = new SendableChooser<>();
  
  //DEFINE SUBSYSTEM INSTANCES
  public static final DriverControls S_DRIVECONTROLS = new DriverControls();
  public static final DriveTrain S_DRIVETRAIN = new DriveTrain();
  public static final VisionSubsystem S_REAR_VISION_SUBSYSTEM = new VisionSubsystem("limelight");
  public static final Claw S_INTAKE = new Claw();
  public static final Protruder S_PROTRUDER = new Protruder();

  //Compressor
  public Compressor compressor;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //SET DEFAULT COMMANDS
    setDefaultCommands();
    
    configureBindings();
    configureAutonomousOptions();
  }

  /**
   * Set default commands for the subsystems.
   * Default commands run when a subsystem has no other commands running.
   * Example: drivetrain should default to joysticks when not being used by
   * automatic driving classes.
   */
  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_DRIVETRAIN, new Drive(S_DRIVETRAIN, S_DRIVECONTROLS));
  }

  /**
   * Set button bindings for the driver in {@link DriverControls}, but set operator bindings here.
   */
  private void configureBindings() {
    S_DRIVECONTROLS.registerTriggers(S_DRIVETRAIN, S_REAR_VISION_SUBSYSTEM, S_INTAKE, S_PROTRUDER);
  }

  /**
   * Configure the {@link SendableChooser} for our autononomous options here.
   */
  private void configureAutonomousOptions(){
    mSendableChooser.addOption("DeadReckoning No Obstacle Auto", S_DRIVETRAIN.followTrajectoryCommand(PathPlanner.loadPath("NoObstacleDeadReckoning", new PathConstraints(4.0, 3.0)), true));

    //TODO: Put on main comp tab
    SmartDashboard.putData("Auto Chooser", mSendableChooser);
  }

  public Command getAutonomousCommand() {
    if(mSendableChooser.getSelected() == null)
    {
      return Commands.print("No autonomous command selected");
    }
    return (Command) mSendableChooser.getSelected();
  }
}
