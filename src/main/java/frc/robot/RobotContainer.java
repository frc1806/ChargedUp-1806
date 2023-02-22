// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoModes.DeadReckoningNoObstacle;
import frc.robot.shuffleboard.ShuffleboardManager;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Protruder;
import frc.robot.subsystems.TwoLEDSubsytem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  public enum GamePieceMode{
    ConeMode,
    CubeMode,
    OffMode
  };

  public static final SendableChooser<Command> mSendableChooser = new SendableChooser<>();
  
  //DEFINE SUBSYSTEM INSTANCES
  public static final DriverControls S_DRIVECONTROLS = new DriverControls();
  public static final DriveTrain S_DRIVETRAIN = new DriveTrain();
  public static final VisionSubsystem S_REAR_VISION_SUBSYSTEM = new VisionSubsystem("limelight");
  public static final Claw S_INTAKE = new Claw();
  public static final Protruder S_PROTRUDER = new Protruder();
  public static final PivotArm S_PIVOTARM = new PivotArm();
  public static final TwoLEDSubsytem S_TWO_LED_SUBSYTEM = new TwoLEDSubsytem();

  public static GamePieceMode E_CURRENT_GAME_PIECE_MODE = GamePieceMode.CubeMode;

  //Shuffleboard Manager
  public ShuffleboardManager mShuffleboardManager;
  //Compressor
  public Compressor compressor;

  public RobotContainer() {
    mShuffleboardManager = new ShuffleboardManager();
    mShuffleboardManager.registerTabs();

    DriverStation.silenceJoystickConnectionWarning(true);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    S_REAR_VISION_SUBSYSTEM.updateLimelightPose(Units.inchesToMeters(2), Units.inchesToMeters(2), Units.inchesToMeters(9), 180, 30, 0); //TODO: Update limelight pose to reflect actual robot
    //SET DEFAULT COMMANDS
    setDefaultCommands();
    
    configureBindings();
    configureAutonomousOptions();

    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
//eventMap.put("marker1", new PrintCommand("Passed marker 1"));

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    S_DRIVETRAIN::getPose, // Pose2d supplier
    S_DRIVETRAIN::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    new RamseteController(),
    S_DRIVETRAIN.mDifferentialDriveKinematics, // SwerveDriveKinematics
    new SimpleMotorFeedforward(Constants.kDriveTrainKs, Constants.kDriveTrainKv, Constants.kDriveTrainKa),
    S_DRIVETRAIN::getWheelSpeeds,
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    S_DRIVETRAIN::outputVolts,
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    S_DRIVETRAIN // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(pathGroup);


    
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
    S_DRIVECONTROLS.registerTriggers(S_DRIVETRAIN, S_REAR_VISION_SUBSYSTEM, S_INTAKE, S_PROTRUDER, S_PIVOTARM);
  }

  /**
   * Configure the {@link SendableChooser} for our autononomous options here.
   */
  private void configureAutonomousOptions(){
    mSendableChooser.addOption("Dead Reckoning No Obstacle", new DeadReckoningNoObstacle(S_DRIVETRAIN));
    mShuffleboardManager.addAutoChooser(mSendableChooser);
    
  }

  public Command getAutonomousCommand() {
    if(mSendableChooser.getSelected() == null)
    {
      return Commands.print("No autonomous command selected");
    }
    return (Command) mSendableChooser.getSelected();
  }

  public ShuffleboardManager getShuffleboardManager(){
    return mShuffleboardManager;
  }
}
