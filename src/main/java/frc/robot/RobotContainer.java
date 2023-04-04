// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveArmToPlacement;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.AutoModes.DeadReckoningNoObstacle;
import frc.robot.commands.DebugCommands.CymbalSpinManual;
import frc.robot.commands.Intake.CloseClaw;
import frc.robot.commands.Intake.GroundIntake;
import frc.robot.commands.Intake.ManualRotateCone;
import frc.robot.commands.Intake.OpenClaw;
import frc.robot.game.Placement;
import frc.robot.shuffleboard.ShuffleboardManager;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CymbalSpinner;
import frc.robot.subsystems.Protruder;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  public enum GamePieceMode{
    ConeMode,
    CubeMode,
    OffMode
  };

  public static final SendableChooser<CommandBase> mSendableChooser = new SendableChooser<>();
  
  //DEFINE SUBSYSTEM INSTANCES
  public static final DriverControls S_DRIVECONTROLS = new DriverControls();
  public static final DriveTrain S_DRIVETRAIN = new DriveTrain();
  public static final Vision S_REAR_VISION_SUBSYSTEM = new Vision("limelight");
  public static final Claw S_INTAKE = new Claw();
  public static final Protruder S_PROTRUDER = new Protruder();
  public static final PivotArm S_PIVOTARM = new PivotArm();
  public static final LED S_TWO_LED_SUBSYTEM = new LED();
  public static final CymbalSpinner S_CYMBAL_SPEEEEEEN = new CymbalSpinner();

  private static GamePieceMode ES_CURRENT_GAME_PIECE_MODE = GamePieceMode.CubeMode;
  
  public static GamePieceMode GetCurrentGamePieceMode() {
    return ES_CURRENT_GAME_PIECE_MODE;
  }

  public static void SetCurrentGamePieceMode(GamePieceMode eS_CURRENT_GAME_PIECE_MODE) {
    ES_CURRENT_GAME_PIECE_MODE = eS_CURRENT_GAME_PIECE_MODE;
  }

  private static Placement S_CURRENT_PLACEMENT = Placement.HOME;

  public static Placement GetCurrentPlacement(){
    return S_CURRENT_PLACEMENT;
  }

  public static void SetCurrentPlacement(Placement placement){
    S_CURRENT_PLACEMENT = placement;
  }

  //Shuffleboard Manager
  public ShuffleboardManager mShuffleboardManager;
  //Compressor
  public Compressor compressor;

  public RobotContainer() {
    mShuffleboardManager = new ShuffleboardManager();
    mShuffleboardManager.registerTabs();

    DriverStation.silenceJoystickConnectionWarning(true);
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    S_REAR_VISION_SUBSYSTEM.updateLimelightPose(Units.inchesToMeters(2), Units.inchesToMeters(8.75), Units.inchesToMeters(35), 180, -30, 0); //TODO: Update limelight pose to reflect actual robot
    //SET DEFAULT COMMANDS
    setDefaultCommands();
    
    configureBindings();
    configureAutonomousOptions();
    
    compressor.enableAnalog(90.0,120.0);
  }

  /**
   * Set default commands for the subsystems.
   * Default commands run when a subsystem has no other commands running.
   * Example: drivetrain should default to joysticks when not being used by
   * automatic driving classes.
   */
  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_DRIVETRAIN, new Drive(S_DRIVETRAIN, S_DRIVECONTROLS));
    CommandScheduler.getInstance().setDefaultCommand(S_CYMBAL_SPEEEEEEN, new ManualRotateCone());
  }

  /**
   * Set button bindings for the driver in {@link DriverControls}, but set operator bindings here.
   */
  private void configureBindings() {
    S_DRIVECONTROLS.registerTriggers(S_DRIVETRAIN, S_REAR_VISION_SUBSYSTEM, S_INTAKE, S_PROTRUDER, S_PIVOTARM, S_TWO_LED_SUBSYTEM);
  }

  /**
   * Configure the {@link SendableChooser} for our autononomous options here.
   */
  private void configureAutonomousOptions(){

// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
//List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));



    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("lowCube", new MoveArmToPlacement(Placement.LOW_PLACEMENT_CUBE));
    eventMap.put("lowCone", new MoveArmToPlacement(Placement.LOW_PLACEMENT_CONE));
    eventMap.put("medCube", new MoveArmToPlacement(Placement.MED_PLACEMENT_CUBE));
    eventMap.put("medCone", new MoveArmToPlacement(Placement.MED_PLACEMENT_CONE));
    eventMap.put("highCube", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE));
    eventMap.put("highCone", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CONE));
    eventMap.put("homeArm", new  MoveArmToPlacement(Placement.HOME));
    eventMap.put("openClaw", new OpenClaw(S_INTAKE));
    eventMap.put("closeClaw", new CloseClaw(S_INTAKE));
    eventMap.put("placeConeHigh", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CONE).andThen(new OpenClaw(S_INTAKE)).andThen( new MoveArmToPlacement(Placement.HOME)));
    eventMap.put("placeCubeHigh", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE).andThen(new OpenClaw(S_INTAKE)).andThen( new MoveArmToPlacement(Placement.HOME)));
    eventMap.put("groundIntakeCone", new GroundIntake(GamePieceMode.ConeMode));
    eventMap.put("groundIntakeCube", new GroundIntake(GamePieceMode.CubeMode));


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

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

      //mSendableChooser.addOption("Full Auto", autoBuilder.fullAuto(pathGroup));
      mSendableChooser.addOption("Dead Reckoning No Obstacle", new DeadReckoningNoObstacle(S_DRIVETRAIN));
      mSendableChooser.addOption("PlaceCubeHigh", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE).andThen(new ToggleIntake(S_INTAKE)).andThen(new WaitCommand(2.0)).andThen(new MoveArmToPlacement(Placement.HOME)));
      mSendableChooser.addOption("PlaceCubeHighShortMobility", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE).andThen(new ToggleIntake(S_INTAKE)).andThen(new WaitCommand(2.0)).andThen(new MoveArmToPlacement(Placement.HOME)).andThen(new TimedDriveCommand(S_DRIVETRAIN, 2.5, 0.25)));
      mSendableChooser.addOption("PlaceCubeHighLongMobility", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE).andThen(new ToggleIntake(S_INTAKE)).andThen(new WaitCommand(2.0)).andThen(new MoveArmToPlacement(Placement.HOME)).andThen(new TimedDriveCommand(S_DRIVETRAIN, 3.5, 0.25)));
      mSendableChooser.addOption("PlaceCubeHighChargeStationMobility", new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE).andThen(new ToggleIntake(S_INTAKE)).andThen(new WaitCommand(2.0)).andThen(new MoveArmToPlacement(Placement.HOME)).andThen(new TimedDriveCommand(S_DRIVETRAIN, 2.5, 0.4).andThen(new TimedDriveCommand(S_DRIVETRAIN, 0.1, -.2))));
      mSendableChooser.addOption("ShortSideMobilityOnly", new TimedDriveCommand(S_DRIVETRAIN, 2.5, 0.25));
      mSendableChooser.addOption("LongSideMobilityOnly", new TimedDriveCommand(S_DRIVETRAIN, 3.5, 0.25));
      mSendableChooser.addOption("Desperation Mode",   new TimedDriveCommand(S_DRIVETRAIN, 1.0, -0.4).andThen(new TimedDriveCommand(S_DRIVETRAIN, 3.5, 0.25)));

    
      mSendableChooser.addOption("DoNothing", new CommandBase() {
        @Override
        public void initialize() {
          System.out.println("Nothing Auto. Fortnite Gaming.");
        }
      });
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
