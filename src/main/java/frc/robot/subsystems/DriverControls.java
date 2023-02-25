package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.RearVisionSteerAndDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.DebugCommands.LeftSolenoid;
import frc.robot.commands.DebugCommands.ManualRotate;
import frc.robot.commands.DebugCommands.RightSolenoid;
import frc.robot.commands.Intake.GoHome;
import frc.robot.game.Placement;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;
import frc.robot.util.SWATXboxController;

public class DriverControls extends SubsystemBase{
    private SWATXboxController driverController;
    private SWATXboxController operatorController;
    private SWATXboxController debugController;
    DriverControlType selectedControls;

    public static SendableChooser<DriverControlType> controllerConfigChooser;
    private enum DriverControlType {
        Classic,
        Forza,
    }

    /**
     * Creates a DriverControls subsystem. The subsystem used to keep track of the driver's controls based on the status of a sendable chooser.
     */
    public DriverControls(){
        driverController = new SWATXboxController(Constants.kDriverPort, "Driver", XboxControllerConfigValues.kDriverControllerDefaultConfig);
        operatorController = new SWATXboxController(Constants.kOperatorPort, "Operator", XboxControllerConfigValues.kOperatorControllerDefaultConfig);
        debugController = new SWATXboxController(Constants.kDebugPort, "Debug", XboxControllerConfigValues.kOperatorControllerDefaultConfig);

        controllerConfigChooser = new SendableChooser<DriverControlType>();
        controllerConfigChooser.addOption("Forza", DriverControlType.Forza);
        controllerConfigChooser.addOption("Classic", DriverControlType.Classic);
        selectedControls = DriverControlType.Classic;

    }

    /**
     * Get the current selected driver controls
     */
    public DriverControlType getCurrentDriverControls(){
        return selectedControls;
    }

    /**
     * Get drivetrain throttle control value
     * @return a {@link double} between -1 and 1
     */
    public double getThrottle(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getLeftY();
            case Forza:
                if(driverController.getLeftTriggerAxis() > 0){
                    return driverController.getLeftTriggerAxis();
                } else if (driverController.getRightTriggerAxis() > 0){
                    return -driverController.getRightTriggerAxis();
                } else {
                    return 0.0;
                }
        }
    }

    /**
     * Get drivetrain turn control value For curvature drive, 1 is counterclockwise.
     * @return a {@link double} between -1 and 1
     */
    public double getTurn(){
        switch(selectedControls){
            default:
            case Classic:
                return -driverController.getRightX();
            case Forza:
                return -driverController.getLeftX();
        }
    }

    /**
     * 
     * @return Whether or not to turn quickly and/or allow turn in place.
     */
    public boolean getQuickTurn(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getRightTriggerDigital();
            case Forza:
                return driverController.getBButton();
        }
    }

    /**
     * 
     * @return Whether or not to drive in creep mode
     */
    public boolean getCreepMode(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getLeftTriggerDigital();
            case Forza:
                return driverController.getXButton();
        }
    }

    public boolean getVisionLineup(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getAButton();
            case Forza:
                return driverController.getAButton();
        }
    }

    public boolean getIntakeMode(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getBButton();
            case Forza:
                return driverController.getLeftBumper();
        }
    }

    //Operator Controls

    public boolean o_lowCubePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.LOW_PLACEMENT_CUBE);
        return operatorController.getPOVDown();
    }

    public boolean o_medCubePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.MED_PLACEMENT_CUBE);
        return operatorController.getPOVLeft();
    }

    public boolean o_highCubePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.HIGH_PLACEMENT_CUBE);
        return operatorController.getPOVUp();
    }

    public boolean o_lowConePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.LOW_PLACEMENT_CONE);
        return operatorController.getAButton();
    }

    public boolean o_medConePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.MED_PLACEMENT_CONE);
        return operatorController.getBButton();
    }

    public boolean o_highConePlacement(){
        RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.HIGH_PLACEMENT_CONE);
        return operatorController.getYButton();
    }

    public boolean o_goHome(){
        if(!o_lowConePlacement() && !o_lowCubePlacement() && !o_medConePlacement() && !o_medCubePlacement() && !o_medConePlacement() && !o_highConePlacement() && !o_highCubePlacement()){
            RobotContainer.S_PROTRUDER.setCurrentPlacement(Placement.Home);
            return true;
        }
        return false;
    }

    public double o_spinnerThrottle(){
        return operatorController.getRightX();
    }

    public boolean o_wantSpin(){
        return o_spinnerThrottle() != 0;
    }


    //Operator LED Control



    //Debug Controls

    public double d_pivotArmManual(){
        return debugController.getRightY();
    }

    public boolean d_wantArmManual(){
        return d_pivotArmManual() != 0;
    }

    public boolean d_getIntakeLeft(){
        return debugController.getLeftTriggerDigital();
    }

    public boolean d_getIntakeRight(){
        return debugController.getRightTriggerDigital();
    }


    // Debug Tabs

    public boolean debugTabs(){
        return driverController.getStartButton() && driverController.getBackButton();
    }

    public boolean o_debugTabs(){
        return operatorController.getStartButton() && operatorController.getBackButton();
    }

    public boolean d_debugTabs(){
        return debugController.getStartButton() && debugController.getBackButton();
    }


    /**
     * Register all the controls for the robot. Note that selected controls updates won't happen without a roborio reboot due to the way that triggers work.
     * @param driveTrain Our one and only drivetrain
     * @param visionSubsystem our (currently) one and only vision subsystem representing the limelight
     */
    public void registerTriggers(DriveTrain driveTrain, VisionSubsystem visionSubsystem, Claw intake, Protruder protruder, PivotArm arm){
        //Driver
        new Trigger(this::getVisionLineup).whileTrue(new RearVisionSteerAndDrive(driveTrain, this, visionSubsystem));
        new Trigger(this::getIntakeMode).onTrue(new ToggleIntake(intake));

        //Operator
        new Trigger(this::o_lowCubePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_lowConePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_medConePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_medCubePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_highConePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_highCubePlacement).onTrue(new PlaceGamePiece(protruder, arm));
        new Trigger(this::o_goHome).whileTrue(new GoHome(arm, protruder));
        new Trigger(this::o_wantSpin).whileTrue(RobotContainer.S_INTAKE.rotateClaw(o_spinnerThrottle()));

        //Debug
        new Trigger(this::d_getIntakeLeft).onTrue(new LeftSolenoid(intake));
        new Trigger(this::d_getIntakeRight).onTrue(new RightSolenoid(intake));
        new Trigger(this::d_wantArmManual).whileTrue(new ManualRotate(arm, this));
    }

    @Override
    public void periodic() {
        if(controllerConfigChooser.getSelected() != null){
            if(selectedControls != controllerConfigChooser.getSelected()){
                selectedControls = controllerConfigChooser.getSelected();
                //registerBindings(); TODO: Maybe someday we can un-declare triggers.
            }
        } else {
            selectedControls = DriverControlType.Classic;
        }
    }
    
}
