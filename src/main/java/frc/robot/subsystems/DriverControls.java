package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.RearVisionSteerAndDrive;
import frc.robot.commands.ToggleIntake;
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
        debugController = new SWATXboxController(Constants.kOperatorPort, "Debug", XboxControllerConfigValues.kOperatorControllerDefaultConfig);

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

    public boolean getConfirmPlacement(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getYButton();
            case Forza:
                return driverController.getYButton();
        }
    }

    //Operator Controls

    

    //Debug Controls

    public boolean getIntakeLeft(){
        return debugController.getLeftTriggerDigital();
    }

    public boolean getIntakeRight(){
        return debugController.getRightTriggerDigital();
    }


    /**
     * Register all the controls for the robot. Note that selected controls updates won't happen without a roborio reboot due to the way that triggers work.
     * @param driveTrain Our one and only drivetrain
     * @param visionSubsystem our (currently) one and only vision subsystem representing the limelight
     */
    public void registerTriggers(DriveTrain driveTrain, VisionSubsystem visionSubsystem, Intake intake){
        new Trigger(this::getVisionLineup).whileTrue(new RearVisionSteerAndDrive(driveTrain, this, visionSubsystem));
        new Trigger(this::getIntakeMode).onTrue(new ToggleIntake(intake));
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
