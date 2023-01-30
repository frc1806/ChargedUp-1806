package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.VisionSteerAndDrive;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;
import frc.robot.util.SWATXboxController;

public class DriverControls extends SubsystemBase{
    private SWATXboxController driverController;
    private SWATXboxController operatorController;
    private SWATXboxController debugController;
    DriverControlType selectedControls;

    private SendableChooser<DriverControlType> controllerConfigChooser;
    private enum DriverControlType {
        Classic,
        Forza,
    }

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
                return driverController.getRightTriggerAxis();
            
        }
    }

    /**
     * Get drivetrain turn control value
     * @return a {@link double} between -1 and 1
     */
    public double getTurn(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getRightX();
            case Forza:
                return driverController.getLeftX();
        }
    }

    public boolean getQuickTurn(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getRightTriggerDigital();
            case Forza:
                return driverController.getAButton();
        }
    }

    public boolean getCreepMode(){
        switch(selectedControls){
            default:
            case Classic:
                return driverController.getLeftTriggerDigital();
            case Forza:
                return driverController.getBButton();
        }
    }

    public void registerTriggers(DriveTrain driveTrain, VisionSubsystem visionSubsystem){
        
        switch(selectedControls){
            default:
            case Classic:
                new Trigger(driverController::getAButton).whileTrue(new VisionSteerAndDrive(driveTrain, this, visionSubsystem));
                break;
            case Forza:
                break;

        }
        

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
