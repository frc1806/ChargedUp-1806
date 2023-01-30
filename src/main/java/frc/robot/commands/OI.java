package frc.robot.commands;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.XboxController;

public class OI extends CommandBase {
    private XboxController DriverController;
    private XboxController OperatorController;
    private XboxController DebugController;

    private DriveTrain mDriveTrain;

    private SendableChooser<DriverControls> controllerConfigChooser;
    private enum DriverControls {
        Classic,
        Forza,
    }

    public OI(DriveTrain driveTrain){
        DriverController = new XboxController(Constants.kDriverPort, "Driver", XboxControllerConfigValues.kDriverControllerDefaultConfig);
        OperatorController = new XboxController(Constants.kOperatorPort, "Operator", XboxControllerConfigValues.kOperatorControllerDefaultConfig);
        DebugController = new XboxController(Constants.kOperatorPort, "Debug", XboxControllerConfigValues.kOperatorControllerDefaultConfig);

        controllerConfigChooser = new SendableChooser<DriverControls>();
        controllerConfigChooser.addOption("Forza", DriverControls.Forza);
        controllerConfigChooser.addOption("Classic", DriverControls.Classic);

        mDriveTrain = mDriveTrain.getInstance();
        addRequirements(driveTrain);
    }

    @Override
    public void end(boolean interrupted) {
        mDriveTrain.stop();
    }

    @Override
    public void execute() {
        DriverControls selectedControls;

        if(controllerConfigChooser.getSelected() != null){
            selectedControls = controllerConfigChooser.getSelected();
        } else {
            selectedControls = DriverControls.Classic;
        }

        double throttle;
        double turn;
        boolean quickTurn;
        boolean creep;

        switch(selectedControls){
            case Forza:
                return;
            default:
            case Classic:
                throttle = DriverController.getLeftJoyY();
                turn = DriverController.getRightJoyX();
                quickTurn = DriverController.getRightTriggerAsDigital();
                creep = DriverController.getLeftTriggerAsDigital();
        }


        if(DriverController.getLeftTriggerAsDigital()){
            mDriveTrain.setCreepMode(throttle, turn, quickTurn);
        } else {
            mDriveTrain.setDriveMode(throttle, turn, quickTurn);
        }
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        if (RobotState.isDisabled()){
            return true;
        }
        return false;
    }
    
}
