
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.FalconPunch;
import frc.robot.commands.FeederStationLineupDrive;
import frc.robot.commands.ForceUpdateOdometryFromVision;
import frc.robot.commands.MoveArmToPlacement;
import frc.robot.commands.PlaceSequence;
import frc.robot.commands.ScoringLineupDrive;
import frc.robot.commands.SetDriveBrake;
import frc.robot.commands.ToggleGamePieceMode;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.DebugCommands.CymbalSpinManual;
import frc.robot.commands.DebugCommands.LeftSolenoid;
import frc.robot.commands.DebugCommands.ManualExtend;
import frc.robot.commands.DebugCommands.RightSolenoid;
import frc.robot.commands.DebugCommands.ToggleProtruderBrake;
import frc.robot.commands.Intake.FeederStation;
import frc.robot.commands.Intake.FeederStationCone;
import frc.robot.commands.Intake.GroundIntake;
import frc.robot.commands.Intake.RotateCone;
import frc.robot.game.Placement;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;
import frc.robot.util.SWATXboxController;

public class DriverControls extends SubsystemBase {
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
     * Creates a DriverControls subsystem. The subsystem used to keep track of the
     * driver's controls based on the status of a sendable chooser.
     */
    public DriverControls() {
        driverController = new SWATXboxController(Constants.kDriverPort, "Driver",
                XboxControllerConfigValues.kDriverControllerDefaultConfig);
        operatorController = new SWATXboxController(Constants.kOperatorPort, "Operator",
                XboxControllerConfigValues.kOperatorControllerDefaultConfig);
        debugController = new SWATXboxController(Constants.kDebugPort, "Debug",
                XboxControllerConfigValues.kOperatorControllerDefaultConfig);

        controllerConfigChooser = new SendableChooser<DriverControlType>();
        controllerConfigChooser.addOption("Forza", DriverControlType.Forza);
        controllerConfigChooser.addOption("Classic", DriverControlType.Classic);
        selectedControls = DriverControlType.Classic;

    }

    /**
     * Get the current selected driver controls
     */
    public DriverControlType getCurrentDriverControls() {
        return selectedControls;
    }

    /**
     * Get drivetrain throttle control value
     * 
     * @return a {@link double} between -1 and 1
     */
    public double getThrottle() {
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getLeftY();
            case Forza:
                if (driverController.getLeftTriggerAxis() > 0) {
                    return driverController.getLeftTriggerAxis();
                } else if (driverController.getRightTriggerAxis() > 0) {
                    return -driverController.getRightTriggerAxis();
                } else {
                    return 0.0;
                }
        }
    }

    public double getLeftStick(){
        return driverController.getLeftY();
    }

    public double getRightStick(){
        return driverController.getRightY();
    }

    /**
     * Get drivetrain turn control value For curvature drive, 1 is counterclockwise.
     * 
     * @return a {@link double} between -1 and 1
     */
    public double getTurn() {
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getRightX();
            case Forza:
                return driverController.getLeftX();
        }
    }

    /**
     * 
     * @return Whether or not to turn quickly and/or allow turn in place.
     */
    public boolean getQuickTurn() {
        switch (selectedControls) {
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
    public boolean getCreepMode() {
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getLeftTriggerDigital();
            case Forza:
                return driverController.getXButton();
        }
    }

    public boolean getSuperPowerBrake(){
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getStartButton();
            case Forza:
                return driverController.getStartButton();
        }
    }

    public boolean getFeederLineup(){
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getYButton();
            case Forza:
                return driverController.getYButton();
        }
    }

    public boolean getScoringLineup(){
        switch (selectedControls) {
            default:
            case Classic:
                return driverController.getXButton();
            case Forza:
                return driverController.getXButton();
        }
    }

    public boolean floorIntakeMode(){
        return driverController.getRightBumper();
    }

    // Operator Controls

    public boolean o_lowConePlacement() {
        return operatorController.getPOVDown() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode;
    }

    public boolean o_lowCubePlacement() {
        return operatorController.getPOVDown() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode;
    }

    public boolean o_medConePlacement(){
        return operatorController.getPOVLeft() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode;
    }

    public boolean o_medCubePlacement(){
        return operatorController.getPOVLeft() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode;
    }

    public boolean o_highConePlacement() {
        return operatorController.getPOVUp() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode;
    }

    public boolean o_highCubePlacement() {
        return operatorController.getPOVUp() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode;
    }

    public boolean o_feederStation(){
        return operatorController.getAButton() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode;
    }

    public boolean o_feederStationCone(){
        return operatorController.getAButton() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode;
    }

    public boolean o_goHome(){
        return operatorController.getStartButton();
    }

    public double o_getManualRotateCone(){
        return operatorController.getLeftY();
    }

    
    public boolean o_wantSpin() {
        return operatorController.getBButton();
    }

    

    public boolean o_switchModes(){
        return operatorController.getRightBumper();
    }

    public double o_spinnerThrottle(){
        return operatorController.getRightY();
    }

    public boolean o_wantSpinThrottle(){
        return o_spinnerThrottle() != 0.0;
    }

    public boolean o_getIntakeMode() {
        return operatorController.getLeftBumper();
    }

    public boolean o_getGroundIntakeCube(){
        return operatorController.getYButton() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.CubeMode;
    }

    public boolean o_getGroundIntakeCone(){
        return operatorController.getYButton() && RobotContainer.GetCurrentGamePieceMode() == GamePieceMode.ConeMode;
    }

    public boolean o_throwGamePiece(){
        return operatorController.getXButton();
    }

    public double o_rotateThrottle(){
        return operatorController.getRightY();
    }

    public boolean o_wantManualArmRotate(){
        return RobotContainer.S_INTAKE.isAngleSafeForClawOpen(RobotContainer.S_PIVOTARM.mCurrentDesiredAngle) 
        && operatorController.getRightTriggerDigital();
    }

    public boolean o_wantPlaceSequence(){
        return operatorController.getLeftTriggerDigital();
    }

    // Operator LED Control

    // Debug Controls

    public double d_pivotArmManual() {
        return debugController.getRightY();
    }

    public boolean d_wantArmManual() {
        return d_pivotArmManual() != 0;
    }

    public boolean d_getIntakeLeft() {
        return debugController.getLeftTriggerDigital();
    }

    public boolean d_getIntakeRight() {
        return debugController.getRightTriggerDigital();
    }

    public boolean d_getWantForceOdometryUpdate(){
        return driverController.getBackButton();
    }

    /*
    public double d_cymbalThrottle() {
        return debugController.getLeftY();
    }
    

    public boolean d_wantCymbalManual() {
        return d_cymbalThrottle() != 0;
    }
    */

    public double d_extendThrottle(){
        return debugController.getLeftY();
    }

    public boolean d_wantManualExtend(){
        return d_extendThrottle() != 0;
    }

    public boolean d_toggleProtruderBrakeMode(){
        return debugController.getXButton();
    }
    // Debug Tabs

    public boolean debugTabs() {
        return driverController.getStartButton() && driverController.getBackButton();
    }

    public boolean o_debugTabs() {
        return operatorController.getStartButton() && operatorController.getBackButton();
    }

    public boolean d_debugTabs() {
        return debugController.getStartButton() && debugController.getBackButton();
    }



    /**
     * Register all the controls for the robot. Note that selected controls updates won't happen without a roborio reboot due to the way that triggers work.
     * @param driveTrain Our one and only drivetrain
     * @param visionSubsystem our (currently) one and only vision subsystem representing the limelight
     */
    public void registerTriggers(DriveTrain driveTrain, Vision visionSubsystem, Claw intake, Protruder protruder, PivotArm arm, LED led){
        //Driver
        new Trigger(this::getFeederLineup).whileTrue(new FeederStationLineupDrive(driveTrain, this, visionSubsystem));
        new Trigger(this::getScoringLineup).whileTrue(new ScoringLineupDrive(driveTrain, this, visionSubsystem));
        new Trigger(this::getSuperPowerBrake).onTrue(new SetDriveBrake(driveTrain));

        //Operator
        new Trigger(this::o_lowConePlacement).onTrue(new MoveArmToPlacement(Placement.LOW_PLACEMENT_CONE));
        new Trigger(this::o_medConePlacement).onTrue(new MoveArmToPlacement(Placement.MED_PLACEMENT_CONE));
        new Trigger(this::o_highConePlacement).onTrue(new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CONE));
        new Trigger(this::o_lowCubePlacement).onTrue(new MoveArmToPlacement(Placement.LOW_PLACEMENT_CUBE));
        new Trigger(this::o_medCubePlacement).onTrue(new MoveArmToPlacement(Placement.MED_PLACEMENT_CUBE));
        new Trigger(this::o_highCubePlacement).onTrue(new MoveArmToPlacement(Placement.HIGH_PLACEMENT_CUBE));
        new Trigger(this::o_goHome).onTrue(new MoveArmToPlacement(Placement.HOME));
        new Trigger(this::o_feederStation).onTrue(new FeederStation());
        new Trigger(this::o_feederStationCone).onTrue(new FeederStationCone());
        new Trigger(this::o_getGroundIntakeCube).onTrue(new GroundIntake(GamePieceMode.CubeMode));
        new Trigger(this::o_getGroundIntakeCone).onTrue(new GroundIntake(GamePieceMode.ConeMode));
        new Trigger(this::o_wantSpin).onTrue(new RotateCone());
        new Trigger(this::o_switchModes).onTrue(new ToggleGamePieceMode());
        new Trigger(this::o_wantSpinThrottle).whileTrue(new CymbalSpinManual(intake, this));
        new Trigger(this::o_getIntakeMode).onTrue(new ToggleIntake(intake));
        new Trigger(this::o_wantPlaceSequence).onTrue(new PlaceSequence());
        //Debug
        new Trigger(this::d_getIntakeLeft).onTrue(new LeftSolenoid(intake));
        new Trigger(this::d_getIntakeRight).onTrue(new RightSolenoid(intake));
        //new Trigger(this::d_wantCymbalManual).whileTrue(new CymbalSpinManual(intake, this));
        new Trigger(this::d_wantManualExtend).whileTrue(new ManualExtend(protruder, this));
        new Trigger(this::d_toggleProtruderBrakeMode).onTrue(new ToggleProtruderBrake(protruder));
        new Trigger(this::d_getWantForceOdometryUpdate).whileTrue(new ForceUpdateOdometryFromVision());
    }

    @Override
    public void periodic() {
        if (controllerConfigChooser.getSelected() != null) {
            if (selectedControls != controllerConfigChooser.getSelected()) {
                selectedControls = controllerConfigChooser.getSelected();
                // registerBindings(); TODO: Maybe someday we can un-declare triggers.
            }
        } else {
            selectedControls = DriverControlType.Classic;
        }

        RobotContainer.S_PIVOTARM.setWantedManualPower(o_rotateThrottle());
    }

}
