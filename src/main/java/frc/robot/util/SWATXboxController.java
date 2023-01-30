package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.shuffleboard.tabs.XboxControllerConfigDashboard;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;

/*
 * The XboxController class is a wrapper of the joystick class,
 * auto applies deadzones and makes nice methods for us
 */
public class SWATXboxController extends edu.wpi.first.wpilibj.XboxController {
	private static Map<String, Object> AXIS_DISPLAY_BAR_PROPS = new HashMap<>();

    static {
        AXIS_DISPLAY_BAR_PROPS.put("Min", -1.0d);
        AXIS_DISPLAY_BAR_PROPS.put("Max", 1.0d);
        AXIS_DISPLAY_BAR_PROPS.put("Center", 0.0d);
    }

	private final int mPort;
	public Timer rumbleTimer;
	private String mName;
	private ShuffleboardTab mControllerConfigTab;
	private XboxControllerConfigDashboard mControllerConfigDashboard;
	private XboxControllerConfigValues mConfigValues;
	
	public SWATXboxController(int port, String name, XboxControllerConfigValues defaultConfig) {
		super(port);
		mConfigValues = defaultConfig;
		mName = name;
		mPort = port;
		mControllerConfigTab = Shuffleboard.getTab(mName.concat(" Controller Configuration"));
		mControllerConfigDashboard = new XboxControllerConfigDashboard(mControllerConfigTab, defaultConfig);
		updateConfig();
		configValueDisplay();
	}

	public double getRawAxis(final int axis) {
		return DriverStation.getStickAxis(mPort, axis);
	}

	public double applyConfig(double value, double deadZone, double minimumOutput, double linearity)
	{
		if(Math.abs(value) < deadZone)
		{
			return 0.0;
		}
		//Apply Deadzone
		if(deadZone != 0)
		{
			value =(value>0?1:-1) * ((Math.abs(value) - deadZone) * (1.0 / (1.0 - deadZone)));
		}
		//Apply Linearity this doesn't need any absolute value to handle so long as the non-linear part is an odd power.
		value = (value * linearity) + ( Math.pow(value, 5.0) * (1.0 - linearity));
		//Apply Minimum output
		if(Math.abs(value) > 0)
		{
			value = (value>0?1:-1) * (Math.abs(value) * (1.0-minimumOutput) + minimumOutput);
		}
		return value;
		
	}


	@Override
	public double getRightTriggerAxis() {
			return applyConfig( getRawAxis(3), mConfigValues.getTriggerDeadzone(), mConfigValues.getTriggerMinimumOutput(), mConfigValues.getTriggerLinearity());
	}

	@Override
	public double getLeftTriggerAxis() {
			return applyConfig(getRawAxis(2), mConfigValues.getTriggerDeadzone(), mConfigValues.getTriggerMinimumOutput(), mConfigValues.getTriggerLinearity());
	}

	@Override
	public double getRightX() {
			return applyConfig(getRawAxis(4), mConfigValues.getRightXDeadzone(), mConfigValues.getRightXMinimumOutput(), mConfigValues.getRightXLinearity());
	}

	@Override
	public double getRightY() {
			return applyConfig(-getRawAxis(5), mConfigValues.getRightYDeadzone(), mConfigValues.getRightYMinimumOutput(), mConfigValues.getRightYLinearity());
	}

	@Override
	public double getLeftX() {
			return applyConfig(getRawAxis(0), mConfigValues.getLeftXDeadzone(), mConfigValues.getLeftXMinimumOutput(), mConfigValues.getLeftXLinearity());
	}

	@Override
	public double getLeftY() {
			return applyConfig(-getRawAxis(1), mConfigValues.getLeftYDeadzone(), mConfigValues.getLeftYMinimumOutput(), mConfigValues.getLeftYLinearity());
	}
	

	public boolean getLeftTriggerDigital(){
		return getRawAxis(2) > mConfigValues.getTriggerAsDigitalDeadzone();
	}

	public boolean getRightTriggerDigital(){
		return getRawAxis(3) > mConfigValues.getTriggerAsDigitalDeadzone();
	}

	public void updateConfig()
	{
			mConfigValues = mControllerConfigDashboard.getUpdatedConfigValues();
	}

	public XboxControllerConfigValues getConfigValues(){
		return mConfigValues;
	}

	public void configValueDisplay()
	{
		
		mControllerConfigTab.addNumber("Left X", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftX();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(0, 3);

		mControllerConfigTab.addNumber("Left Y", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftY();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(2, 3);

		mControllerConfigTab.addNumber("Right X", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightX();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(4, 3);

		mControllerConfigTab.addNumber("Right Y", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightY();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(6, 3);

		mControllerConfigTab.addNumber("Left Trigger", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftTriggerAxis();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(6, 4);

		mControllerConfigTab.addNumber("Right Trigger", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightTriggerAxis();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS).withSize(2, 1).withPosition(8, 4);
	}

	/**
	 * Makes the controller rumble.
	 * @param l The left rumble value.
	 * @param r The right rumble value.
	 */
	public void rumble(double l, double r) {
		setRumble(RumbleType.kLeftRumble, l);
		setRumble(RumbleType.kRightRumble, r);
	}
}