package frc.robot.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class StringPotentiometer {
    
    AnalogInput mAnalogInput;


    public StringPotentiometer(int analogChannel){
        mAnalogInput = new AnalogInput(analogChannel);
    }

    public double getExtensionInInches(){
        //tuned for our arm with a slightly off kilter mount, if using this, retune
        return  (-42.53 * mAnalogInput.getVoltage())+ 16.4591 +46.50;
    }

    public double getRawVoltage(){
        return mAnalogInput.getVoltage();
    }
}
