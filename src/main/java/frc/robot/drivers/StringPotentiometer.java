package frc.robot.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class StringPotentiometer {
    
    AnalogInput mAnalogInput;


    public StringPotentiometer(int analogChannel){
        mAnalogInput = new AnalogInput(analogChannel);
    }

    public double getExtensionInInches(){
        return 34 - 6.606 * mAnalogInput.getVoltage();
    }
}
