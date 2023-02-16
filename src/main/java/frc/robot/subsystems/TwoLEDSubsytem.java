package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TwoLEDSubsytem extends SubsystemBase {
   
   private CANdle mGPCANdle;
   private CANdleConfiguration mGPConfig;


   public TwoLEDSubsytem(){
    
    mGPCANdle = new CANdle(1);

    mGPConfig = new CANdleConfiguration();
    mGPConfig.stripType = LEDStripType.RGB;
    mGPConfig.brightnessScalar = 1.0;
    mGPCANdle.configAllSettings(mGPConfig);

    mGPCANdle.setLEDs(255, 129, 3, 0, 1, 12); //Cone LED
    mGPCANdle.setLEDs(154,  3, 255, 0, 13, 12); //Cube LED


    }



}
