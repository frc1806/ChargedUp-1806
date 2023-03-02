package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class LEDs extends SubsystemBase {

    private AddressableLED mLEDStrip;
    private AddressableLEDBuffer mLEDBuffer;

    public LEDs(){

        mLEDStrip = new AddressableLED(RobotMap.LEDStrip);
        mLEDBuffer = new AddressableLEDBuffer(Constants.kTotalLEDCount);
        
        mLEDStrip.setLength(mLEDBuffer.getLength());

        mLEDStrip.setData(mLEDBuffer);
        mLEDStrip.start();


    }


    public void setLEDsOff(){



          //this will set the leds to black which will also be off
         
          for(var i = 0; i < mLEDBuffer.getLength(); i++) {

            mLEDBuffer.setRGB(i, 0, 0, 0);

        }



    }





    public void setConeLEDOn(){

        //this will set the leds to an orange color

        for(var i = 0; i < mLEDBuffer.getLength(); i++) {

            mLEDBuffer.setRGB(i, 255, 129, 3);

        }




    }


    public void setCubeLEDOn(){


        //this will set the leds to a purple color
        
        for(var i = 0; i < mLEDBuffer.getLength(); i++ ) {

            mLEDBuffer.setRGB(i, 154,  3, 255);
        }


        
    }


    @Override
    public void periodic(){
        if(RobotContainer.GetCurrentGamePieceMode() == RobotContainer.GamePieceMode.CubeMode){
            setCubeLEDOn();
        }
        if (RobotContainer.GetCurrentGamePieceMode() == RobotContainer.GamePieceMode.ConeMode){
            setConeLEDOn();
        }
        if (RobotContainer.GetCurrentGamePieceMode() == RobotContainer.GamePieceMode.OffMode){
            setLEDsOff();
        }
    }



}