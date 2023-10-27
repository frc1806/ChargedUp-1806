package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED extends SubsystemBase {
   
   private CANdle mGPCANdle;
   private CANdleConfiguration mGPConfig;
   private ColorFlowAnimation mCubeAnimation;
   private ColorFlowAnimation mConeAnimation;
   private TwinkleAnimation mTestAnim;
   private RainbowAnimation mRainbowAnimation;
   private FireAnimation mIdleAnimation;
   private LarsonAnimation mLarsonAnimation;
   private GamePieceMode lastGamePieceMode;
   private boolean wasDisabled;
   private boolean startup = true;
   private ErrorCode faultsError, error;


   public LED(){
        mGPCANdle = new CANdle(0);
        mGPConfig = new CANdleConfiguration();

        mGPConfig.stripType = LEDStripType.GRB;
        mGPConfig.brightnessScalar = 1.0;
        mGPConfig.enableOptimizations = true;
        mGPConfig.v5Enabled = false;
        mGPConfig.vBatOutputMode = VBatOutputMode.Off;

        mGPCANdle.configAllSettings(mGPConfig);

        mConeAnimation = new ColorFlowAnimation(255, 129, 3, 0, 0.69, 180,  Direction.Forward);
        mCubeAnimation = new ColorFlowAnimation(255, 0, 255, 0, 0.69, 180, Direction.Forward);   
        mIdleAnimation = new FireAnimation(.7, 0.3, 180, .7, .3);
        mTestAnim = new TwinkleAnimation(0, 255, 0, 0, 0.50, 180, TwinklePercent.Percent42);
        mRainbowAnimation = new RainbowAnimation(1, 0.69, 180);
        mLarsonAnimation = new LarsonAnimation(255,255,255,0,0.69,180,BounceMode.Front,65);
        
        
        error = mGPCANdle.getLastError(); // gets the last error generated by the CANdle
        CANdleFaults faults = new CANdleFaults();
        faultsError = mGPCANdle.getFaults(faults); // fills faults with the current CANdle faults; returns the last error generated
        lastGamePieceMode = GamePieceMode.OffMode;
        wasDisabled = true;
        mGPCANdle.setLEDs(255, 255, 255, 255, 0, 58);
        setIdleAnimation();
    }

    public void setIdleAnimation(){
        mGPCANdle.animate(mIdleAnimation);
    }

    public void setConeAnim(){
        mGPCANdle.animate(mConeAnimation);
    }

    public void setCubeAnim(){
        mGPCANdle.animate(mCubeAnimation);
    }

    public void setTestAnim(){
        mGPCANdle.animate(mTestAnim);
    }

    public void setRainbowAnim(){
        mGPCANdle.animate(mRainbowAnimation);
    }

    public void setLarsonAnim(){
        mGPCANdle.animate(mLarsonAnimation);
    }

    public ErrorCode getFaultsError(){
        return faultsError;
    }

    public ErrorCode getLastError(){
        return error;
    }
    
    @Override
    public void periodic(){
        if(startup){
            setIdleAnimation();
            startup = false;
        }
        GamePieceMode currentGamePieceMode = RobotContainer.GetCurrentGamePieceMode();
        boolean isDisabled = RobotState.isDisabled();
        if(currentGamePieceMode != lastGamePieceMode || (wasDisabled && !isDisabled)){
            mGPCANdle.clearAnimation(0);
            if(currentGamePieceMode == RobotContainer.GamePieceMode.CubeMode){
                setCubeAnim();
            }
            if (currentGamePieceMode == RobotContainer.GamePieceMode.ConeMode){
                setConeAnim();
            }
            if (currentGamePieceMode == RobotContainer.GamePieceMode.OffMode){
                setIdleAnimation();
            }
            if(RobotState.isDisabled()){
                setIdleAnimation();
            }
            
        }
        if(!wasDisabled && isDisabled){
            mGPCANdle.clearAnimation(0);
            setIdleAnimation();
        }
        wasDisabled = isDisabled;
        lastGamePieceMode = currentGamePieceMode;
    }



}
