package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.game.Placement;

public class Protruder extends SubsystemBase{

    private TalonSRX mProtrusionMotor;
    
    private Encoder mEncoder;
    private enum ProtruderStates {
        Idle,
        Extending,
        Calculating
    };
    private Placement currentPlacement;
    private PIDController mPidController;
    private ProtruderStates mProtruderStates;
    private Double encoderSnapshot;
    
    public Protruder(){
        mProtrusionMotor = new TalonSRX(RobotMap.protrusionMotor);
        mProtruderStates = ProtruderStates.Idle;
        currentPlacement = new Placement(0.0,0.0);
        mPidController = new PIDController(Constants.kProtruderkP, Constants.kProtruderkI, Constants.kProtruderkD);
        encoderSnapshot = 0.0;
    }

    public Placement getCurrentPlacement(){
        return currentPlacement;
    }

    public void setCurrentPlacement(Placement placement){
        currentPlacement = placement;
    }

    public void goToExtension(Placement placement) {
        mProtruderStates = ProtruderStates.Extending;
        encoderSnapshot = getEncoderDistance();
        mProtrusionMotor.set(TalonSRXControlMode.PercentOutput, mPidController.calculate(placement.getExtendDistance()));
        mProtruderStates = ProtruderStates.Calculating;
    }

    public boolean checkIfAtPosition(Placement placement){
        if((getEncoderDistance() - encoderSnapshot) >= placement.getExtendDistance()){
            return true;
        }
        return false;
    }

    public void setMotor(Double number){
        mProtruderStates = ProtruderStates.Extending;
        mProtrusionMotor.set(TalonSRXControlMode.PercentOutput,number);
    }

    public void stop(){
        mProtruderStates = ProtruderStates.Idle;
        setMotor(0.0);
    }

    public void zeroSensors(){
        mEncoder.reset();
    }

    private Double getEncoderDistance(){
        return mEncoder.getDistance();
    }

    private void outputToSmartDashboard(){
        SmartDashboard.putString("Protruder state: ", mProtruderStates.toString());
        SmartDashboard.putNumber("Protrusion applied output: ", mProtrusionMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Protrusion encoder distance: ", mEncoder.getDistance());
    }

    @Override
    public void periodic() {
        if(mProtruderStates == ProtruderStates.Calculating){
            if (checkIfAtPosition(currentPlacement)){
                mProtruderStates = ProtruderStates.Idle;
                stop();
            }
        }
        outputToSmartDashboard();
    }

    @Override
    public void simulationPeriodic() {
    }
    


}
