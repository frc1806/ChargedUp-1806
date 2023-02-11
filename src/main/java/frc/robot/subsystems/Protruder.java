package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.game.Placement;

public class Protruder extends SubsystemBase{

    private TalonSRX mProtrusionMotorA, mProtrusionMotorB;
    private enum ProtruderStates {
        Idle,
        Extending,
        Calculating
    };
    private Placement currentPlacement;
    private PIDController mPidController;
    private ProtruderStates mProtruderStates;
    private AnalogPotentiometer mPotentiometer;
    private Double encoderSnapshot;
    
    public Protruder(){
        mProtrusionMotorA = new TalonSRX(RobotMap.protrusionMotor);
        mProtrusionMotorB = new TalonSRX(RobotMap.protrusionMotorB);
        mPotentiometer = new AnalogPotentiometer(RobotMap.protrusionStringPotentiometer);
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
        encoderSnapshot = getDistance();
        setMotors(placement.getExtendDistance());
        mProtruderStates = ProtruderStates.Calculating;
    }

    public boolean checkIfAtPosition(Placement placement){
        if((getDistance() - encoderSnapshot) >= placement.getExtendDistance()){
            return true;
        }
        return false;
    }

    public void setMotors(Double number){
        mProtruderStates = ProtruderStates.Extending;
        mProtrusionMotorA.set(TalonSRXControlMode.PercentOutput,mPidController.calculate(number));
        mProtrusionMotorB.set(TalonSRXControlMode.PercentOutput,mPidController.calculate(number));
    }

    public void stop(){
        mProtruderStates = ProtruderStates.Idle;
        mProtrusionMotorA.set(TalonSRXControlMode.Disabled, 0.0);
        mProtrusionMotorB.set(TalonSRXControlMode.Disabled, 0.0);
    }

    private Double getDistance(){
        return mPotentiometer.get();
    }

    public AnalogPotentiometer getPotentiometer(){
        return mPotentiometer;
    }

    public TalonSRX getMotorA(){
        return mProtrusionMotorA;
    }

    public TalonSRX getMotorB(){
        return mProtrusionMotorB;
    }

    @Override
    public void periodic() {
        if(mProtruderStates == ProtruderStates.Calculating){
            if (checkIfAtPosition(currentPlacement)){
                mProtruderStates = ProtruderStates.Idle;
                stop();
            }
        }
    }

    @Override
    public void simulationPeriodic() {
    }
    


}
