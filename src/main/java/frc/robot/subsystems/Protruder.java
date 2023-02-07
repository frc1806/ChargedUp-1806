package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.game.Placement;

public class Protruder extends SubsystemBase{

    private CANSparkMax mProtrusionMotor;
    private Encoder mEncoder;
    private enum ProtruderStates {
        Idle,
        Extending,
        Calculating
    };
    private Placement currentPlacement;

    private ProtruderStates mProtruderStates;
    
    public Protruder(){
        mProtrusionMotor = new CANSparkMax(RobotMap.protrusionMotor, MotorType.kBrushless);
        mEncoder = new Encoder(RobotMap.protrusionMotorA, RobotMap.ProtrusionMotorB);
        mProtruderStates = ProtruderStates.Idle;
        currentPlacement = new Placement(0.0,0.0,0.0,0.0);
    }

    public Placement getCurrentPlacement(){
        return currentPlacement;
    }

    public void setCurrentPlacement(Placement placement){
        currentPlacement = placement;
    }

    public void goToExtension(Placement placement) {
        mProtruderStates = ProtruderStates.Extending;
        setMotor(placement.getExtendSpeed());
        mProtruderStates = ProtruderStates.Calculating;
    }

    public boolean checkIfAtPosition(Placement placement){
        if(getEncoderDistance() > placement.getExtendDistance()){
            return true;
        }
        return false;
    }

    public void setMotor(Double number){
        mProtruderStates = ProtruderStates.Extending;
        mProtrusionMotor.setVoltage(number);
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
        SmartDashboard.putNumber("Protrusion applied output: ", mProtrusionMotor.getAppliedOutput());
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
