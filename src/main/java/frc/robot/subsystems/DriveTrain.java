package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.NavX;

public class DriveTrain extends SubsystemBase{

    private DriveTrain mDriveTrain;
    private DifferentialDrive mDifferentialDrive;
    private MotorControllerGroup leftMotorGroup, rightMotorGroup;

    private CANSparkMax leftLeader, rightLeader, leftFollower, rightFollower;
    private Encoder leftEncoder, rightEncoder;
    private NavX mNavX;

    private double leftEncoderDistance, rightEncoderDistance, leftVelocity, rightVelocity;

    private DriveTrain(){
        mDriveTrain = new DriveTrain();

        leftLeader = new CANSparkMax(RobotMap.leftLeaderID, MotorType.kBrushless);
        leftFollower = new CANSparkMax(RobotMap.leftFollowerID, MotorType.kBrushless);
        rightLeader = new CANSparkMax(RobotMap.rightLeaderID, MotorType.kBrushless);
        rightFollower = new CANSparkMax(RobotMap.rightFollowerID, MotorType.kBrushless);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightFollower);

        leftMotorGroup = new MotorControllerGroup(leftLeader, leftFollower);
        rightMotorGroup = new MotorControllerGroup(rightLeader, rightFollower);

        leftMotorGroup.setInverted(true);
        rightMotorGroup.setInverted(false);

        mDifferentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        leftLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        leftFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        rightLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        rightFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);

        leftEncoder = new Encoder(RobotMap.leftDriveEncoderA, RobotMap.leftDriveEncoderB);
        leftEncoder = new Encoder(RobotMap.rightDriveEncoderA, RobotMap.rightDriveEncoderB);

        mNavX = new NavX(SPI.Port.kMXP);

        leftEncoderDistance = 0;
        rightEncoderDistance = 0;
        leftVelocity = 0;
        rightVelocity = 0;
    }

    public DriveTrain getInstance(){
        return mDriveTrain;
    }

    public void setDriveMode(double leftY, double rightX, boolean quickTurn){
        mDifferentialDrive.curvatureDrive(leftY * 12, rightX * 12, quickTurn);
    }

    public void setCreepMode(double leftY, double rightX, boolean quickTurn){
        mDifferentialDrive.curvatureDrive(leftY * 3, rightX * 6, quickTurn);
    }

    public void setIdleMode(){
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode(){
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void stop(){
        leftLeader.setVoltage(0);
        rightLeader.setVoltage(0);
    }

    public double getLeftDistanceInches() {
		return leftEncoderDistance * Constants.kDriveInchesPerCount;
	}

	public double getLeftVelocityInchesPerSec() {
		return leftVelocity * Constants.kDriveInchesPerCount / 60;
	}

	public double getRightDistanceInches() {
		return rightEncoderDistance * Constants.kDriveInchesPerCount;
	}

	public double getRightVelocityInchesPerSec() {
		return rightVelocity * Constants.kDriveInchesPerCount / 60;
	}

    public Rotation2d getYaw(){
        return mNavX.getYaw();
    }

    public void setYaw(Rotation2d angle){
        mNavX.zeroYaw();
        mNavX.setAngleAdjustment(angle);
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Left Encoder Distance", leftEncoderDistance);
        SmartDashboard.putNumber("Right Encoder Distance", rightEncoderDistance);
    }

    @Override
    public void periodic() {
        leftEncoderDistance = leftEncoder.getDistance();
        rightEncoderDistance = rightEncoder.getDistance();
        outputToSmartDashboard();
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();
    }
}
