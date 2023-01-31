package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.NavX;

public class DriveTrain extends SubsystemBase{

    private DifferentialDrive mDifferentialDrive;
    private DifferentialDriveOdometry mDifferentialDriveOdometry;
    private DifferentialDriveKinematics mDifferentialDriveKinematics;
    private MotorControllerGroup leftMotorGroup, rightMotorGroup;

    private CANSparkMax leftLeader, rightLeader, leftFollower, rightFollower;
    private Encoder leftEncoder, rightEncoder;
    private NavX mNavX;

    private double leftEncoderDistance, rightEncoderDistance, leftVelocity, rightVelocity;

    private Field2d mField;

    //Simulation
    private EncoderSim leftEncoderSim, rightEncoderSim;
    DifferentialDrivetrainSim differentialDrivetrainSim;
    SimDouble navXYaw;

    

    public DriveTrain(){

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
        mDifferentialDriveKinematics = new DifferentialDriveKinematics(Constants.kDriveTrainTrackWidthMeters);
        

        leftLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        leftFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        rightLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        rightFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);

        leftEncoder = new Encoder(RobotMap.leftDriveEncoderA, RobotMap.leftDriveEncoderB);
        leftEncoder.setReverseDirection(true);
        rightEncoder = new Encoder(RobotMap.rightDriveEncoderA, RobotMap.rightDriveEncoderB);

        mNavX = new NavX(SPI.Port.kMXP);

        leftEncoderDistance = 0;
        rightEncoderDistance = 0;
        leftVelocity = 0;
        rightVelocity = 0;

        mField = new Field2d();

        mDifferentialDriveOdometry = new DifferentialDriveOdometry(getYaw(), 0, 0);

        //SIMULATION
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
        navXYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
        differentialDrivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 9, 7.5, Units.lbsToKilograms(150.0), Units.inchesToMeters(6), Units.inchesToMeters(27.0), VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    }

    /**
     * Drive in teleop as normal
     * @param throttle Desired movement forward/backward (1 is full forward, -1 is full backward, 0 is no movement)
     * @param steer Desired turning (-1 is full left, 1 is full right, 0 is no turning )
     * @param quickTurn Turn fast?
     */
    public void setDriveMode(double throttle, double steer, boolean quickTurn){
        mDifferentialDrive.curvatureDrive(throttle, steer, quickTurn);
    }

    /**
     * Drive in teleop but slowly
     * @param throttle Desired movement forward/backward (1 is full forward, -1 is full backward, 0 is no movement)
     * @param steer Desired turning (-1 is full left, 1 is full right, 0 is no turning )
     * @param quickTurn Turn fast?
     */
    public void setCreepMode(double throttle, double steer, boolean quickTurn){
        mDifferentialDrive.curvatureDrive(throttle /4 , steer / 2, quickTurn);
    }

    public void setBrakeMode(){
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

    public double getLeftDistanceMeters() {
		return leftEncoderDistance * Constants.kDriveMetersPerCount;
	}

	public double getLeftVelocityMetersPerSec() {
		return leftVelocity * Constants.kDriveMetersPerCount / 60;
	}

	public double getRightDistanceMeters() {
		return rightEncoderDistance * Constants.kDriveMetersPerCount;
	}

	public double getRightVelocityMetersPerSec() {
		return rightVelocity * Constants.kDriveMetersPerCount / 60;
	}

    public DifferentialDriveKinematics getKinematics(){
        return mDifferentialDriveKinematics;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    public void outputVolts(double leftVolts, double rightVolts){
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    public void resetOdometry(Pose2d newPose){
        mDifferentialDriveOdometry.resetPosition(getYaw(), getLeftDistanceMeters(), getRightDistanceMeters(), newPose);
        differentialDrivetrainSim.setPose(newPose);
    }

    public Pose2d getPose(){
        return mDifferentialDriveOdometry.getPoseMeters();
    }

    /**
     * Follow a PathPlanner Trajectory
     * @param traj the trajectory to follow
     * @param isFirstPath is this the first path being run?
     * @return a {@link Command} representing the action to follow the path
     */
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.kDriveTrainKs, Constants.kDriveTrainKv, Constants.kDriveTrainKa),
            this.mDifferentialDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
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
        SmartDashboard.putNumber("Navx Yaw(Degrees):", mNavX.getYaw().getDegrees());
        SmartDashboard.putNumber("Pose X:", getPose().getX());
        SmartDashboard.putNumber("Pose Y:", getPose().getY());
        SmartDashboard.putNumber("Pose Rotation(Degrees):", getPose().getRotation().getDegrees());
        SmartDashboard.putData("Field", mField);
    }

    @Override
    public void periodic() {
        leftEncoderDistance = leftEncoder.getDistance();
        rightEncoderDistance = rightEncoder.getDistance();
        leftVelocity = leftEncoder.getRate();
        rightVelocity = rightEncoder.getRate();
        outputToSmartDashboard();
        mDifferentialDriveOdometry.update(getYaw(), getLeftDistanceMeters(), getRightDistanceMeters());
        mField.setRobotPose(mDifferentialDriveOdometry.getPoseMeters());
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();
        differentialDrivetrainSim.setInputs(-leftLeader.get() * RobotController.getInputVoltage(), rightLeader.get() * RobotController.getInputVoltage());
        differentialDrivetrainSim.update(0.02);
        leftEncoderSim.setDistance(differentialDrivetrainSim.getLeftPositionMeters() / Constants.kDriveMetersPerCount);
        leftEncoderSim.setRate((differentialDrivetrainSim.getLeftVelocityMetersPerSecond() * 60) / Constants.kDriveMetersPerCount);
        rightEncoderSim.setDistance(differentialDrivetrainSim.getRightPositionMeters() / Constants.kDriveMetersPerCount);
        rightEncoderSim.setRate((differentialDrivetrainSim.getRightVelocityMetersPerSecond() * 60) / Constants.kDriveMetersPerCount);
        navXYaw.set(differentialDrivetrainSim.getHeading().getDegrees());

    }
}
