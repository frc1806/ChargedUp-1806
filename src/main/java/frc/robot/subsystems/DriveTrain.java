package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.SWATDifferentialDrive;

public class DriveTrain extends SubsystemBase{

    private SWATDifferentialDrive mDifferentialDrive;
    private DifferentialDrivePoseEstimator mDifferentialDrivePoseEstimator;
    public DifferentialDriveKinematics mDifferentialDriveKinematics;
    private MotorControllerGroup mLeftMotorGroup, mRightMotorGroup;

    private CANSparkMax mLeftLeader, mRightLeader, mLeftFollower, mRightFollower;
    private Encoder mLeftEncoder, mRightEncoder;
    private AHRS mNavX;

    private Field2d mField;
    public Boolean isBrake;

    //Simulation
    private EncoderSim mLeftEncoderSim, mRightEncoderSim;
    DifferentialDrivetrainSim mDifferentialDrivetrainSim;
    SimDouble mNavXYawSim;

    Pose2d mLastVisionUpdate = null;

    public DriveTrain(){

        mLeftLeader = new CANSparkMax(RobotMap.leftLeader, MotorType.kBrushless);
        mLeftFollower = new CANSparkMax(RobotMap.leftFollower, MotorType.kBrushless);
        mRightLeader = new CANSparkMax(RobotMap.rightLeader, MotorType.kBrushless);
        mRightFollower = new CANSparkMax(RobotMap.rightFollower, MotorType.kBrushless);

        mLeftMotorGroup = new MotorControllerGroup(mLeftLeader, mLeftFollower);
        mRightMotorGroup = new MotorControllerGroup(mRightLeader, mRightFollower);

        if(!Constants.isCompBot){
            mLeftMotorGroup.setInverted(true);
            mRightMotorGroup.setInverted(false);
        } else {
            mLeftMotorGroup.setInverted(false);
            mRightMotorGroup.setInverted(true);
        }
        

        mDifferentialDrive = new SWATDifferentialDrive(mLeftMotorGroup, mRightMotorGroup);
        mDifferentialDriveKinematics = new DifferentialDriveKinematics(Constants.kDriveTrainTrackWidthMeters);

        mLeftLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        mLeftFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        mRightLeader.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);
        mRightFollower.setSmartCurrentLimit(Constants.kDriveTrainCurrentLimit);

        mLeftEncoder = new Encoder(RobotMap.leftDriveEncoderA, RobotMap.leftDriveEncoderB);
        mLeftEncoder.setReverseDirection(false);
        mLeftEncoder.setDistancePerPulse(Constants.kDriveMetersPerCount);
        mRightEncoder = new Encoder(RobotMap.rightDriveEncoderA, RobotMap.rightDriveEncoderB);
        mRightEncoder.setDistancePerPulse(Constants.kDriveMetersPerCount);
        mRightEncoder.setReverseDirection(true);

        mNavX = new AHRS();

        mField = new Field2d();

        mDifferentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(mDifferentialDriveKinematics, new Rotation2d(Units.degreesToRadians(-mNavX.getAngle())), 0, 0, new Pose2d());

        //SIMULATION
        mLeftEncoderSim = new EncoderSim(mLeftEncoder);
        mRightEncoderSim = new EncoderSim(mRightEncoder);
        mNavXYawSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
        mDifferentialDrivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 5.9, 7.5, Units.lbsToKilograms(118), Units.inchesToMeters(2), Constants.kDriveTrainTrackWidthMeters, VecBuilder.fill(0.0000, 0.0000, 0.0005, 0.00, 0.00, 0.0000, 0.0000));
        isBrake = false;
    }

    public boolean powerBrake(double power, double turn, double brakePower){
        /*
        if(Math.abs(getWheelSpeeds().leftMetersPerSecond) < Constants.kDriveTrainMinimumMovingSpeed && Math.abs(getWheelSpeeds().rightMetersPerSecond) < Constants.kDriveTrainMinimumMovingSpeed && Math.abs(power) < .005 && Math.abs(turn) < .005){
            mLeftLeader.set(brakePower);
            mLeftFollower.set(-brakePower);
            mRightLeader.set(brakePower);
            mRightFollower.set(-brakePower);
            return true;
        }
        else{
            return false;
        }
        */
        return false;
    }

    public void setDrivePower(Double drivePower){
        mLeftMotorGroup.set(drivePower);
        mRightMotorGroup.set(drivePower);
    }

    /**
     * Drive in teleop as normal
     * @param throttle Desired movement forward/backward (1 is full forward, -1 is full backward, 0 is no movement)
     * @param steer Desired turning (-1 is full right, 1 is full left, 0 is no turning )
     * @param quickTurn Turn fast?
     */
    public void setDriveMode(double throttle, double steer, boolean quickTurn){
        mDifferentialDrive.curvatureDrive(throttle, steer * Constants.kDriveTurningSensitivity, quickTurn);
    }

    public void setTankDrive(double leftThrottle, double rightThrottle){
        mDifferentialDrive.tankDrive(leftThrottle, rightThrottle);
    }

        /**
     * Drive in teleop as normal
     * @param throttle Desired movement forward/backward (1 is full forward, -1 is full backward, 0 is no movement)
     * @param steer Desired turning (-1 is full right, 1 is full left, 0 is no turning )
     * @param quickTurn Turn fast?
     */
    public void setSuperBrakeMode(double throttle, double steer, boolean quickTurn){
        if(!powerBrake(steer, throttle, Constants.kDriveTrainRampPowerBrakePower))
        {
            mDifferentialDrive.curvatureDrive(throttle, steer * Constants.kDriveTurningSensitivity, quickTurn);
        }
        
    }

    /**
     * Drive in teleop but slowly
     * @param throttle Desired movement forward/backward (1 is full forward, -1 is full backward, 0 is no movement)
     * @param steer Desired turning (-1 is full right, 1 is full left, 0 is no turning )
     * @param quickTurn Turn fast?
     */
    public void setCreepMode(double throttle, double steer, boolean quickTurn){
        if(!powerBrake(steer, throttle, Constants.kDriveTrainRampPowerBrakePower))
        {
            mDifferentialDrive.curvatureDrive(throttle / 3.33 , steer * Constants.kDriveTurningSensitivity, quickTurn);
        }
    }

    public void setBrakeMode(){
        isBrake = true;
        mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode(){
        isBrake = false;
        mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void stop(){
        mDifferentialDrive.curvatureDrive(0.0, 0.0, false);
    }

    public DifferentialDriveKinematics getKinematics(){
        return mDifferentialDriveKinematics;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(mLeftEncoder.getRate(), mRightEncoder.getRate());
    }

    public void outputVolts(double leftVolts, double rightVolts){
        mDifferentialDrive.tankDrive(leftVolts/RobotController.getInputVoltage(), rightVolts/RobotController.getInputVoltage(), false);
    }

    public synchronized void resetOdometry(Pose2d newPose){
        resetEncoders();
        mNavX.reset();
        mNavX.setAngleAdjustment(-newPose.getRotation().getDegrees());
        mNavXYawSim.set(-newPose.getRotation().getDegrees()); //SIM
        mDifferentialDrivetrainSim.setPose(newPose);
        mDifferentialDrivePoseEstimator.resetPosition(new Rotation2d(Units.degreesToRadians(-mNavX.getAngle())), mLeftEncoder.getDistance(), mRightEncoder.getDistance(), newPose);
        
        
    }

    public void resetEncoders(){
        mLeftEncoder.reset();
        mRightEncoder.reset();
    }

    public Pose2d getPose(){
        return mDifferentialDrivePoseEstimator.getEstimatedPosition();
    }

    public Double getLeftDrivePower(){
        return mLeftMotorGroup.get();
    }

    public Double getRightDrivePower(){
        return mRightMotorGroup.get();
    }

    public CANSparkMax getLeftLeader(){
        return mLeftLeader;
    }

    public CANSparkMax getRightLeader(){
        return mRightLeader;
    }

    public CANSparkMax getLeftFollower(){
        return mLeftFollower;
    }

    public CANSparkMax getRightFollower(){
        return mRightFollower;
    }

    public float getRobotPitch(){
        return mNavX.getRoll();
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
                Pose2d startPose = traj.getInitialPose();
                if(RobotContainer.S_REAR_VISION_SUBSYSTEM.getCurrentAlliance() == Alliance.Red)
                {
                    startPose = new Pose2d(new Translation2d(startPose.getX(), Constants.kFieldWidth - startPose.getY()), startPose.getRotation());
                }
              this.resetOdometry(startPose);
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.kDriveTrainKs, Constants.kDriveTrainKv, Constants.kDriveTrainKa),
            this.mDifferentialDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0.5, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0.5, 0, 0), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Paths should be made for the Blue Alliance to be mirrored.
            this // Requires this drive subsystem
        )
    );
}

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Left Encoder Distance", mLeftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder Distance", mRightEncoder.getDistance());
        SmartDashboard.putNumber("Left Encoder Rate", mLeftEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder Rate", mRightEncoder.getRate());
        SmartDashboard.putNumber("Navx Yaw(Degrees):", mNavX.getAngle());
        SmartDashboard.putNumber("Pose X:", getPose().getX());
        SmartDashboard.putNumber("Pose Y:", getPose().getY());
        SmartDashboard.putNumber("Pose Rotation(Degrees):", getPose().getRotation().getDegrees());
        SmartDashboard.putData("Field", mField);
        SmartDashboard.putNumber("Left Applied Output %", mLeftMotorGroup.get());
        SmartDashboard.putNumber("Right Applied Output %", mRightMotorGroup.get());
        SmartDashboard.putNumber("Robot Pitch", getRobotPitch());
    }

    public void forceVisionUpdate(){
        Pose2d visionUpdate = RobotContainer.S_REAR_VISION_SUBSYSTEM.getBotPose();
        if(RobotContainer.S_REAR_VISION_SUBSYSTEM.hasAprilTagTarget()){
            mLastVisionUpdate = visionUpdate;
            mDifferentialDrivePoseEstimator.resetPosition(new Rotation2d(Units.degreesToRadians(-mNavX.getAngle())), mLeftEncoder.getDistance(), mRightEncoder.getDistance(), RobotContainer.S_REAR_VISION_SUBSYSTEM.getBotPose());
        }
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
        mDifferentialDrivePoseEstimator.update(new Rotation2d(Units.degreesToRadians(-mNavX.getAngle())), mLeftEncoder.getDistance(), mRightEncoder.getDistance());
        Pose2d fieldZero = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        if(RobotContainer.S_REAR_VISION_SUBSYSTEM.getCurrentAlliance() == Alliance.Red){
            fieldZero = new Pose2d(new Translation2d(Constants.kFieldLength, Constants.kFieldWidth), new Rotation2d(Math.PI));
        }
        mField.setRobotPose(mDifferentialDrivePoseEstimator.getEstimatedPosition().relativeTo(fieldZero));

        Pose2d visionUpdate = RobotContainer.S_REAR_VISION_SUBSYSTEM.getBotPose();
        if(visionUpdate != null && mLastVisionUpdate == null){
            forceVisionUpdate();
        }
        else if(RobotContainer.S_REAR_VISION_SUBSYSTEM.hasAprilTagTarget() && visionUpdate != mLastVisionUpdate){
            if(visionUpdate.getTranslation().getDistance(mDifferentialDrivePoseEstimator.getEstimatedPosition().getTranslation()) < 3.0){
                mDifferentialDrivePoseEstimator.addVisionMeasurement(visionUpdate, Timer.getFPGATimestamp() - 0.2);
                mLastVisionUpdate = visionUpdate;
            } // I don't see our odometry being off an entire meter.
           
        }

        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        mDifferentialDrivetrainSim.setInputs(mDifferentialDrive.getWheelSpeeds().left * RobotController.getInputVoltage(), mDifferentialDrive.getWheelSpeeds().right * RobotController.getInputVoltage());
        mDifferentialDrivetrainSim.update(0.02);
        mLeftEncoderSim.setDistance( mDifferentialDrivetrainSim.getLeftPositionMeters());
        mLeftEncoderSim.setRate(mDifferentialDrivetrainSim.getLeftVelocityMetersPerSecond());
        mRightEncoderSim.setDistance(mDifferentialDrivetrainSim.getRightPositionMeters());
        mRightEncoderSim.setRate(mDifferentialDrivetrainSim.getRightVelocityMetersPerSecond());
        mNavXYawSim.set(-mDifferentialDrivetrainSim.getHeading().getDegrees()); //this has to be negative because the record odometry command flipps from clockwise positive to ccw positive

    }
}
