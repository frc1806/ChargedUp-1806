package frc.robot.util;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class SWATDifferentialDrive extends DifferentialDrive {

    private static int instances;

    private final MotorControllerGroup m_leftMotor;
    private final MotorControllerGroup m_rightMotor;
  
    private boolean m_reported;

    public static <T> T requireNonNullParam(T obj, String paramName, String methodName) {
        return requireNonNull(
            obj,
            "Parameter "
                + paramName
                + " in method "
                + methodName
                + " was null when it"
                + " should not have been!  Check the stacktrace to find the responsible line of code - "
                + "usually, it is the first line of user-written code indicated in the stacktrace.  "
                + "Make sure all objects passed to the method in question were properly initialized -"
                + " note that this may not be obvious if it is being called under "
                + "dynamically-changing conditions!  Please do not seek additional technical assistance"
                + " without doing this first!");
    }

    public SWATDifferentialDrive(MotorControllerGroup leftMotor, MotorControllerGroup rightMotor) {
        super(leftMotor, rightMotor);
        requireNonNullParam(leftMotor, "leftMotor", "DifferentialDrive");
        requireNonNullParam(rightMotor, "rightMotor", "DifferentialDrive");
    
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        SendableRegistry.addChild(this, m_leftMotor);
        SendableRegistry.addChild(this, m_rightMotor);
        instances++;
        SendableRegistry.addLW(this, "DifferentialDrive", instances);
    }

    @Override
    public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
        //WHY DOES THIS EXIST??? This class exists to use setVoltage instead of set for better driving.
        if (!m_reported) {
            HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialCurvature, 2);
            m_reported = true;
        }

        allowTurnInPlace = xSpeed == 0.0?true:allowTurnInPlace;
    
        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
        zRotation = MathUtil.applyDeadband(zRotation, m_deadband);
    
        var speeds = curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace);
    
        m_leftMotor.setVoltage(speeds.left * 12);
        m_rightMotor.setVoltage(speeds.right * 12);
    
        feed();
    }
}
