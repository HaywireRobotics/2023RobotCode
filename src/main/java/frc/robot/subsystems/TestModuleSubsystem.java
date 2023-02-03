package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;
import frc.robot.wrappers.SwerveModule;

public class TestModuleSubsystem extends SubsystemBase {
    public final SwerveModule module;
    public SwerveModuleState currentState;
    public boolean stopped = false;
    public static final SwerveModuleState restState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    public TestModuleSubsystem() {
        NEO FRdriveMotor = new NEO(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        NEO FRrotationMotor = new NEO(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR);
        CANCoder FRrotationEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

        NEO FLdriveMotor = new NEO(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
        NEO FLrotationMotor = new NEO(Constants.FRONT_LEFT_MODULE_STEER_MOTOR);
        CANCoder FLrotationEncoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);

        this.module = new SwerveModule(FRdriveMotor, FRrotationMotor, FRrotationEncoder, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        this.currentState = restState;
    }

    public void setState(SwerveModuleState state) {
        stopped = false;
        currentState = state;
    }

    public void restModule() {
        stopped = false;
        currentState = restState;
    }

    public void stopModule() {
        stopped = true;
        module.driveDirect(0.0, 0.0);
    }

    public void periodic() {
        // System.out.println(module.getRotation());
        if (!stopped) {
            module.setState(currentState);
        }
        
        // This is stuff from SDS code. I'm not sure exactly what's going on but I think it's the next step
        // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        // SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        // m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    }
    
}
