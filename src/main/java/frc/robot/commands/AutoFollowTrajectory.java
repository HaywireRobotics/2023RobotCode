package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Vector;

public class AutoFollowTrajectory extends CommandBase{
    
    private final Trajectory m_trajectory;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ProfiledPIDController headingPID;

    private final double targetHeading;

    private final Timer timer;

    private final double kP = 0.0; // Proportional gain
    private final double kV = 0.1; // Velocity gain
    private final double kA = 0.0; // Acceleration gain

    private final double HEADING_KP = 0.04;
    private final double HEADING_KI = 0;
    private final double HEADING_KD = 0;
    private final double HEADING_MAX_ACC = 200.0;
    private final double HEADING_MAX_VEL = 300.0;

    private double lastPower = 0.0;
    private final double ON_TARGET_MAX_POWER = 0.1;

    public AutoFollowTrajectory(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory, double targetHeading) {
        this.m_trajectory = trajectory;
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        timer = new Timer();
        this.targetHeading = targetHeading;

        double smoothedHeadingVelocity = Math.abs(m_drivetrainSubsystem.getPose().getRotation().getDegrees()-targetHeading)/trajectory.getTotalTimeSeconds();

        TrapezoidProfile.Constraints headingProfile =  new TrapezoidProfile.Constraints(Math.min(HEADING_MAX_VEL, smoothedHeadingVelocity), HEADING_MAX_ACC);
        this.headingPID = new ProfiledPIDController(HEADING_KP, HEADING_KI, HEADING_KD,
                                                    headingProfile, 0.02);

        timer.reset();
        timer.start();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double time = Math.min(timer.get(), m_trajectory.getTotalTimeSeconds());
        Trajectory.State state = m_trajectory.sample(time);
        
        double velocityHeading = state.poseMeters.getRotation().getDegrees();
        double positionError = state.poseMeters.getTranslation().getDistance(m_drivetrainSubsystem.getPose().getTranslation());

        Vector ffVector = Vector.fromAngle(velocityHeading, kV * state.velocityMetersPerSecond + kA * state.accelerationMetersPerSecondSq);
        Vector proportionalVector = Vector.fromTranslation(state.poseMeters.getTranslation().minus(m_drivetrainSubsystem.getPose().getTranslation())).scale(kP);
        Vector driveVector = proportionalVector.add(ffVector);

        double currentHeading = m_drivetrainSubsystem.getPose().getRotation().getDegrees();
        double headingOutput = headingPID.calculate(currentHeading, targetHeading);

        m_drivetrainSubsystem.driveVectorMetersPerSecond(driveVector.magnitude(), driveVector.direction(), headingOutput);

        lastPower = driveVector.magnitude();

        SmartDashboard.putNumber("Trajectory: Heading Error", headingOutput);
        SmartDashboard.putNumber("Trajectory: Position Error", positionError);
        SmartDashboard.putNumber("Trajectory: kV*m/s", kV * state.velocityMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return false;//timer.get() >= m_trajectory.getTotalTimeSeconds() && Math.abs(lastPower) < ON_TARGET_MAX_POWER;
    }
}
