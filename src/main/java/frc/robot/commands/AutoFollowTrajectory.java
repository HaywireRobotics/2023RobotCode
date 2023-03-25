package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class AutoFollowTrajectory extends CommandBase{
    
    private final PathPlannerTrajectory m_pathPlannerTrajectory;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ProfiledPIDController headingPID;

    private final Timer timer;

    private final double kP = 0.4;//0.5; // Proportional gain
    private final double kV = 0.40; // Velocity gain
    private final double kA = 0.15; // Acceleration gain

    private final double HEADING_KP = 0.04;
    private final double HEADING_KI = 0;
    private final double HEADING_KD = 0;
    private final double HEADING_KF = 0.9;
    private final double HEADING_MAX_ACC = 200.0;
    private final double HEADING_MAX_VEL = 300.0;

    private double lastPower = 0.0;
    private final double ON_TARGET_MAX_POWER = 0.1;

    public AutoFollowTrajectory(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory trajectory) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        timer = new Timer();
        this.m_pathPlannerTrajectory = trajectory;
        double targetHeading = trajectory.getEndState().poseMeters.getRotation().getDegrees();
        // double smoothedHeadingVelocity = Math.abs(m_drivetrainSubsystem.getPose().getRotation().getDegrees() - targetHeading)/trajectory.getTotalTimeSeconds();

        TrapezoidProfile.Constraints headingProfile =  new TrapezoidProfile.Constraints(HEADING_MAX_VEL, HEADING_MAX_ACC); //Math.min(HEADING_MAX_VEL, smoothedHeadingVelocity)
        this.headingPID = new ProfiledPIDController(HEADING_KP, HEADING_KI, HEADING_KD,
                                                    headingProfile, 0.02);


        addRequirements(drivetrainSubsystem);

        SmartDashboard.putNumber("Trajectory: Heading", 0);
        double[] positionErrorArray = {0,0};
        SmartDashboard.putNumberArray("Trajectory: Position Error", positionErrorArray);
        // SmartDashboard.putNumberArray("Trajectory: Target Position", positionErrorArray);
        SmartDashboard.putNumber("Trajectory: Time", 0);

        SmartDashboard.putNumber("Trajectory: mps", 0);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        m_drivetrainSubsystem.updateOdometry();
        PathPlannerState startSample = (PathPlannerState) m_pathPlannerTrajectory.getInitialState();
        m_drivetrainSubsystem.resetGyroscope(0);
        m_drivetrainSubsystem.resetPose(startSample.poseMeters.getX(), startSample.poseMeters.getY(), startSample.holonomicRotation.getDegrees());
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.updateOdometry();
        double time = Math.min(timer.get(), m_pathPlannerTrajectory.getTotalTimeSeconds());
        // Trajectory.State state = m_trajectory.sample(time);
        PathPlannerState state = (PathPlannerState) m_pathPlannerTrajectory.sample(time);

        double velocityHeading = state.poseMeters.getRotation().getDegrees();
        Translation2d positionErrorTranslation = state.poseMeters.getTranslation().minus(m_drivetrainSubsystem.getPose().getTranslation());
        double positionError = state.poseMeters.getTranslation().getDistance(m_drivetrainSubsystem.getPose().getTranslation());

        Vector ffVector = Vector.fromAngle(Math.toRadians(velocityHeading), kV * state.velocityMetersPerSecond + kA * state.accelerationMetersPerSecondSq).scale(-1);
        Vector proportionalVector = Vector.fromTranslation(positionErrorTranslation).scale(-kP);
        Vector driveVector = proportionalVector.add(ffVector);
        driveVector = new Vector(driveVector.x, -driveVector.y);

        double currentHeading = m_drivetrainSubsystem.getPose().getRotation().getDegrees();
        double headingError = Statics.angleDifference(currentHeading, state.holonomicRotation.getDegrees());
        // double headingOutput = headingPID.calculate(currentHeading, targetHeading);
        double headingOutput = headingPID.calculate(headingError, 0);//+state.holonomicAngularVelocityRadPerSec*HEADING_KF;
        

        m_drivetrainSubsystem.driveVector(driveVector.magnitude(), Math.toDegrees(driveVector.direction()), headingOutput);

        lastPower = driveVector.magnitude();

        SmartDashboard.putNumber("Trajectory: Heading", state.holonomicRotation.getDegrees());
        // SmartDashboard.putNumber("Trajectory: Position Error", positionError);
        SmartDashboard.putNumber("Trajectory: Time", time);
        double[] positionErrorArray = {positionErrorTranslation.getX(), positionErrorTranslation.getY()};
        double[] targetPositionArray = {state.poseMeters.getTranslation().getX(), state.poseMeters.getTranslation().getY()};
        SmartDashboard.putNumberArray("Trajectory: Position Error", positionErrorArray);
        // SmartDashboard.putNumberArray("Trajectory: Target Position", targetPositionArray);

        SmartDashboard.putNumber("Trajectory: mps", state.velocityMetersPerSecond);
        
    }

    @Override
    public boolean isFinished() {
        return false;//timer.get() >= m_trajectory.getTotalTimeSeconds() && Math.abs(lastPower) < ON_TARGET_MAX_POWER;
    }
}
