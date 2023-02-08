package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveToTarget extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private Pose2d targetPose;
    // private double translationAcc = 0.05;
    // private double targetSpeed = 0.0;
    // private double speed = 0.0;
    // private double powerPerMeter = 1.0/3.0;
    // private double powerPerDegree = 1.0/180.0;
    // private double maxSpeed = 0.8;

    private final double TRANSLATION_KP = 1.5;
    private final double TRANSLATION_KI = 0;
    private final double TRANSLATION_KD = 0.0;
    private final double TRANSLATION_MAX_ACC = 4;
    private final double TRANSLATION_MAX_VEL = 9;
    private final double HEADING_KP = 0.04;
    private final double HEADING_KI = 0;
    private final double HEADING_KD = 0;
    private final double HEADING_MAX_ACC = 200.0;
    private final double HEADING_MAX_VEL = 300.0;

    private final ProfiledPIDController translationPID;
    private final ProfiledPIDController headingPID;
    
    public AutoDriveToTarget(DrivetrainSubsystem subsystem) {
        this(subsystem, new Pose2d());
    }
    public AutoDriveToTarget(DrivetrainSubsystem subsystem, Pose2d target) {
        this.m_subsystem = subsystem;
        this.targetPose = target;
        
        this.translationPID = new ProfiledPIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD,
            new TrapezoidProfile.Constraints(TRANSLATION_MAX_VEL, TRANSLATION_MAX_ACC), 0.02);
        this.headingPID = new ProfiledPIDController(HEADING_KP, HEADING_KI, HEADING_KD,
            new TrapezoidProfile.Constraints(HEADING_MAX_VEL, HEADING_MAX_ACC), 0.02);

        this.addRequirements(subsystem);
    }

    public void setTarget(Pose2d target){
        targetPose = target;
    }
    @Override
    public void execute() {
        Transform2d offset = targetPose.minus(m_subsystem.getPose());
        double offset_mag = this.getPositionError();
        double angle = Math.toDegrees(Math.atan2(offset.getX(), offset.getY()))-m_subsystem.getPose().getRotation().getDegrees();

        double output_translation = translationPID.calculate(offset_mag, 0);
        // double output_translation = 0.0;

        SmartDashboard.putNumber("Offset", offset_mag);
        // SwerveModuleState translate = new SwerveModuleState(output_translation*Constants.MAX_SPEED, Rotation2d.fromRadians(angle));


        // double rawSpeed = Math.min(offset_mag * powerPerMeter, maxSpeed);
        // speed += Math.min(rawSpeed-speed, translationAcc);
        // System.out.println("speed: "+speed);
        // SwerveModuleState translate = new SwerveModuleState(speed*Constants.MAX_SPEED, Rotation2d.fromRadians(angle));
        double currentHeading = m_subsystem.getPose().getRotation().getDegrees();
        double targetHeading = targetPose.getRotation().getDegrees();
        double angle_offset = targetHeading - currentHeading;
        double output_heading = headingPID.calculate(currentHeading, targetHeading);
        // double output_heading = 0.0;

        SmartDashboard.putNumber("setpoint", translationPID.getSetpoint().position);

        m_subsystem.driveVector(output_translation, angle, output_heading);


        // m_subsystem.setAllToState(new SwerveModuleState(speed*Constants.MAX_SPEED, Rotation2d.fromRadians(angle)));
        m_subsystem.updateOdometry();
    }
    @Override
    public void initialize(){
        System.out.println("START");
        translationPID.reset(getPositionError());
        headingPID.reset(m_subsystem.getPose().getRotation().getDegrees());
    }

    public double getPositionError(){
        Transform2d offset = targetPose.minus(m_subsystem.getPose());
        return -Math.hypot(offset.getX(), offset.getY());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        m_subsystem.setAllToState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
