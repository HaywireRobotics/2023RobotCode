package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveToTarget extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private Pose2d targetPose;
    private double translationAcc = 0.05;
    private double targetSpeed = 0.0;
    private double speed = 0.0;
    private double powerPerMeter = 1.0/3.0;
    private double powerPerDegree = 1.0/180.0;
    private double maxSpeed = 0.8;

    private final double TRANSLATION_KP = 0.04;
    private final double TRANSLATION_KI = 0;
    private final double TRANSLATION_KD = 0;
    private final double HEADING_KP = 0.04;
    private final double HEADING_KI = 0;
    private final double HEADING_KD = 0;

    private final PIDController translationPID;
    private final PIDController headingPID;
    
    public AutoDriveToTarget(DrivetrainSubsystem subsystem) {
        this(subsystem, new Pose2d());
    }
    public AutoDriveToTarget(DrivetrainSubsystem subsystem, Pose2d target) {
        this.m_subsystem = subsystem;
        this.targetPose = target;

        this.translationPID = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        this.headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        this.addRequirements(subsystem);
    }

    public void setTarget(Pose2d target){
        targetPose = target;
    }
    @Override
    public void execute() {
        m_subsystem.updateOdometry();

        Transform2d offset = targetPose.minus(m_subsystem.getPose());
        double offset_mag = Math.hypot(offset.getX(), offset.getY());
        double angle = Math.atan2(offset.getY(), offset.getX());

        double output_translation = translationPID.calculate(offset_mag, 0);
        // SwerveModuleState translate = new SwerveModuleState(output_translation*Constants.MAX_SPEED, Rotation2d.fromRadians(angle));


        // double rawSpeed = Math.min(offset_mag * powerPerMeter, maxSpeed);
        // speed += Math.min(rawSpeed-speed, translationAcc);
        // System.out.println("speed: "+speed);
        // SwerveModuleState translate = new SwerveModuleState(speed*Constants.MAX_SPEED, Rotation2d.fromRadians(angle));
        double currentHeading = m_subsystem.getPose().getRotation().getDegrees();
        double targetHeading = targetPose.getRotation().getDegrees();
        double angle_offset = targetHeading - currentHeading;
        double output_heading = headingPID.calculate(currentHeading, targetHeading);

        m_subsystem.driveVector(output_translation, angle, output_heading);


        // m_subsystem.setAllToState(new SwerveModuleState(speed*Constants.MAX_SPEED, Rotation2d.fromRadians(angle)));
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
