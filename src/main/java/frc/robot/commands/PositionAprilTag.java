package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.Camera;

public class PositionAprilTag extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Camera m_camera;

    private final double distOffset;
    private final double angleOffset;

    private final PIDController translationPID;

    private final double TRANSLATION_KP = 1;
    private final double TRANSLATION_KI = 0;
    private final double TRANSLATION_KD = 0.001;

    private final PIDController anglePID;

    private final double ANGLE_KP = 0.01;
    private final double ANGLE_KI = 0;
    private final double ANGLE_KD = 0.001;

    public PositionAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera camera, double distOffset, double angleOffset) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_camera = camera;

        translationPID = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        anglePID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);

        this.distOffset = distOffset;
        this.angleOffset = angleOffset;

        m_drivetrainSubsystem.aligning = true;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (!m_camera.hasTargets()) {
            System.out.println("No April Tag Detected");
            m_drivetrainSubsystem.driveXY(0, 0, 0);
            return;
        }
        if (!m_camera.camera.isConnected()) {
            System.out.println("Camera Disconnected");
            m_drivetrainSubsystem.driveXY(0, 0, 0);
            return; 
        }

        Transform2d target2D = m_camera.getTarget2D();
        if (target2D == null) {
            System.out.println("target2D is null");
            m_drivetrainSubsystem.driveXY(0, 0, 0);
            return;
        }

        double distError = target2D.getX() - distOffset;
        double distOutput = translationPID.calculate(distError, 0);

        SmartDashboard.putNumber("distError", distError);
        SmartDashboard.putNumber("distOutput", distOutput);

        double angleRaw = target2D.getRotation().getDegrees();
        double angleSign = (angleRaw/Math.abs(angleRaw));
        double angle = angleSign * (180 - Math.abs(angleRaw));
        double angleError = angle - angleOffset;
        double angleOutput = anglePID.calculate(angleError, 0);

        if (distError < 0.05 && angleError < 5) {
            m_drivetrainSubsystem.aligned = true;
        } else {
            m_drivetrainSubsystem.aligned = false;
        }

        m_drivetrainSubsystem.driveVector(distOutput, angle, angleOutput);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.aligning = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
