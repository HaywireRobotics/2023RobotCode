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

    private final double xOffset;
    private final double yOffset;
    private final double angleOffset;
    private final boolean onlyX;

    private final PIDController translationPID;

    private final double TRANSLATION_KP = 0.9;
    private final double TRANSLATION_KI = 0;
    private final double TRANSLATION_KD = 0.001;

    private final PIDController anglePID;

    private final double ANGLE_KP = 0.05;
    private final double ANGLE_KI = 0;
    private final double ANGLE_KD = 0.001;

    public PositionAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera camera, double xOffset, double yOffset, double angleOffset) {
        this(drivetrainSubsystem, camera, xOffset, yOffset, angleOffset, false);
    }

    public PositionAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera camera, double xOffset, double yOffset, double angleOffset, boolean onlyX) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_camera = camera;

        translationPID = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        anglePID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);

        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.angleOffset = angleOffset;
        this.onlyX = onlyX;

        m_drivetrainSubsystem.aligning = true;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (!m_camera.hasTargets()) {
            System.out.println("No April Tag Detected");
            return;
        }
        if (!m_camera.camera.isConnected()) {
            System.out.println("Camera Disconnected");
            return; 
        }

        Transform2d target2D = m_camera.getTarget2D();
        if (target2D == null) {
            System.out.println("target2D is null");
            return;
        }

        double yError = 0;
        double yOutput = 0;
        if (!onlyX) {
            yError = target2D.getY() - yOffset;
            yOutput = translationPID.calculate(yError, 0);
        } else {
            yOutput = 0;
        }

        double xError = target2D.getX() - xOffset;
        double xOutput = translationPID.calculate(xError, 0);

        SmartDashboard.putNumber("xError", xError);
        SmartDashboard.putNumber("xOutput", xOutput);

        double angleError = target2D.getRotation().getDegrees() - angleOffset;
        double angleOutput = anglePID.calculate(angleError, 0);

        if (xError < 0.05 && yError < 0.05 && angleError < 5) {
            m_drivetrainSubsystem.aligned = true;
        } else {
            m_drivetrainSubsystem.aligned = false;
        }

        m_drivetrainSubsystem.driveXY(xOutput, yOutput, angleOutput);
    }

    @Override
    public boolean isFinished() {
        m_drivetrainSubsystem.aligning = false;
        return false;
    }
}
