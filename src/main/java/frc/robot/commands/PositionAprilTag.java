package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.Camera;

public class PositionAprilTag extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Camera m_camera;

    private final double xOffset;
    private final double yOffset;
    private final double angleOffset;

    private final PIDController translationPID;

    private final double TRANSLATION_KP = 4;
    private final double TRANSLATION_KI = 0;
    private final double TRANSLATION_KD = 0;

    public PositionAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera camera, double xOffset, double yOffset, double angleOffset) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_camera = camera;

        translationPID = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);

        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.angleOffset = angleOffset;
    }

    @Override
    public void execute() {
        if (!m_camera.hasTargets()) {
            System.out.println("No April Tag Detected");
            return;
        }

        Transform2d target2D = m_camera.getTarget2D();

        double yError = target2D.getY();
        double yOutput = translationPID.calculate(yError, yOffset);

        double xError = target2D.getX();
        double xOutput = translationPID.calculate(xError, xOffset);

        m_drivetrainSubsystem.driveXY(xOutput, yOutput, 0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
